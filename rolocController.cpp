#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"
#include "gpio.hpp"
#define DBG_BLOCK 0
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

namespace {
    const quint8  I2C_BUS                                   = 1;
    const quint8  LINEFINDER_I2C_HW_BASE_ADDRESS            = 0x08;
    const qint16  LINEFINDER_I2C_ID                         = 0x0102;
    const qint16  LINEFINDER_AUTOMATIC_GAIN                 = 0x400;
    const qint16  BAD_DATA_READ                             = -2;
    const qint64  MIN_ROLOC_SIGNAL_STRENTH                  = 0x16;
    const qint64  MAX_BAD_READS                             = 30;
    const qint16  DEPTH_TYPE                                = 0x0200;

    // TODO none of these dec-> hex values are correct
    const int    LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED = 4;  // 0x0010;
    const int    LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    = 5;  // 0x0020;
    const int    LINEFINDER_PING_DATA_RETURNED              = 6;  // 0x0040;
    const int    LINEFINDER_CAL_FAILURE_VALUE               = 253;

    const int    N_MULTI_LF_DEPTH_SAMPLE                    = 5;
    const int    N_MULTI_LF_DEPTH_DELTA                     = 6;
    const int    N_MULTI_LF_DEPTH_OK                        = 2;

    const int    LINEFINDER_NO_DATA                         = 0xFFFF;

    const int    TIMER_DATA_POLLING_PERIOD                  = 500; // period between data polls in ms
    const int    TIMER_3SECONDS                             = 3000;
    const int    FREQ_SET_TIMER_INTERVAL                    = 100;
    const float  INCHES_TO_METERS                           = 0.0254;

    const int    ROLOC_BAD_PACKET_TIMEOUT_VALUE             = 3000;//3sec
    const int    POWER_RESET_WAIT_TIME                      = 250 * 1000; //250ms
    const int    ROLOC_5V_GPIO_NUMBER                       = 229;

    GPIO_CTRL::tGPIO POWER_GPIO = { ROLOC_5V_GPIO_NUMBER, GPIO_CTRL::OUTPUT, GPIO_CTRL::EDGE_BOTH, false, GPIO_CTRL::HIGH };
}

/**
 * @brief ROLOCcontroller::ROLOCcontroller - ctor. construct any objects
 */
ROLOCcontroller::ROLOCcontroller()
: m_i2cBus()
, mI2cAddr(LINEFINDER_I2C_HW_BASE_ADDRESS)
, mDbusHandler(*new InspROLOCControllerDbus())
, mCurrentMode(ROLOC::eMODE_GET_SIGNAL_STRENGTH)
, mROLOCsignalStrenth(0)
, mROLOCdepthMeasurement(0.0)
, mCurrVolume(ROLOC::eVOLUME_OFF)
, mFrequency(ROLOC::eFREQ_50HZ_PASSIVE)
, mNumSamples(0)
, mDepthAccumulator()
, mInfoPacket(*new RolocInfoPacket())
, mpRolocDataPollingTimer(NULL)
, mpFreqencySetTimer(NULL)
, mpDisconnectTimer(NULL)
, mCurrentState(ROLOC::eSTATE_DISCONNECTED)
, mPendingFreq(mFrequency)
{
    gpio::configureIO(POWER_GPIO);
}

/**
 * @brief ROLOCcontroller::~ROLOCcontroller - dtor. destroy any created objects
 */
ROLOCcontroller::~ROLOCcontroller()
{
    delete &mInfoPacket;
    delete mpRolocDataPollingTimer;
    delete mpFreqencySetTimer;
    delete mpDisconnectTimer;
}

/**
 * @brief ROLOCcontroller::init - init the class
 */
void ROLOCcontroller::init()
{
    // init the i2c
    m_i2cBus.i2c_setDevice(I2C_BUS);

    // init dbus
    mDbusHandler.init();

    //init the bad packet timeout
    mpDisconnectTimer = new QTimer();
    QObject::connect(mpDisconnectTimer, &QTimer::timeout, [this]()
    {
        qWarning() << "No Response from ROLOC, consider it disconnected";
        mDbusHandler.sendPresent(false);
        //verify that the output state is for sure high if were done with the roloc
        gpio::setOutputState(POWER_GPIO.gpio, GPIO_CTRL::HIGH);
    });
    mpDisconnectTimer->setSingleShot(true);
    mpDisconnectTimer->setInterval(ROLOC_BAD_PACKET_TIMEOUT_VALUE);

    // init the freq change timer
    mpFreqencySetTimer = new QTimer();
    QObject::connect(mpFreqencySetTimer, SIGNAL(timeout()), this, SLOT(onFreqSetTimerExpired()) );
    mpFreqencySetTimer->setInterval(FREQ_SET_TIMER_INTERVAL);

    // init the polling timer
    mpRolocDataPollingTimer = new QTimer();
    QObject::connect(mpRolocDataPollingTimer, SIGNAL(timeout()), this, SLOT(pollROLOC()));
    mpRolocDataPollingTimer->setInterval(TIMER_DATA_POLLING_PERIOD);

    mpRolocDataPollingTimer->start();

    // connect sigs/slots
    //dbus signals -> controller
    QObject::connect(&mDbusHandler,   SIGNAL(getDataReportHandler()),
                     this,  SLOT(getDataReportHandler()) );
    QObject::connect(&mDbusHandler,   SIGNAL(setVolumeHandler(ROLOC::eLINEFINDER_VOLUME)),
                     this,  SLOT(setVolumeHandler(ROLOC::eLINEFINDER_VOLUME)) );
    QObject::connect(&mDbusHandler,   SIGNAL(setParametersHandler(ROLOC::eLINEFINDER_MODE, ROLOC::eLINEFINDER_FREQ)),
                     this,  SLOT(setParametersHandler(ROLOC::eLINEFINDER_MODE, ROLOC::eLINEFINDER_FREQ)) );
    QObject::connect(&mDbusHandler,   SIGNAL(requestSetFreq(ROLOC::eLINEFINDER_FREQ)), this, SLOT(setFrequencyHandler(ROLOC::eLINEFINDER_FREQ)) );
    QObject::connect(&mDbusHandler,   SIGNAL(requestSetMode(ROLOC::eLINEFINDER_MODE)), this, SLOT(setModeHandler(ROLOC::eLINEFINDER_MODE))      );

    mDbusHandler.sendPresent(false);
}

/**
 * @brief ROLOCcontroller::initROLOC - init the roloc upon plugging in
 */
void ROLOCcontroller::initROLOC()
{
    qDebug() << "ROLOC plugged in. set volume and freq to defaults";
    rolocSetVolume(ROLOC::eVOLUME_OFF);
    rolocSetParameters(ROLOC::eMODE_GET_SIGNAL_STRENGTH, ROLOC::eFREQ_50HZ_PASSIVE);

    // move to the next state
    rolocBusy(ROLOC::eSTATE_OPERATING);
}

/**
 * @brief ROLOCcontroller::rolocBusy - Start a mode change, when the timer is complete it will transition to the next state
 */
void ROLOCcontroller::rolocBusy(ROLOC::eSTATE nextState)
{
    mCurrentState = ROLOC::eSTATE_BUSY;

    QTimer::singleShot(TIMER_3SECONDS, [&, nextState]()
    {
#if DBG_BLOCK
        qDebug() << "3.0 second timer expired.  Mode Change Complete nextState -> " << nextState;
#endif
        mCurrentState = nextState;
    });
}

/**
 * @brief ROLOCcontroller::pollROLOC - function to poll the roloc
 */
void ROLOCcontroller::pollROLOC()
{
    // update hardware existence
    bool present = rolocHardwarePresent();

    if((!present) && (mCurrentState != ROLOC::eSTATE_DISCONNECTED))
    {
        qWarning() << "ROLOC is experiencing some difficulties";
        //the roloc is all of a sudden not responding while we were in some
        //form of connected state. set it to disconected and start the timer
        mCurrentState = ROLOC::eSTATE_DISCONNECTED;
        mpDisconnectTimer->start();
        //try to reset the ROLOC to see if that fixes it
        resetRoloc();
    }
    else if((present) && (mCurrentState == ROLOC::eSTATE_DISCONNECTED))
    {
        if(mpDisconnectTimer->isActive())
        {
            //it the timers already going then its been initialized already,
            //just set it back to the operating state
            mCurrentState = ROLOC::eSTATE_OPERATING;
        }
        else
        {
            //the roloc is freshly present since the timer wasnt started
            //so set it to its initializing state
            rolocBusy(ROLOC::eSTATE_INITIALIZING);
            mDbusHandler.sendPresent(true);
        }
        mpDisconnectTimer->stop();
    }

    switch (mCurrentState)
    {
    case ROLOC::eSTATE_BUSY:
        // just keep waiting
        break;
    case ROLOC::eSTATE_INITIALIZING:
        initROLOC();
        break;
    case ROLOC::eSTATE_OPERATING:
        processRolocData();
        break;
    case ROLOC::eSTATE_DISCONNECTED:
    default:
        // do nothing.
        break;
    }
}

/**
 * @brief ROLOCcontroller::processRolocData - get and process the roloc data
 */
void ROLOCcontroller::processRolocData()
{
#if 0
    // TODO keep this for records with the accumulator
    if(mHardwarePresent /*&& mbModeChangeComplete */ /*&& mNumSamples < N_MULTI_LF_DEPTH_DELTA*/) // TODO uncomment the block comment
#endif
    qint16 rolocData = rolocGetData();

    if (rolocData != BAD_DATA_READ)
    {
        if(mCurrentMode == ROLOC::eMODE_GET_SIGNAL_STRENGTH)
        {
            // update the sig strength
            mROLOCsignalStrenth = rolocData;
#if DBG_BLOCK
            qDebug() << "sig strength = " << mROLOCsignalStrenth; // TODO verify if is this working ?
#endif
            // clear the depth measurements
            mDepthAccumulator.clear();
            sendDataReport(false);
        }
        else if (mCurrentMode == ROLOC::eMODE_GET_DEPTH_MEASUREMENT)
        {
            mDepthAccumulator.append(rolocData);
            mNumSamples++;

            if(mNumSamples > N_MULTI_LF_DEPTH_SAMPLE)
            {
                QList<quint8> acceptedDepthReadings;
                float scaleOfElimination = static_cast<float>(N_MULTI_LF_DEPTH_DELTA);

                double mean = getMean(mDepthAccumulator);
                double stdDev = qSqrt(getVariance(mDepthAccumulator));
#if DBG_BLOCK
                qDebug() << "*************************** bit set  -> " <<  CHECK_BIT(rolocData, 1)  << "*********************";
                qDebug() << "smaple size: " << mNumSamples;
                qDebug() << "Mean: " << mean;
                qDebug() << "Standard Dev.: " << stdDev;
                qDebug() << "******************************************************************************";
#endif
                for(quint8 i = 0; i < mDepthAccumulator.count(); i++)
                {
                    quint8 isLessThanLowerBound = mDepthAccumulator.at(i) < mean - stdDev * scaleOfElimination;
                    quint8 isGreaterThanUpperBound = mDepthAccumulator.at(i) > mean + stdDev * scaleOfElimination;
                    quint8 isOutOfBounds = isLessThanLowerBound || isGreaterThanUpperBound;

                    if(!isOutOfBounds)
                    {
                        acceptedDepthReadings.append(mDepthAccumulator.at(i));
                    }
                }

                double finalReading = getMean(acceptedDepthReadings);
                mROLOCdepthMeasurement = finalReading  * INCHES_TO_METERS;
                sendDataReport(true);
                mNumSamples = 0;
                mCurrentMode = ROLOC::eMODE_GET_SIGNAL_STRENGTH;
                rolocSetParameters (mCurrentMode, mFrequency);
            }
        }
    }
}

/**
 * @brief ROLOCcontroller::rolocHardwarePresent - check if the roloc hardware is attached
 */
bool ROLOCcontroller::rolocHardwarePresent()
{
    bool present = true;
    qint16 data;

    data = m_i2cBus.i2c_readWord(mI2cAddr, ROLOC::eCMD_GET_ID);

#if DBG_BLOCK
    qDebug() << "rolocHardwarePresent: data --> "  <<  data;
#endif

    if(data == LINEFINDER_I2C_ID)
    {
        present = true;
    }
    else
    {
        present = false;
    }
    return present;
}

/**
 * @brief ROLOCcontroller::rolocGetData - get info data from the roloc
 */
quint16 ROLOCcontroller::rolocGetData()
{
    // read i2c data
    qint16 retValue = 0;

    qint16 data = m_i2cBus.i2c_readWord(mI2cAddr, ROLOC::eCMD_INFO);

    // set the incoming dat
    if (data != BAD_DATA_READ)
    {
        mInfoPacket.set(data);
#if DBG_BLOCK
        // debug print out the packet
        qWarning().noquote() << mInfoPacket.toString();
#endif
        retValue =    mInfoPacket.getData();
    }
    else
    {
#if DBG_BLOCK
        qDebug() << "Error Bad read in rolocGetData   data --> " << data;
#endif
        retValue = BAD_DATA_READ;
    }
    return retValue;
}

/**
 * @brief ROLOCcontroller::rolocSetParameters
 */
void ROLOCcontroller::rolocSetParameters(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ frequency)
{
    // start w/ the automatic gain
    int16_t data = LINEFINDER_AUTOMATIC_GAIN;

    // add in the mode & frequency
    data |= (mode << 8);  // todo:: double check that the shift operation provides the correct results!
    data |= frequency;
    int status = m_i2cBus.i2c_writeWord(mI2cAddr, ROLOC::eCMD_INFO, data);
    if (status != 0)
    {
        qWarning() << "i2c error (" << status << ")" << strerror(errno);
    }
    else
    {
        // successful transaction. update the member vars
        mFrequency = frequency;
        mCurrentMode = mode;
    }

    // update dbus
    mDbusHandler.sendParameters(mCurrentMode, mFrequency);
}

/**
 * @brief ROLOCcontroller::rolocSetVolume - make the volume change in the roloc
 * @param vol - volume value
 */
void ROLOCcontroller::rolocSetVolume(ROLOC::eLINEFINDER_VOLUME vol)
{
    int status = m_i2cBus.i2c_writeWord(mI2cAddr, ROLOC::eCMD_VOLUME, vol);
    if (status != 0)
    {
        qWarning() << "i2c error (" << status << ")" << strerror(errno);
    }
    else
    {
        // no error, assume the update was successful
        mCurrVolume = vol;
    }

    // update dbus
    mDbusHandler.sendVolume(mCurrVolume);
}

/**
 * @brief ROLOCcontroller::getMean
 * @param values
 * @return
 */
double ROLOCcontroller::getMean(QList<quint8> values)
{
    quint16 sum = 0;
    for (quint8 i = 0; i < values.count(); i++)
    {
        sum += values.at(i);
    }

    return (sum / values.count());
}

/**
 * @brief ROLOCcontroller::getVariance
 * @param values
 * @return
 */
double ROLOCcontroller::getVariance(QList<quint8> values)
{
    double mean = getMean(values);
    double temp = 0;

    for(int i = 0; i < values.count(); i++)
    {
        temp += (values.at(i) - mean) * (values.at(i) - mean);
    }

    return temp / (values.count() - 1);
}

// ===========================================
// roloc -> dbus functions
// ===========================================
/**
 * @brief ROLOCcontroller::sendDataReport -- send data report
 *        over dbus
 */
void ROLOCcontroller::sendDataReport(bool isROLOC)
{
    Q_UNUSED(isROLOC);

#if DBG_BLOCK
        qDebug() << "**********************************************************";
        qDebug() << "mROLOCdepthMeasurement ---> " << mROLOCdepthMeasurement;
        qDebug() << "mFrequency ---> " << mFrequency;
        qDebug() << "mCurrentMode ---> " << mCurrentMode;
        qDebug() << "**********************************************************";
#endif
    mDbusHandler.sendDataReport(
                mCurrentMode,
                mFrequency,
                mROLOCsignalStrenth,
                mROLOCdepthMeasurement,
                mInfoPacket.getArrow(),
                (mCurrentState != ROLOC::eSTATE_DISCONNECTED)); // TODO switch the dbus signal to an enum
}

/**
 * @brief ROLOCcontroller::sendDepthReport -- send depth report
 *        over dbus
 */
void ROLOCcontroller::sendDepthReport()
{
    mDbusHandler.sendDataReport(
                ROLOC::eMODE_GET_DEPTH_MEASUREMENT,
                mFrequency,
                mROLOCsignalStrenth,
                mROLOCdepthMeasurement,
                mInfoPacket.getArrow(),
                (mCurrentState != ROLOC::eSTATE_DISCONNECTED)); // TODO switch the dbus signal to an enum
}

// ===========================================
// DBus to roloc handler functions
// ===========================================
/**
 * @brief ROLOCcontroller::getDataReportHandler -- request data
 *        report
 */
void ROLOCcontroller::getDataReportHandler()
{
    sendDataReport(false);
}

/**
 * @brief ROLOCcontroller::setVolumeHandler
 */
void ROLOCcontroller::setVolumeHandler(ROLOC::eLINEFINDER_VOLUME vol)
{
    // change the roloc volume
    rolocSetVolume(vol);
}

/**
 * @brief ROLOCcontroller::setMode - slot callback to set the
 *        operating mode of the ROLOC
 */
void ROLOCcontroller::setParametersHandler(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq)
{
    rolocSetParameters(mode, freq);
}

/**
 * @brief ROLOCcontroller::setFrequencyHandler - set the frequency of the roloc
 * @param freq - frequency enum
 */
void ROLOCcontroller::setFrequencyHandler(ROLOC::eLINEFINDER_FREQ freq)
{
    // set the pending, and star the hysteresis timer
    mPendingFreq = freq;
    qDebug() << "Setting ROLOC Frequency to:" << mFrequency;
    mpFreqencySetTimer->start();
}

/**
 * @brief ROLOCcontroller::onFreqSetTimerExpired - timer callback when we should set the frequency to ensure
 * the ROLOC doesnt get mixed up
 */
void ROLOCcontroller::onFreqSetTimerExpired()
{
    if (mPendingFreq != mFrequency)
    {
        rolocSetParameters(mCurrentMode, mPendingFreq);
    }

    // stop the timer
    mpFreqencySetTimer->stop();
}

/**
 * @brief ROLOCcontroller::rolocSetDepthMode - set depth mode
 * @param mode - mode enum
 */
void ROLOCcontroller::rolocSetDepthMode(ROLOC::eLINEFINDER_MODE mode)
{
    Q_UNUSED(mode);
    qint16 depthPacket = DEPTH_TYPE | mFrequency ;
    int status = m_i2cBus.i2c_writeWord(mI2cAddr, ROLOC::eCMD_INFO, depthPacket);
    if (status == BAD_DATA_READ)
    {
#if DBG_BLOCK
        qDebug() << "i2c_writeWord status --> " << status;
#endif
    }
    mCurrentMode = ROLOC::eMODE_GET_DEPTH_MEASUREMENT;
}

/**
 * @brief ROLOCcontroller::setModeHandler - set the mode of the roloc
 * @param mode - mode enum

 */
void ROLOCcontroller::setModeHandler(ROLOC::eLINEFINDER_MODE mode)
{
    if (ROLOC::eMODE_GET_DEPTH_MEASUREMENT == mode)
    {
        rolocSetDepthMode(mode);
    }
    else
    {
        rolocSetParameters(mode, mFrequency);
    }
}

void ROLOCcontroller::resetRoloc()
{
    gpio::setOutputState(POWER_GPIO.gpio, GPIO_CTRL::LOW);
    usleep(POWER_RESET_WAIT_TIME);
    gpio::setOutputState(POWER_GPIO.gpio, GPIO_CTRL::HIGH);
}
