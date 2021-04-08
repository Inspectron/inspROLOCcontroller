#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"
#define DBG_BLOCK 0
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

namespace {

    const quint8  I2C_BUS                                   =     1;
    const quint8  LINEFINDER_I2C_HW_BASE_ADDRESS            = 8;//(0xFA >> 1);
    const qint16  LINEFINDER_I2C_ID                         = 0x0102;
    const qint16  LINEFINDER_AUTOMATIC_GAIN                 = 0x400;
    const qint16  BAD_DATA_READ                             = -2;
    const qint64  MIN_ROLOC_SIGNAL_STRENTH                 = 0x32;
    const qint64  MAX_BAD_READS                            = 4;
    const qint16  DEPTH_TYPE                               = 0x0200;

    // TODO none of these dec-> hex values are correct
    const int    LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED = 4;  // 0x0010;
    const int    LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    = 5;  // 0x0020;
    const int    LINEFINDER_PING_DATA_RETURNED              = 6;  // 0x0040;
    const int    LINEFINDER_CAL_FAILURE_VALUE               = 253;

    const int    N_MULTI_LF_DEPTH_SAMPLE                    = 5;
    const int    N_MULTI_LF_DEPTH_DELTA                     = 6;
    const int    N_MULTI_LF_DEPTH_OK                        = 2;

    const int    LINEFINDER_NO_DATA                         = 0xFFFF;

    const int    TIMER_DATA_POLLING_PERIOD                  = 333;          // period between data polls in ms
    const int    TIMER_3SECONDS                             = 3000;
    const int    FREQ_SET_TIMER_INTERVAL                    = 100;
    const float    INCHES_TO_METERS                         = 0.0254;
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
, mFrequency(ROLOC::eFREQ_512HZ_SONDE)
, mNumSamples(0)
, mDepthAccumulator()
, mInfoPacket(*new RolocInfoPacket())
, mpRolocDataPollingTimer(NULL)
, mCurrentState(ROLOC::eSTATE_DISCONNECTED)
, mpFreqencySetTimer(NULL)
, mDisplayRetry(0)
, mBadReadCount(0)
, mPrevPresent(true)
, mPendingFreq(mFrequency)
{}

/**
 * @brief ROLOCcontroller::~ROLOCcontroller - dtor. destroy any created objects
 */
ROLOCcontroller::~ROLOCcontroller()
{
    delete &mInfoPacket;
    delete mpRolocDataPollingTimer;
    delete mpFreqencySetTimer;
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
}



/**
 * @brief ROLOCcontroller::initROLOC - init the roloc upon plugging in
 */
void ROLOCcontroller::initROLOC()
{
    qDebug() << "ROLOC plugged in. set volume and freq to defaults";
    rolocSetVolume(ROLOC::eVOLUME_OFF);
    rolocSetParameters(ROLOC::eMODE_GET_SIGNAL_STRENGTH, ROLOC::eFREQ_512HZ_SONDE);

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
#if DBG_BLOCK
    // dbg print out state changes

    static ROLOC::eSTATE prevState = ROLOC::eSTATE_DISCONNECTED;

    if (prevState != mCurrentState)
    {
#if DBG_BLOCK
        qDebug() << "state change " << getString(prevState) << "--->" << getString(mCurrentState);
#endif
        prevState = mCurrentState;
    }

#endif
        // update hardware existence
        bool present = rolocHardwarePresent();
        if (present == false)
         {
           if (mBadReadCount >= MAX_BAD_READS)
           {
              mPrevPresent = present;
           }
           else
           {
             mBadReadCount++;
             mPrevPresent = true;
           }
         }
         else {
            mBadReadCount = 0; // reset
            mPrevPresent = present;
         }

         mDbusHandler.sendPresent(mPrevPresent);

        if (mCurrentMode == ROLOC::eMODE_GET_SIGNAL_STRENGTH)
        {
            mDbusHandler.sendPresent(present);
        }
        else
        {
            present = true;
            mDbusHandler.sendPresent(present);
        }
        if (present)
        {
            if (mCurrentState == ROLOC::eSTATE_DISCONNECTED)
            {
                rolocBusy(ROLOC::eSTATE_INITIALIZING);
            }
        }
        else
        {
            // roloc is disconnected
            mCurrentState = ROLOC::eSTATE_DISCONNECTED;
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
            sendDataReport();
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
                sendDataReport();
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
#if DBG_BLOCK
        qDebug() << "rolocHardwarePresent:   Good eCMD_GET_ID Data ";
#endif
    }
    else
    {
        present = false;
#if DBG_BLOCK
        qDebug() << "rolocHardwarePresent:   Bad eCMD_GET_ID Data ";
#endif
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
        qDebug() << "rolocGetData:*** *** Good Good Good *** ***";
        // debug print out the packet
        qWarning().noquote() << mInfoPacket.toString();
        qWarning().noquote() << "rolocGetData:  The type is ---> " << mInfoPacket.getType();
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

#if 0
    // TODO figure out what this is doing
    // TODO trigger the "busy" state if the mode changed
    if(mode != mCurrentMode || mEnabled == false)
    {
        mCurrentMode = mode;
        startModeChange();
    }
#endif

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
void ROLOCcontroller::sendDataReport()
{
    if (mROLOCsignalStrenth > MIN_ROLOC_SIGNAL_STRENTH)
    {
        mDbusHandler.sendDataReport(
                mCurrentMode,
                mFrequency,
                mROLOCsignalStrenth,
                mROLOCdepthMeasurement,
                mInfoPacket.getArrow(),
                (mCurrentState != ROLOC::eSTATE_DISCONNECTED)); // TODO switch the dbus signal to an enum
    }
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
    sendDataReport();
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
    qDebug() << "write status --> " << status;
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
