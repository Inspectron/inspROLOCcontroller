#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"

#define DBG_BLOCK 0

namespace {

    const quint8  I2C_BUS                                   =      1;
    const quint8  LINEFINDER_I2C_HW_BASE_ADDRESS            =   0xFA;
    const qint16  LINEFINDER_I2C_ID                         =  0x0102;

    const int    LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED = 4; //  0x0010;
    const int    LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    = 5; // 0x0020;
    const int    LINEFINDER_PING_DATA_RETURNED              = 6; // 0x0040;
    const int    LINEFINDER_CAL_FAILURE_VALUE               = 253;

    const int    N_MULTI_LF_DEPTH_SAMPLE                    = 5;
    const int    N_MULTI_LF_DEPTH_DELTA                     = 6;
    const int    N_MULTI_LF_DEPTH_OK                        = 2;

    const int    LINEFINDER_NO_DATA                         = 0xFFFF;

    const int    PARAM_LF_FREQ_512HZ_SONDE                  = (0 << 11);
    const int    PARAM_LF_FREQ_640HZ_SONDE                  = (1 << 11);
    const int    PARAM_LF_FREQ_50HZ_PASSIVE                 = (2 << 11);
    const int    PARAM_LF_FREQ_60HZ_PASSIVE                 = (3 << 11);
    const int    PARAM_LF_FREQ_32_5KHZ_ACTIVE               = (8 << 11);
    const int    PARAM_LF_FREQ_32_5KHZ_PASSIVE              = (9 << 11);

    const int    TIMER_DATA_POLLING_PERIOD                  = 500;          // period between data polls in ms
    const int    TIMER_3SECONDS                             = 3000;

}

/**
 * @brief ROLOCcontroller::ROLOCcontroller - ctor. construct any objects
 */
ROLOCcontroller::ROLOCcontroller()
: m_i2cBus()
, mI2cAddr(0)
, mDbusHandler(*new InspROLOCControllerDbus())
, mEnabled(false)
, mHardwarePresent(false)
, mCurrentMode(ROLOC::eMODE_GET_SIGNAL_STRENGTH)
, mROLOCsignalStrenth(0)
, mROLOCdepthMeasurement(0.0)
, mCurrVolume(ROLOC::eVOLUME_OFF)
, mFrequency(PARAM_LF_FREQ_512HZ_SONDE)
, mbModeChangeComplete(true)
, mNumSamples(0)
, mDepthAccumulator()
, mRolocArrows(*new ROLOCArrows())
, mpRolocDataPollingTimer(NULL)
{
}

/**
 * @brief ROLOCcontroller::~ROLOCcontroller - dtor. destroy any created objects
 */
ROLOCcontroller::~ROLOCcontroller()
{
    delete &mRolocArrows;
}

/**
 * @brief ROLOCcontroller::init - init the class
 */
void ROLOCcontroller::init()
{
    // init the i2c
    m_i2cBus.i2c_setDevice(I2C_BUS);
    mI2cAddr = (LINEFINDER_I2C_HW_BASE_ADDRESS >> 1);

    // init dbus
    mDbusHandler.init();

    // init the polling timer
    mpRolocDataPollingTimer = new QTimer();
    connect(mpRolocDataPollingTimer, SIGNAL(timeout()), this, SLOT(pollROLOC()));
    mpRolocDataPollingTimer->setInterval(TIMER_DATA_POLLING_PERIOD);

    rolocHardwarePresent();
    qDebug() << "Hardware Present: " << mHardwarePresent;
    rolocSetVolume(mCurrVolume);
    rolocSetParameters(ROLOC::eMODE_GET_SIGNAL_STRENGTH, mFrequency);

    // connect sigs/slots
    //dbus signals -> controller
    QObject::connect(&mDbusHandler,   SIGNAL(getDataReportHandler()),         this,   SLOT(getDataReportHandler()) );
    QObject::connect(&mDbusHandler,   SIGNAL(setVolumeHandler(int)),          this,   SLOT(setVolumeHandler(int)) );
    QObject::connect(&mDbusHandler,   SIGNAL(setParametersHandler(int, int)), this,   SLOT(setParametersHandler(int, int)) );

    stop();    // don't start until active mode set
}

void ROLOCcontroller::start()
{
    QTimer::singleShot(TIMER_3SECONDS, this, SLOT(modeChangeComplete()));
    mpRolocDataPollingTimer->start();
    mbModeChangeComplete = false;
    mEnabled = true;
}

void ROLOCcontroller::stop()
{
    mbModeChangeComplete = false;
    mEnabled = false;       // TODO WHAT THE FUCK DOES ENABLED MEAN?
}

/**
 * @brief ROLOCcontroller::pollROLOC - function to poll the roloc
 */
void ROLOCcontroller::pollROLOC()
{
    bool hw = mHardwarePresent;

    // TODO: THIS TIMER CODE STARTS ITSELF. SO ON BOOT IT WONT WORK. GOOD JOB.
    // ITS ALMOST A FUCKING INFINITE LOOP.

    rolocHardwarePresent();
    if(mHardwarePresent != hw)
    {
        if(mHardwarePresent)
        {
            start();
        }
        else
        {
            stop();
        }
    }
    if(mHardwarePresent && mbModeChangeComplete && mEnabled /*&& mNumSamples < N_MULTI_LF_DEPTH_DELTA*/)
    {
        qint16 rolocData = rolocGetData();
        //qDebug() << "ROLOC DATA: " << rolocData;

        if(mCurrentMode == ROLOC::eMODE_GET_SIGNAL_STRENGTH)
        {
            // update the sig strength
            mROLOCsignalStrenth = rolocData;

            qDebug() << "sig strength = " << mROLOCsignalStrenth; // TODO verify if is this working ?

            // clear the depth measurements
            mDepthAccumulator.clear();
        }
        else
        {
            mDepthAccumulator.append(rolocData);
            mNumSamples++;

            if(mNumSamples > N_MULTI_LF_DEPTH_SAMPLE)
            {
                QList<quint8> acceptedDepthReadings;
                float scaleOfElimination = (float)N_MULTI_LF_DEPTH_DELTA;

                double mean = getMean(mDepthAccumulator);
                double stdDev = qSqrt(getVariance(mDepthAccumulator));

                qDebug() << "Mean: " << mean;
                qDebug() << "Standard Dev.: " << stdDev;

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
                qDebug() << finalReading;
                mROLOCdepthMeasurement = finalReading;
            }
        }
        sendDataReport();
    }
}

// TODO WHAT THE FUCK DOES THIS EVEN MEAN. WHY IS THERE A MYTHICAL 3.5 SECOND TIMER
void ROLOCcontroller::modeChangeComplete()
{
    mbModeChangeComplete = true;
    qDebug() << "3.5 second timer expired.  Mode Change Complete";
}

/**
 * @brief ROLOCcontroller::rolocHardwarePresent
 */
void ROLOCcontroller::rolocHardwarePresent()
{
    qint16 data;
    // TODO WHAT. THE. FUCK. fix this.
    // THIS LOOKS LIKE IT WAS IMPLEMENTED BY SOMEONE WITH A HEAD INJURY
    // JUST RETURN THE DAMN BOOL

    data = m_i2cBus.i2c_readWord(mI2cAddr, ROLOC::eCMD_GET_ID);

    if(data != LINEFINDER_I2C_ID)
    {
        qWarning() << "Could not read ID from ROLOC Hardware. data = " << data;
        mHardwarePresent = false;
    }
    else
    {
        mHardwarePresent = true;
    }

    mDbusHandler.sendPresent(mHardwarePresent);
}

/**
 * @brief ROLOCcontroller::rolocGetData
 * @return
 */
qint16 ROLOCcontroller::rolocGetData()
{
    qint16 data;
    data = m_i2cBus.i2c_readWord(mI2cAddr, ROLOC::eCMD_INFO);

    qDebug() << "raw data: " << hex << data;

    if(data >= 0)
    {
        quint8 statusByte = ((data & 0xFF00) >> 8);

        quint8 depthORCalTest = (statusByte >> LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED) & 1; //  (statusByte & (LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED));
        quint8 calORBalance   = (statusByte >> LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED) & 1;    //  (statusByte & (LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED   ));
        quint8 ping           = (statusByte >> LINEFINDER_PING_DATA_RETURNED) & 1;              //  (statusByte & (LINEFINDER_PING_DATA_RETURNED             ));
        quint8 specialDataReceived = (depthORCalTest || calORBalance || ping);

        Q_UNUSED(specialDataReceived);

#if DBG_BLOCK
        qDebug() << "statusByte         : " << statusByte;
        qDebug() << "depthORCalTest     : " << depthORCalTest;
        qDebug() << "calORBalance       : " << calORBalance;
        qDebug() << "ping               : " << ping;
        qDebug() << "specialDataReceived: " << specialDataReceived;
#endif

        if(depthORCalTest || calORBalance)
        {
            data = data & 0x00FF;
        }
        else
        {
            // this is either the sig strength or the depth
            // this range limits the values 0 -> 0xF9 (240)
            data = data & 0x00FF;
            if(data > 240) data = 240;
            if(data <  10) data = 0;

            // set the arrowsS
            mRolocArrows.set( statusByte );

            // oddly enough the device will provide multiple arrow indicators
            // therefore we filter the results such that if the data is zero
            // then indicate a center arrow, else priority to right arrow if left
            // and right are true.  Old code for displaying the arrow was:
            // e.g. screen = (mCenterArrow || mNoArrow || data == 0) ? dislay center arrow : ((mRightArrow) ? display rigth : display left
#if DBG_BLOCK
            qDebug() << mRolocArrows.getString();
#endif
        }
    }
    else
    {
        qWarning() << "Could not read ROLOC I2C Status";
    }

    return data;
}

/**
 * @brief ROLOCcontroller::rolocSetParameters
 */
void ROLOCcontroller::rolocSetParameters(quint16 mode, int frequency)
{
    int16_t data = 0x0400;  // TODO : WHAT THE FUCK IS THIS MAGIC NUMBER ?

    data |= (mode << 8);  // todo:: double check that the shift operation provides the correct results!
    data |= frequency;    // TODO GARBAGE WILL ACTUALY CLIP THE FREQ VALUE since its 16 bit. WHAT THE FUCK

    int status = m_i2cBus.i2c_writeWord(mI2cAddr, ROLOC::eCMD_INFO, data);

    if (status != 0)
    {
        qWarning() << "i2c error (" << status << ")" << strerror(errno);
    }

    mFrequency = frequency;
    if(mode != mCurrentMode || mEnabled == false)
    {
        mCurrentMode = mode;
        start();
    }
}

/**
 * @brief ROLOCcontroller::rolocSetVolume
 * @param data
 */
void ROLOCcontroller::rolocSetVolume(int16_t data)
{
    if(data < ROLOC::eVOLUME_OFF)
    {
        data = ROLOC::eVOLUME_OFF;
    }
    else if(data > ROLOC::eVOLUME_HIGH)
    {
        // TODO -- check how to handle out of range value
        data = ROLOC::eVOLUME_OFF;
    }

    int status = m_i2cBus.i2c_writeWord(mI2cAddr, ROLOC::eCMD_VOLUME, data);
    if (status != 0)
    {
        qWarning() << "i2c error (" << status << ")" << strerror(errno);
    }

    mCurrVolume = data;
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
#if DBG_BLOCK
    qDebug("Report: mode=%d freq=%d signal=%d depth=%0.2f arrow=%d hw=%d",
           getModeDBUS(), getFrequencyDBUS(),
            mROLOCsignalStrenth, mROLOCdepthMeasurement, getArrowDBUS(), mHardwarePresent);
#endif
            mDbusHandler.sendDataReport(
            getModeDBUS(),
            getFrequencyDBUS(),
            mROLOCsignalStrenth,
            mROLOCdepthMeasurement,
            mRolocArrows.getDBusValue(),
            mHardwarePresent);
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
void ROLOCcontroller::setVolumeHandler(int lvl)
{
    qint8 hwvol;

    switch(lvl)
    {
    case ROLOC_DBUS_API::eROLOC_VOLUME_OFF:
        hwvol = ROLOC::eVOLUME_OFF;
        break;
    case ROLOC_DBUS_API::eROLOC_VOLUME_MED:
        hwvol = ROLOC::eVOLUME_MED;
        break;
    case ROLOC_DBUS_API::eROLOC_VOLUME_HIGH:
        hwvol = ROLOC::eVOLUME_HIGH;
        break;
    default:
        hwvol = mCurrVolume;
        break;
    }

    rolocSetVolume(hwvol);
    mDbusHandler.sendVolume(mCurrVolume);
}

/**
 * @brief ROLOCcontroller::setMode - slot callback to set the
 *        operating mode of the ROLOC
 */
void ROLOCcontroller::setParametersHandler(int mode, int freq)
{
    qint8 hwmode;
    qint16 hwfreq;


    // TODO: LETS FUCKING CONVERT ENUMS BACK AND FORTH RATHER THAN SIMPLY
    // MAKE THEM THE SAME
    switch(mode)
    {
    case ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH:
        hwmode = ROLOC::eMODE_GET_SIGNAL_STRENGTH;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT:
        hwmode = ROLOC::eMODE_GET_DEPTH_MEASUREMENT;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION:
        hwmode = ROLOC::eMODE_CALIBRATION;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST:
        hwmode = ROLOC::eMODE_CALIBRATION_TEST;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_BALANCE:
        hwmode = ROLOC::eMODE_BALANCE;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_NOCHANGE:
    default:
        hwmode = mCurrentMode;
        break;
    }

    switch(freq)
    {
    case ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE:
        hwfreq = PARAM_LF_FREQ_512HZ_SONDE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE:
        hwfreq = PARAM_LF_FREQ_640HZ_SONDE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE:
        hwfreq = PARAM_LF_FREQ_50HZ_PASSIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE:
        hwfreq = PARAM_LF_FREQ_60HZ_PASSIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE:
        hwfreq = PARAM_LF_FREQ_32_5KHZ_ACTIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE:
        hwfreq = PARAM_LF_FREQ_32_5KHZ_PASSIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_NOCHANGE:
    default:
        hwfreq = mFrequency;
        break;
    }

    if(mode == ROLOC_DBUS_API::eROLOC_MODE_OFF)
    {
        stop();
    }
    else
    {
        // TODO remove
        qCritical() << "request for mode: "
                       << getString(static_cast<ROLOC_DBUS_API::eROLOC_MODE>(mode))
                       << QString("--hwmode-->").arg(QString::number(hwmode, 16))
                       << "@ freq "
                       << getString(static_cast<ROLOC_DBUS_API::eROLOC_FREQUENCY>(freq))
                       << QString("--hwfreq--> 0x%1").arg(QString::number(hwfreq, 16));

        rolocSetParameters(hwmode, hwfreq);
    }
    mDbusHandler.sendParameters(getModeDBUS(), getFrequencyDBUS());
}

/**
 * @brief ROLOCcontroller::getFrequencyDBUS -- convert internal
 *        frequency to DBUS enum
 * @return ROLOC_DBUS_API::eROLOC_FREQUENCY
 */
ROLOC_DBUS_API::eROLOC_FREQUENCY ROLOCcontroller::getFrequencyDBUS()
{
    ROLOC_DBUS_API::eROLOC_FREQUENCY freq;

    // convert hardware frequency values to dbus enum
    switch(mFrequency)
    {
    case PARAM_LF_FREQ_512HZ_SONDE:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE;
        break;
    case PARAM_LF_FREQ_640HZ_SONDE:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE;
        break;
    case PARAM_LF_FREQ_50HZ_PASSIVE:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE;
        break;
    case PARAM_LF_FREQ_60HZ_PASSIVE:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE;
        break;
    case PARAM_LF_FREQ_32_5KHZ_ACTIVE:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE;
        break;
    case PARAM_LF_FREQ_32_5KHZ_PASSIVE:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE;
        break;
    default:
        freq = ROLOC_DBUS_API::eROLOC_FREQ_NOCHANGE;
        break;
    }
    return(freq);
}

/**
 * @brief ROLOCcontroller::getModeDBUS -- convert internal
 *        mode bools to DBUS enum
 * @return ROLOC_DBUS_API::eROLOC_MODE
 */
ROLOC_DBUS_API::eROLOC_MODE ROLOCcontroller::getModeDBUS()
{
    ROLOC_DBUS_API::eROLOC_MODE mode;

    if(mEnabled)
    {
        // convert hardware frequency values to dbus enum
        switch(mCurrentMode)
        {
        case ROLOC::eMODE_GET_SIGNAL_STRENGTH:
            mode = ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH;
            break;
        case ROLOC::eMODE_GET_DEPTH_MEASUREMENT:
            mode = ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT;
            break;
        case ROLOC::eMODE_CALIBRATION:
            mode = ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION;
            break;
        case ROLOC::eMODE_CALIBRATION_TEST:
            mode = ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST;
            break;
        case ROLOC::eMODE_BALANCE:
            mode = ROLOC_DBUS_API::eROLOC_MODE_BALANCE;
            break;
        default:
            mode = ROLOC_DBUS_API::eROLOC_MODE_NOCHANGE;
            break;
        }
    }
    else
    {
        mode = ROLOC_DBUS_API::eROLOC_MODE_OFF;
    }
    return(mode);
}
