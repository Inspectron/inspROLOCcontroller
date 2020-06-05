#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"

namespace {

    const quint8  I2C_BUS                                   =      1;
    const quint8  LINEFINDER_I2C_HW_BASE_ADDRESS            =   0xFA;

    const quint8  LINEFINDER_GET_ID                         =   0xDC;
    const quint8  LINEFINDER_VOLUME                         =   0x07;
    const quint8  LINEFINDER_INFO                           =   0xFA;

    const quint8  LINEFINDER_VOLUME_OFF                     =   0x00;
    const quint8  LINEFINDER_VOLUME_MED                     =   0x01;
    const quint8  LINEFINDER_VOLUME_HIGH                    =   0x02;

    const int    LINEFINDER_MODE_GET_SIGNAL_STRENGTH        = (0x0000);
    const int    LINEFINDER_MODE_GET_DEPTH_MEASUREMENT      = (0x0002);
    const int    LINEFINDER_MODE_CALIBRATION                = (0x0001 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT);
    const int    LINEFINDER_MODE_CALIBRATION_TEST           = (0x0080 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT);
    const int    LINEFINDER_MODE_BALANCE                    = (0x0081 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT);

    const int    LINEFINDER_LEFT_ARROW_BIT                  = 3;
    const int    LINEFINDER_RIGHT_ARROW_BIT                 = 2;
    const int    LINEFINDER_CENTER_ARROW_BIT                = 1;

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

}

/**
 * @brief ROLOCcontroller::ROLOCcontroller - ctor. construct any objects
 */
ROLOCcontroller::ROLOCcontroller()
: mDbusHandler(* new InspROLOCControllerDbus())
, mpRolocDataPollingTimer(NULL)
{
}

/**
 * @brief ROLOCcontroller::~ROLOCcontroller - dtor. destroy any created objects
 */
ROLOCcontroller::~ROLOCcontroller()
{

}

/**
 * @brief ROLOCcontroller::init - init the class
 * @param argc
 * @param argv
 */
void ROLOCcontroller::init(int argc, char *argv[]) // TODO are these being used ?
{
    m_i2cBus.i2c_setDevice(I2C_BUS);
    mI2cAddr = (LINEFINDER_I2C_HW_BASE_ADDRESS >> 1);

    mCurrentMode = LINEFINDER_MODE_GET_SIGNAL_STRENGTH;  //LINEFINDER_MODE_GET_DEPTH_MEASUREMENT
    mFrequency   = PARAM_LF_FREQ_512HZ_SONDE; //todo:: over-ride with setting in UI if UI setting is "sticky"
    mCurrVolume  = LINEFINDER_VOLUME_OFF;
    m_bModeChangeComplete = true;
    mHardwarePresent = false;
    mEnabled = false;

    mDbusHandler.init();

    mpRolocDataPollingTimer = new QTimer();
    connect(mpRolocDataPollingTimer, SIGNAL(timeout()), this, SLOT(pollROLOC()));
    mpRolocDataPollingTimer->setInterval(TIMER_DATA_POLLING_PERIOD);

    rolocHardwarePresent(); qDebug() << "Hardware Present: " << mHardwarePresent;
    rolocSetVolume(mCurrVolume);
    rolocSetParameters(LINEFINDER_MODE_GET_SIGNAL_STRENGTH, mFrequency);

    // connect sigs/slots
    //dbus signals -> controller
    QObject::connect(&mDbusHandler,   SIGNAL(getDataReportHandler()),         this,   SLOT(getDataReportHandler()) );
    QObject::connect(&mDbusHandler,   SIGNAL(setVolumeHandler(int)),          this,   SLOT(setVolumeHandler(int)) );
    QObject::connect(&mDbusHandler,   SIGNAL(setParametersHandler(int, int)), this,   SLOT(setParametersHandler(int, int)) );

    stop();    // don't start until active mode set
}

void ROLOCcontroller::start()
{
    QTimer::singleShot(3000, this, SLOT(modeChangeComplete()));
    mpRolocDataPollingTimer->start();
    m_bModeChangeComplete = false;
    mEnabled = true;
}

void ROLOCcontroller::stop()
{
    m_bModeChangeComplete = false;
    mEnabled = false;
}

void ROLOCcontroller::pollROLOC()
{
    bool hw = mHardwarePresent;

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
    if(mHardwarePresent && m_bModeChangeComplete && mEnabled /*&& m_nSamples < N_MULTI_LF_DEPTH_DELTA*/)
    {
        qint16 rolocData = rolocGetData();
        //qDebug() << "ROLOC DATA: " << rolocData;

        if(mCurrentMode == LINEFINDER_MODE_GET_SIGNAL_STRENGTH)
        {
            mDepthAccumulator.clear();
        }
        else
        {
            mDepthAccumulator.append(rolocData);
            m_nSamples++;

            if(m_nSamples > N_MULTI_LF_DEPTH_SAMPLE)
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

void ROLOCcontroller::modeChangeComplete()
{
    m_bModeChangeComplete = true;
    qDebug() << "3.5 second timer expired.  Mode Change Complete";
}

/**
 * @brief ROLOCcontroller::rolocHardwarePresent
 */
void ROLOCcontroller::rolocHardwarePresent()
{
    qint16 data;

    data = m_i2cBus.i2c_readWord(mI2cAddr, LINEFINDER_GET_ID);

    if(data != 0x0102)
    {
        qWarning() << "Could not read ID from ROLOC Hardware";
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
    data = m_i2cBus.i2c_readWord(mI2cAddr, LINEFINDER_INFO);
    //qDebug() << "raw data: " << hex << data;

    if(data >= 0)
    {
        quint8 statusByte = ((data & 0xFF00) >> 8);

        quint8 depthORCalTest = (statusByte >> LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED) & 1; //  (statusByte & (LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED));
        quint8 calORBalance   = (statusByte >> LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED) & 1;    //  (statusByte & (LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED   ));
        quint8 ping           = (statusByte >> LINEFINDER_PING_DATA_RETURNED) & 1;              //  (statusByte & (LINEFINDER_PING_DATA_RETURNED             ));
        quint8 specialDataReceived = (depthORCalTest || calORBalance || ping);

#if 0
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
            data = data & 0x00FF;
            if(data > 240) data = 240;
            if(data <  10) data = 0;

            mLeftArrow   = (statusByte >> LINEFINDER_LEFT_ARROW_BIT)   & 1;
            mCenterArrow = (statusByte >> LINEFINDER_CENTER_ARROW_BIT) & 1;
            mRightArrow  = (statusByte >> LINEFINDER_RIGHT_ARROW_BIT)  & 1;
            mNoArrow     = !mLeftArrow && !mCenterArrow && !mRightArrow;

            // oddly enough the device will provide multiple arrow indicators
            // therefore we filter the results such that if the data is zero
            // then indicate a center arrow, else priority to right arrow if left
            // and right are true.  Old code for displaying the arrow was:
            // e.g. screen = (mCenterArrow || mNoArrow || data == 0) ? dislay center arrow : ((mRightArrow) ? display rigth : display left
#if 0
            qDebug() << "statusByte  : " << hex << statusByte;
            qDebug() << "LeftArrow   : " << hex << mLeftArrow;
            qDebug() << "RightArrow  : " << hex << mRightArrow;
            qDebug() << "CenterArrow : " << hex << mCenterArrow;
            qDebug() << "NoArrow     : " << hex << mNoArrow;
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
void ROLOCcontroller::rolocSetParameters(quint16 mode, quint8 frequency)
{
    int16_t data = 0x0400;

    data |= (mode << 8);  // todo:: double check that the shift operation provides the correct results!
    data |= frequency;

    /*qint8 i2cStatus =*/ m_i2cBus.i2c_writeWord(mI2cAddr, LINEFINDER_INFO, data);
    // todo:: consider error case for I2C

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
    if(data < LINEFINDER_VOLUME_OFF)
    {
        data = LINEFINDER_VOLUME_OFF;
    }
    else if(data > LINEFINDER_VOLUME_HIGH)
    {
        // TODO -- check how to handle out of range value
        data = LINEFINDER_VOLUME_OFF;
    }

    /*qint8 status =*/  m_i2cBus.i2c_writeWord(mI2cAddr, LINEFINDER_VOLUME, data);
    //todo:: consider error case for I2C

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
    qDebug("Report: mode=%d freq=%d signal=%d depth=%0.2f arrow=%d hw=%d",
           getModeDBUS(), getFrequencyDBUS(),
            mROLOCsignalStrenth, mROLOCdepthMeasurement, getArrowDBUS(), mHardwarePresent);
            mDbusHandler.sendDataReport(
            getModeDBUS(),
            getFrequencyDBUS(),
            mROLOCsignalStrenth,
            mROLOCdepthMeasurement,
            getArrowDBUS(),
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
        hwvol = LINEFINDER_VOLUME_OFF;
        break;
    case ROLOC_DBUS_API::eROLOC_VOLUME_MED:
        hwvol = LINEFINDER_VOLUME_MED;
        break;
    case ROLOC_DBUS_API::eROLOC_VOLUME_HIGH:
        hwvol = LINEFINDER_VOLUME_HIGH;
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

    switch(mode)
    {
    case ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH:
        hwmode = LINEFINDER_MODE_GET_SIGNAL_STRENGTH;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT:
        hwmode = LINEFINDER_MODE_GET_DEPTH_MEASUREMENT;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION:
        hwmode = LINEFINDER_MODE_CALIBRATION;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST:
        hwmode = LINEFINDER_MODE_CALIBRATION_TEST;
        break;
    case ROLOC_DBUS_API::eROLOC_MODE_BALANCE:
        hwmode = LINEFINDER_MODE_BALANCE;
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
        case LINEFINDER_MODE_GET_SIGNAL_STRENGTH:
            mode = ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH;
            break;
        case LINEFINDER_MODE_GET_DEPTH_MEASUREMENT:
            mode = ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT;
            break;
        case LINEFINDER_MODE_CALIBRATION:
            mode = ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION;
            break;
        case LINEFINDER_MODE_CALIBRATION_TEST:
            mode = ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST;
            break;
        case LINEFINDER_MODE_BALANCE:
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

/**
 * @brief ROLOCcontroller::getArrowDBUS -- convert internal
 *        arrow bools to DBUS enum
 * @return ROLOC_DBUS_API::eROLOC_ARROW
 */
ROLOC_DBUS_API::eROLOC_ARROW ROLOCcontroller::getArrowDBUS()
{
    ROLOC_DBUS_API::eROLOC_ARROW arrow;

    // generate enumerated value from arrow bools
    if(mCenterArrow || mNoArrow)
    {
        arrow = ROLOC_DBUS_API::eCENTER_ARROW;
    }
    else if(mRightArrow)
    {
        arrow = ROLOC_DBUS_API::eRIGHT_ARROW;
    }
    else
    {
        arrow = ROLOC_DBUS_API::eLEFT_ARROW;
    }
    return(arrow);
}
