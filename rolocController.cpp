#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"

#define DBG_BLOCK 0

namespace {

    const quint8  I2C_BUS                                   =      1;
    const quint8  LINEFINDER_I2C_HW_BASE_ADDRESS            =  (0xFA >> 1);
    const qint16  LINEFINDER_I2C_ID                         =  0x0102;

    // TODO none of these dec-> hex values are correct
    const int    LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED = 4;  // 0x0010;
    const int    LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    = 5;  // 0x0020;
    const int    LINEFINDER_PING_DATA_RETURNED              = 6;  // 0x0040;
    const int    LINEFINDER_CAL_FAILURE_VALUE               = 253;

    const int    N_MULTI_LF_DEPTH_SAMPLE                    = 5;
    const int    N_MULTI_LF_DEPTH_DELTA                     = 6;
    const int    N_MULTI_LF_DEPTH_OK                        = 2;

    const int    LINEFINDER_NO_DATA                         = 0xFFFF;

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
, mFrequency(ROLOC::eFREQ_512HZ_SONDE)
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
    mI2cAddr = (LINEFINDER_I2C_HW_BASE_ADDRESS);

    // init dbus
    mDbusHandler.init();

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

#if 0
    // TODO can this be removed ?
    stop();    // don't start until active mode set
#endif
}

/**
 * @brief ROLOCcontroller::initROLOC - init the roloc upon plugging in
 */
void ROLOCcontroller::initROLOC()
{
    qDebug() << "ROLOC plugged in. set volume and freq to defaults";
    rolocSetVolume(ROLOC::eVOLUME_OFF);
    rolocSetParameters(ROLOC::eMODE_GET_SIGNAL_STRENGTH, ROLOC::eFREQ_512HZ_SONDE);
}

/**
 * @brief ROLOCcontroller::start - Start a mode change ? and just delay everything for a bit ?
 */
void ROLOCcontroller::startModeChange()
{       
    mbModeChangeComplete = false;
    mEnabled             = true;

    QTimer::singleShot(TIMER_3SECONDS, [&]()
    {
        mbModeChangeComplete = true;
        qDebug() << "3.5 second timer expired.  Mode Change Complete";
    });
}

// TODO what does this mean? two bools are being used for one purpose. it can be a state or bool mbIsBusy
void ROLOCcontroller::stop()
{
    mbModeChangeComplete = false;
    mEnabled             = false;
}

/**
 * @brief ROLOCcontroller::pollROLOC - function to poll the roloc
 */
void ROLOCcontroller::pollROLOC()
{
    // update hardware existence
    bool present = rolocHardwarePresent();

    // roloc was turned on/plugged in
    if (!mHardwarePresent && present)
    {
        initROLOC();
    }

    mHardwarePresent = present;


    // TODO figure this out
#if 0
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
#endif

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

// TODO cleaner way to implement mode change
void ROLOCcontroller::modeChangeComplete()
{

}


/**
 * @brief ROLOCcontroller::rolocHardwarePresent - check if the roloc hardware is attached
 */
bool ROLOCcontroller::rolocHardwarePresent()
{
    bool present = true;
    qint16 data;
    // TODO fix this. the function should return a bool

    data = m_i2cBus.i2c_readWord(mI2cAddr, ROLOC::eCMD_GET_ID);

    if(data != LINEFINDER_I2C_ID)
    {
        qWarning() << "Could not read ID from ROLOC Hardware. data = " << data;
        present = false;
    }

    // udpate dbus
    mDbusHandler.sendPresent(present);

    // return the value
    return present;
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
void ROLOCcontroller::rolocSetParameters(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ frequency)
{
    int16_t data = 0x0400;  // TODO : WHAT IS THIS MAGIC NUMBER ?

    data |= (mode << 8);  // todo:: double check that the shift operation provides the correct results!
    data |= frequency;    // TODO THIS WILL ACTUALY CLIP THE FREQ VALUE since its 16 bit.

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
#if DBG_BLOCK
    qDebug("Report: mode=%d freq=%d signal=%d depth=%0.2f arrow=%d hw=%d",
            mCurrentMode, mFrequency,
            mROLOCsignalStrenth, mROLOCdepthMeasurement, getArrowDBUS(), mHardwarePresent);
#endif
            mDbusHandler.sendDataReport(
            mCurrentMode,
            mFrequency,
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
#if 0
    qint8 hwmode;
    qint16 hwfreq;

    // TODO: converting enums back and forth is sloppy, the conversion can be done in dbus or not at all
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
        hwfreq = ROLOC::eFREQ_512HZ_SONDE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE:
        hwfreq = ROLOC::eFREQ_640HZ_SONDE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE:
        hwfreq = ROLOC::eFREQ_50HZ_PASSIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE:
        hwfreq = ROLOC::eFREQ_60HZ_PASSIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE:
        hwfreq = ROLOC::eFREQ_32_5KHZ_ACTIVE;
        break;
    case ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE:
        hwfreq = ROLOC::eFREQ_32_5KHZ_PASSIVE;
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
    }
#else

    rolocSetParameters(mode, freq);
#endif
}
