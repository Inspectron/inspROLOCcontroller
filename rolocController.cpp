#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"

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
    const int    LINEFINDER_CAL_FAILURE_VALUE			    = 253;

    const int    N_MULTI_LF_DEPTH_SAMPLE	                = 5;
    const int    N_MULTI_LF_DEPTH_DELTA 	                = 6;
    const int    N_MULTI_LF_DEPTH_OK 	                    = 2;

    const int    LINEFINDER_NO_DATA                         = 0xFFFF;

    const int    PARAM_LF_FREQ_512HZ_SONDE                  = (0 << 11);
    const int    PARAM_LF_FREQ_640HZ_SONDE                  = (1 << 11);
    const int    PARAM_LF_FREQ_50HZ_PASSIVE                 = (2 << 11);
    const int    PARAM_LF_FREQ_60HZ_PASSIVE                 = (3 << 11);
    const int    PARAM_LF_FREQ_32_5KHZ_ACTIVE               = (8 << 11);
    const int    PARAM_LF_FREQ_32_5KHZ_PASSIVE              = (9 << 11);

}

/**
 * @brief ROLOCcontroller::ROLOCcontroller - ctor. construct any objects
 */
ROLOCcontroller::ROLOCcontroller()
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


    rolocDataPollingTimer = new QTimer();
    connect(rolocDataPollingTimer, SIGNAL(timeout()), this, SLOT(pollROLOC()));
    rolocDataPollingTimer->setInterval(250);

    rolocHardwarePresent(); qDebug() << "Hardware Present: " << mHardwarePresent;
    rolocSetVolume(mCurrVolume);
    rolocSetParameters(LINEFINDER_MODE_GET_SIGNAL_STRENGTH, mFrequency);

    rolocDataPollingTimer->start();
}

void ROLOCcontroller::start()
{
    // TODO start the timer
}

void ROLOCcontroller::stop()
{
    // TODO stop the timer
}

void ROLOCcontroller::pollROLOC()
{
    if(m_bModeChangeComplete /*&& m_nSamples < N_MULTI_LF_DEPTH_DELTA*/)
    {
        qint16 rolocData = rolocGetData();
        qDebug() << "ROLOC DATA: " << rolocData;

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

            }
        }

        //getDepthMeasurementSignalHandler();
        //m_bModeChangeComplete = false;
    }
}

void ROLOCcontroller::modeChangeComplete()
{
    m_bModeChangeComplete = true;
    qDebug() << "3.5 second timer expired.  Mode Change Complete";
}

/**
 * @brief ROLOCcontroller::getSignalStrength - slot callback to get the signal strength
 */
void ROLOCcontroller::getSignalStrength()
{
    // TODO implement
}

/**
 * @brief ROLOCcontroller::getDepthMeasurement - slot callback to get the depth measurement
 */
void ROLOCcontroller::getDepthMeasurement()
{
    // TODO implement
}

/**
 * @brief ROLOCcontroller::getStatus - slot callback to get the status
 */
void ROLOCcontroller::getStatus()
{
    // TODO implement
}

/**
 * @brief ROLOCcontroller::setVolume - slot callback to set the volume
 */
void ROLOCcontroller::setVolume()
{
    // TODO implement
}

/**
 * @brief ROLOCcontroller::rolocHardwarePresent
 */
void ROLOCcontroller::rolocHardwarePresent()
{
    qint16 data = m_i2cBus.i2c_readWord(mI2cAddr, LINEFINDER_GET_ID);

    if(data != 0x0102)
    {
        qWarning() << "Could not read ID from ROLOC Hardware";
        mHardwarePresent = false;
    }
    else
    {
        mHardwarePresent = true;
    }

    //emit rolocPresent(mHardwarePresent);
}

/**
 * @brief ROLOCcontroller::rolocGetData
 * @return
 */
qint16 ROLOCcontroller::rolocGetData()
{
    qint16 data = m_i2cBus.i2c_readWord(mI2cAddr, LINEFINDER_INFO);
    qDebug() << "raw data: " << hex << data;

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

    if(mode != mCurrentMode)
    {
        mCurrentMode = mode;
        m_bModeChangeComplete = false;
        QTimer::singleShot(3000, this, SLOT(modeChangeComplete()));
    }
}

/**
 * @brief ROLOCcontroller::rolocSetVolume
 * @param data
 */
void ROLOCcontroller::rolocSetVolume(int16_t data)
{
    /*qint8 status =*/  m_i2cBus.i2c_writeWord(mI2cAddr, LINEFINDER_VOLUME, data);
    //todo:: consider error case for I2C
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


/**
 * @brief ROLOCcontroller::getSignalStrengthSignalHandler
 */
void ROLOCcontroller::getSignalStrengthSignalHandler()
{
    mCurrentMode = LINEFINDER_MODE_GET_SIGNAL_STRENGTH;
    quint8 signalStrength = rolocGetData();
    //emit rolocSignalStrength(signalStrength);
}

/**
 * @brief ROLOCcontroller::getDepthMeasurementSignalHandler
 */
void ROLOCcontroller::getDepthMeasurementSignalHandler()
{

}

/**
 * @brief ROLOCcontroller::getStatusSignalHandler
 */
void ROLOCcontroller::getStatusSignalHandler()
{
    //emit rolocStatus();
}

/**
 * @brief ROLOCcontroller::setVolumeSignalHandler
 */
void ROLOCcontroller::setVolumeSignalHandler()
{
    //emit rolocVolume();
}
