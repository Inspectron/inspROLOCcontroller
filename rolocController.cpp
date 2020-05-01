#include <QDebug>
#include <QtMath>
#include "rolocController.hpp"

namespace {

    const int    LINEFINDER_GET_ID                          =   0xDC;
    const int    LINEFINDER_VOLUME                          =   0x07;
    const int    LINEFINDER_INFO                            =   0xFA;

    const int    LINEFINDER_MODE_GET_SIGNAL_STRENGTH        = (0x0000);
    const int    LINEFINDER_MODE_GET_DEPTH_MEASUREMENT      = (0x0002);
    const int    LINEFINDER_MODE_CALIBRATION                = (0x0001 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT);
    const int    LINEFINDER_MODE_CALIBRATION_TEST           = (0x0080 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT);
    const int    LINEFINDER_MODE_BALANCE                    = (0x0081 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT);

    const int    LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED = 0x0010;
    const int    LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    = 0x0020;
    const int    LINEFINDER_PING_DATA_RETURNED              = 0x0040;
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
    m_i2cBus.i2c_setDevice(1);  // todo:: look up the correct bus
    mI2cAddr = (0xFA >> 1);

    mHardwarePresent = false; // TODO you should convert this to a bool instead of quint8
    m_mode = LINEFINDER_MODE_GET_SIGNAL_STRENGTH;
    mFrequency = PARAM_LF_FREQ_512HZ_SONDE; //todo:: over-ride with setting in UI if UI setting is "sticky"
}

void ROLOCcontroller::start()
{
    // TODO start the timer
}

void ROLOCcontroller::stop()
{
    // TODO stop the timer
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

// you dont need a run function if you have a timer. notice i removed the qtimer in the hpp
#if 0
void ROLOCcontroller::run()
{
    theROLOCcontroller->rolocHardwarePresent();
}
#endif

/**
 * @brief ROLOCcontroller::rolocHardwarePresent
 */
void ROLOCcontroller::rolocHardwarePresent()
{
//    quint8      data[2];
//    quint8 bytesRead = m_i2cBus.i2c_read_continuous(mI2cAddr, LINEFINDER_GET_ID, data, 2);

//    if(bytesRead != 2 || data != 0x0102)
//    {
//        qWarning() << "Could not read ID from ROLOC Hardware";
//        bPresent = false;
//    }

    qint16 data = m_i2cBus.i2c_readWord(mI2cAddr, LINEFINDER_GET_ID);

    if(data != 0x0102)
    {
        qWarning() << "Could not read ID from ROLOC Hardware";
        mHardwarePresent = false;
        //emit rolocPresent(false);
    }
    else
    {
        mHardwarePresent = true;
        //emit rolocPresent(true);
    }


    //emit rolocPresent(true);
}

/**
 * @brief ROLOCcontroller::rolocGetData
 * @return
 */
qint16 ROLOCcontroller::rolocGetData()
{
    //quint8 bytesRead = m_i2cBus.i2c_read_continuous(mI2cAddr, LINEFINDER_INFO, data, 2);
    qint16 data = m_i2cBus.i2c_readWord(mI2cAddr, LINEFINDER_INFO);

    //if(bytesRead == 2)
    if(data >= 0)
    {
        quint8 depthORCalTest = (data & (LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED << 8));
        quint8 calORBalance   = (data & (LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    << 8));
        quint8 ping           = (data & (LINEFINDER_PING_DATA_RETURNED              << 8));
        //quint8 specialDataReceived = (depthORCalTest || calORBalance || ping);

        if(depthORCalTest || calORBalance)
        {
            data = data & 0x00FF;
        }
    }
    else
    {
        qWarning() << "Could not read ROLOC I2C Status";
        //nTries++;
    }

    return data;
}

/**
 * @brief ROLOCcontroller::rolocSetParameters
 */
void ROLOCcontroller::rolocSetParameters()
{
    quint16 data1 = 0x0400;

    rolocSetVolume(mCurrVolume);

    data1 |= (m_mode << 8);  // todo:: double check that the shift operation provides the correct results!
    data1 |= mFrequency;

    qint8 i2cStatus = m_i2cBus.i2c_writeWord(mI2cAddr, LINEFINDER_INFO, data1);
    // todo:: consider error case for I2C
}

/**
 * @brief ROLOCcontroller::rolocSetVolume
 * @param data
 */
void ROLOCcontroller::rolocSetVolume(quint16 data)
{
    qint8 status =  m_i2cBus.i2c_writeWord(mI2cAddr, LINEFINDER_VOLUME, data);
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
    int temp = 0;

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
    m_mode = LINEFINDER_MODE_GET_SIGNAL_STRENGTH;
    quint8 signalStrength = rolocGetData();
    //emit rolocSignalStrength(signalStrength);
}

/**
 * @brief ROLOCcontroller::getDepthMeasurementSignalHandler
 */
void ROLOCcontroller::getDepthMeasurementSignalHandler()
{
    QList<quint8> depthAccumulator;
    QList<quint8> acceptedDepthReadings;
    float scaleOfElimination = (float)N_MULTI_LF_DEPTH_DELTA;

    m_mode = LINEFINDER_MODE_GET_DEPTH_MEASUREMENT;

    // collect a minimum of five (N_MULTI_LF_DEPTH_SAMPLE) samples, average and report depth measurement
    for (quint8 nSamples = 0; nSamples < N_MULTI_LF_DEPTH_SAMPLE; nSamples++)
    {
        rolocSetParameters();
        depthAccumulator.append(rolocGetData());
    }

    double mean = getMean(depthAccumulator);
    double stdDev = qSqrt(getVariance(depthAccumulator));

    for(quint8 i = 0; i < depthAccumulator.count(); i++)
    {
        quint8 isLessThanLowerBound = depthAccumulator.at(i) < mean - stdDev * scaleOfElimination;
        quint8 isGreaterThanUpperBound = depthAccumulator.at(i) > mean + stdDev * scaleOfElimination;
        quint8 isOutOfBounds = isLessThanLowerBound || isGreaterThanUpperBound;

        if(!isOutOfBounds)
        {
            acceptedDepthReadings.append(depthAccumulator.at(i));
        }
    }

    double finalReading = getMean(acceptedDepthReadings);
    //emit rolocDepthMeasurement(finalReading);
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
