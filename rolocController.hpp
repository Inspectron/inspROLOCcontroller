#ifndef ROLOCCONTROLLER_HPP
#define ROLOCCONTROLLER_HPP

#include <QThread>
#include "i2c.hpp"

class ROLOCcontroller : public QThread
{
    Q_OBJECT
public:
    ROLOCcontroller();
    virtual ~ROLOCcontroller(){}

    void init(int argc, char *argv[]);
    void run();

signals:
    void getSignalStrength();
    void getDepthMeasurement();
    void getStatus();
    void setVolume();

private:

    static ROLOCcontroller* theROLOCcontroller;

    static void getSignalStrengthSignalHandler();
    static void getDepthMeasurementSignalHandler();
    static void getStatusSignalHandler();
    static void setVolumeSignalHandler();

    double getMean(QList<quint8> values);
    double getVariance(QList<quint8> values);
    void rolocHardwarePresent();
    void rolocSetVolume(quint16 data);
    void rolocSetParameters();
    qint16 rolocGetData();

    i2c m_i2cBus;
    int mI2cAddr;

    quint8 mHardwarePresent;
    quint16 m_mode;
    quint16 mROLOCsignalStrenth;
    quint8 mROLOCdepthMeasurement;
    quint8 mCurrVolume;
    quint16 mFrequency;

};

#endif // RTSPSERVER_HPP
