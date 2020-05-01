#ifndef ROLOCCONTROLLER_HPP
#define ROLOCCONTROLLER_HPP

#include <QThread>
#include "i2c.hpp"

class ROLOCcontroller
        : public QObject
{
    Q_OBJECT
public:
    ROLOCcontroller();
    virtual ~ROLOCcontroller();

    void init(int argc, char *argv[]);
#if 0
    // two reasons i removed this:
    // 1) you can achieve the same result with a timer
    // 2) the way you were implementing a qthread is frowned upon by the qt framework.
    //    the correct way to do that is to create your class and call moveToThread
    //
    //    eg:
    //    ROLOCcontroller *r = new ROLOCcontroller();
    //    QThread *t = new QThread();
    //    r->moveToThread(t);
    //    QObject::connect(t, SIGNAL(started()), r, SLOT(startROLOCMonitor() ));
    //    QObject::connect(t, SIGNAL(finished()), r, SLOT(deleteLater() ));
    //
    void run();
#endif

    void start();
    void stop();

signals:
    // this is a place for outbound actions to parent classes
    // i.e. ROLOCcontroller *r = new ROLOCcontroller()
    //      QObject::connect(r, SIGNAL( updateSSR() ), dbus, SLOT( updateSSR() ) );
    void updateStatus(int status);
    void updateSSR(int ssr);
    void updateDepth(double depth);
    void updateVolume(int lvl);

public slots:
    // this is a good way to separate slots that accessible from parent classes
    // i.e. ROLOCcontroller *r = new ROLOCcontroller()
    //      QObject::connect(dbus, SIGNAL( getSSR() ), r, SLOT( getSSR() ) );
    void getSignalStrength();
    void getDepthMeasurement();
    void getStatus();
    void setVolume();

private slots:
    // this is a good way to separate 'self' slots like timer callbacks

private:

    void getSignalStrengthSignalHandler();
    void getDepthMeasurementSignalHandler();
    void getStatusSignalHandler();
    void setVolumeSignalHandler();

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
