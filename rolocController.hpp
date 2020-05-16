#ifndef ROLOCCONTROLLER_HPP
#define ROLOCCONTROLLER_HPP

#include <QThread>
#include <QTimer>
#include "i2c.hpp"
#include "inspRolocControllerDbus.hpp"

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
    void sendDataReport();

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
    void getDataReportHandler();
    void setVolumeHandler(int lvl);
    void setParametersHandler(int mode, int freq);

private slots:
    // this is a good way to separate 'self' slots like timer callbacks
    void pollROLOC();
    void modeChangeComplete();

private:

    void getSignalStrengthSignalHandler();
    void getDepthMeasurementSignalHandler();

    double getMean(QList<quint8> values);
    double getVariance(QList<quint8> values);
    void rolocHardwarePresent();
    void rolocSetVolume(int16_t data);
    void rolocSetParameters(quint16 mode, quint8 frequency);
    qint16 rolocGetData();
    ROLOC_DBUS_API::eROLOC_FREQUENCY getFrequencyDBUS();
	ROLOC_DBUS_API::eROLOC_MODE getModeDBUS();
	ROLOC_DBUS_API::eROLOC_ARROW getArrowDBUS();

    i2c m_i2cBus;
    quint8 mI2cAddr;
    InspROLOCControllerDbus &mDbusHandler;

	bool mEnabled;
    bool mHardwarePresent;
    quint8 mCurrentMode;
    quint16 mROLOCsignalStrenth;
    double mROLOCdepthMeasurement;
    quint8 mCurrVolume;
    quint16 mFrequency;

    bool m_bModeChangeComplete;
    quint8 m_nSamples;
    QList<quint8> mDepthAccumulator;

    bool mLeftArrow;
    bool mRightArrow;
    bool mCenterArrow;
    bool mNoArrow;

    QTimer *mpRolocDataPollingTimer;
};

#endif // RTSPSERVER_HPP
