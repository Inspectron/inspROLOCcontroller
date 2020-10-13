#ifndef ROLOCCONTROLLER_HPP
#define ROLOCCONTROLLER_HPP

#include <QThread>
#include <QTimer>
#include "i2c.hpp"
#include "inspRolocControllerDbus.hpp"
#include "rolocarrows.hpp"

namespace ROLOC
{
    // modes of operation
    enum eLINEFINDER_MODE
    {
        eMODE_GET_SIGNAL_STRENGTH        = (0x0000),
        eMODE_GET_DEPTH_MEASUREMENT      = (0x0002),
        eMODE_CALIBRATION                = (0x0001 | eMODE_GET_DEPTH_MEASUREMENT),
        eMODE_CALIBRATION_TEST           = (0x0080 | eMODE_GET_DEPTH_MEASUREMENT),
        eMODE_BALANCE                    = (0x0081 | eMODE_GET_DEPTH_MEASUREMENT),
    };

    // volume levels
    enum eLINEFINDER_VOLUME
    {
        eVOLUME_OFF                     =   0x00,
        eVOLUME_MED                     =   0x01,
        eVOLUME_HIGH                    =   0x02,
    };

    // commands
    enum eLINEFINDER_CMD
    {
        eCMD_GET_ID                         =   0xDC,
        eCMD_VOLUME                         =   0x07,
        eCMD_INFO                           =   0xFA,
    };



}

class ROLOCcontroller
: public QObject
{
    Q_OBJECT
public:
    ROLOCcontroller();
    virtual ~ROLOCcontroller();

    void init();
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
    void rolocSetParameters(quint16 mode, int frequency);
    qint16 rolocGetData();
    ROLOC_DBUS_API::eROLOC_FREQUENCY getFrequencyDBUS();
    ROLOC_DBUS_API::eROLOC_MODE getModeDBUS();
    QString getString(ROLOC_DBUS_API::eROLOC_MODE mode);
    QString getString(ROLOC_DBUS_API::eROLOC_FREQUENCY freq);

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

    bool mbModeChangeComplete;
    quint8 mNumSamples;
    QList<quint8> mDepthAccumulator;

    ROLOCArrows &mRolocArrows;
    QTimer *mpRolocDataPollingTimer;
};

/**
 * @brief ROLOCcontroller::getString - convert dbus roloc mode to a string
 */
inline QString ROLOCcontroller::getString(ROLOC_DBUS_API::eROLOC_MODE mode)
{
    return (mode == ROLOC_DBUS_API::eROLOC_MODE_NOCHANGE              ? "mode no change"    :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_OFF                   ? "mode off"          :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH   ? "mode sig strength" :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT ? "mode depth meas"   :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION           ? "mode cal"          :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST      ? "mode cal test"     :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_BALANCE               ? "mode balance"      :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_MAX                   ? "mode max"          : "???" ))))))));
}

/**
 * @brief ROLOCcontroller::getString - convert dbus roloc freq to a string
 */
inline QString ROLOCcontroller::getString(ROLOC_DBUS_API::eROLOC_FREQUENCY freq)
{
    return (freq == ROLOC_DBUS_API::eROLOC_FREQ_NOCHANGE        ? "no change"       :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE     ? "512hz sonde"     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE     ? "640hz sonde"     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE    ? "50hz passive"    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE    ? "60hz passive"    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE  ? "32.5khz active"  :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE ? "32.5khz passive" :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_MAX             ? "freq max"        : "???" ))))))));
}

#endif // RTSPSERVER_HPP
