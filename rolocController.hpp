#ifndef ROLOCCONTROLLER_HPP
#define ROLOCCONTROLLER_HPP

#include <QThread>
#include <QTimer>
#include "i2c.hpp"
#include "inspRolocControllerDbus.hpp"
#include "rolocarrows.hpp"
#include "roloctypes.hpp"

class ROLOCcontroller
: public QObject
{
    Q_OBJECT
public:
    ROLOCcontroller();
    virtual ~ROLOCcontroller();

    void init();
    void startModeChange();
    void stop();
    void sendDataReport();

signals:
    void updateStatus(int status);
    void updateSSR(int ssr);
    void updateDepth(double depth);
    void updateVolume(int lvl);

public slots:
    void getDataReportHandler();
    void setVolumeHandler(ROLOC::eLINEFINDER_VOLUME vol);
    void setParametersHandler(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq);

private slots:
    void pollROLOC();
    void modeChangeComplete();

private:
    void initROLOC();

    void getSignalStrengthSignalHandler();
    void getDepthMeasurementSignalHandler();

    double getMean(QList<quint8> values);
    double getVariance(QList<quint8> values);
    bool rolocHardwarePresent();
    void rolocSetVolume(ROLOC::eLINEFINDER_VOLUME vol);
    void rolocSetParameters(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ frequency);
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
    ROLOC::eLINEFINDER_MODE mCurrentMode;
    quint16 mROLOCsignalStrenth;
    double mROLOCdepthMeasurement;
    ROLOC::eLINEFINDER_VOLUME mCurrVolume;
    ROLOC::eLINEFINDER_FREQ mFrequency;

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
    return (mode == ROLOC_DBUS_API::eROLOC_MODE_OFF                   ? "mode off"          :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH   ? "mode sig strength" :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT ? "mode depth meas"   :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION           ? "mode cal"          :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST      ? "mode cal test"     :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_BALANCE               ? "mode balance"      : "???" ))))));
}

/**
 * @brief ROLOCcontroller::getString - convert dbus roloc freq to a string
 */
inline QString ROLOCcontroller::getString(ROLOC_DBUS_API::eROLOC_FREQUENCY freq)
{
    return (freq == ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE     ? "512hz sonde"     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE     ? "640hz sonde"     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE    ? "50hz passive"    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE    ? "60hz passive"    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE  ? "32.5khz active"  :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE ? "32.5khz passive" : "???" ))))));
}

#endif // RTSPSERVER_HPP
