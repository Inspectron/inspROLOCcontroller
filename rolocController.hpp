#ifndef ROLOCCONTROLLER_HPP
#define ROLOCCONTROLLER_HPP

#include <QThread>
#include <QTimer>
#include <QQueue>
#include "i2c.hpp"
#include "inspRolocControllerDbus.hpp"
#include "rolocinfopacket.hpp"
#include "roloctypes.hpp"



class ROLOCcontroller
: public QObject
{
    Q_OBJECT
public:
    ROLOCcontroller();
    virtual ~ROLOCcontroller();

    void init();
    void rolocBusy(ROLOC::eSTATE nextState);
    void sendDataReport();
    void sendDepthReport();


signals:
    void updateStatus(int status);
    void updateSSR(int ssr);
    void updateDepth(double depth);
    void updateVolume(int lvl);

public slots:
    void getDataReportHandler();
    void setVolumeHandler    (ROLOC::eLINEFINDER_VOLUME vol);
    void setParametersHandler(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq);
    void setFrequencyHandler (ROLOC::eLINEFINDER_FREQ freq);
    void setModeHandler      (ROLOC::eLINEFINDER_MODE mode);
    void rolocSetDepthMode(ROLOC::eLINEFINDER_MODE mode);


private slots:
    void pollROLOC();
    void onFreqSetTimerExpired();

private:
    void initROLOC();

    void getSignalStrengthSignalHandler();
    void getDepthMeasurementSignalHandler();

    double getMean(QList<quint8> values);
    double getVariance(QList<quint8> values);
    bool rolocHardwarePresent();
    void rolocSetVolume(ROLOC::eLINEFINDER_VOLUME vol);
    void rolocSetParameters(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ frequency);
    quint16 rolocGetData();
    void processRolocData();
    QString getString(ROLOC::eSTATE state);
    QString getString(ROLOC::eLINEFINDER_FREQ freq);

    i2c m_i2cBus;
    quint8 mI2cAddr;
    InspROLOCControllerDbus &mDbusHandler;

    ROLOC::eLINEFINDER_MODE mCurrentMode;
    quint16 mROLOCsignalStrenth;
    double mROLOCdepthMeasurement;
    ROLOC::eLINEFINDER_VOLUME mCurrVolume;
    ROLOC::eLINEFINDER_FREQ mFrequency;

    quint8 mNumSamples;
    QList<quint8> mDepthAccumulator;

    RolocInfoPacket &mInfoPacket;
    QTimer *mpRolocDataPollingTimer;
    ROLOC::eSTATE mCurrentState;
    QTimer *mpFreqencySetTimer;
    ROLOC::eLINEFINDER_FREQ mPendingFreq;
    int mDisplayRetry;
    int mBadReadCount;
    bool mPrevPresent;
};

/**
 * @brief getString - convert the state to a string
 */
inline QString ROLOCcontroller::getString(ROLOC::eSTATE state)
{
    return (state == ROLOC::eSTATE_DISCONNECTED ? "DISCONNECTED" :
           (state == ROLOC::eSTATE_OPERATING    ? "OPERATING"    :
           (state == ROLOC::eSTATE_INITIALIZING ? "INITIALIZING" :
           (state == ROLOC::eSTATE_BUSY         ? "BUSY"         : "???" ))));
}

/**
 * @brief HMIMain::getString - convert freq to a string
 */
inline QString ROLOCcontroller::getString(ROLOC::eLINEFINDER_FREQ freq)
{
    return (freq == ROLOC::eFREQ_512HZ_SONDE     ? "512 Hz"           :
           (freq == ROLOC::eFREQ_640HZ_SONDE     ? "640 Hz"           :
           (freq == ROLOC::eFREQ_50HZ_PASSIVE    ? "50 Hz passive"    :
           (freq == ROLOC::eFREQ_60HZ_PASSIVE    ? "60 Hz passive"    :
           (freq == ROLOC::eFREQ_32_5KHZ_ACTIVE  ? "32.5 kHz active"  :
           (freq == ROLOC::eFREQ_32_5KHZ_PASSIVE ? "32.5 kHz passive" : "???" ))))));
}

#endif // RTSPSERVER_HPP
