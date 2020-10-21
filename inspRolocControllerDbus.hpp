#ifndef DBUSHANDLER_HPP
#define DBUSHANDLER_HPP

#include <QObject>
#include "roloctypes.hpp"

namespace ROLOC_DBUS_API
{
    enum eROLOC_ARROW
    {
        eLEFT_ARROW   = ROLOC::eARROW_LEFT,
        eCENTER_ARROW = ROLOC::eARROW_CENTER,
        eRIGHT_ARROW  = ROLOC::eARROW_RIGHT,
    };

    enum eROLOC_MODE
    {
        eROLOC_MODE_OFF,
        eROLOC_MODE_GET_SIGNAL_STRENGTH,
        eROLOC_MODE_GET_DEPTH_MEASUREMENT,
        eROLOC_MODE_CALIBRATION,
        eROLOC_MODE_CALIBRATION_TEST,
        eROLOC_MODE_BALANCE
    };

    enum eROLOC_FREQUENCY
    {
       eROLOC_FREQ_512HZ_SONDE,
       eROLOC_FREQ_640HZ_SONDE,
       eROLOC_FREQ_50HZ_PASSIVE,
       eROLOC_FREQ_60HZ_PASSIVE,
       eROLOC_FREQ_32_5KHZ_ACTIVE,
       eROLOC_FREQ_32_5KHZ_PASSIVE,
    };

    enum eROLOC_VOLUME
    {
        eROLOC_VOLUME_OFF  = ROLOC::eVOLUME_OFF,
        eROLOC_VOLUME_MED  = ROLOC::eVOLUME_MED,
        eROLOC_VOLUME_HIGH = ROLOC::eVOLUME_HIGH,
        eROLOC_VOLUME_MAX  = ROLOC::eVOLUME_HIGH
    };
}

class InspROLOCControllerDbus : public QObject
{
    Q_OBJECT
public:
    InspROLOCControllerDbus();
    ~InspROLOCControllerDbus();

    void init();


signals:
    //dbus signals -> controller
    void getDataReportHandler();
    void setVolumeHandler(ROLOC::eLINEFINDER_VOLUME lvl);
    void setParametersHandler(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq);
    void requestSetMode(ROLOC::eLINEFINDER_MODE mode);
    void requestSetFreq(ROLOC::eLINEFINDER_FREQ freq);
    //roloc -> dbus signal
    void rolocUpdateDataReport(int mode, int frequency, int siglvl, double depth, int arrow, bool isPresent);
    void rolocUpdatePresent(bool isPresent);
    void rolocUpdateParameters(int mode, int frequency);
    void rolocUpdateVolume(int volume);

public slots:
    // client -> roloc dbus functions
    void rolocGetDataReport();
    void rolocSetVolume(int lvl);
    void rolocSetParameters(int mode, int freq);
    void rolocSetMode(int mode);
    void rolocSetFrequency(int freq);

    //dbus send signals -> client
    void sendVolume(ROLOC::eLINEFINDER_VOLUME volume);
    void sendParameters(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq);
    void sendDataReport(
        ROLOC::eLINEFINDER_MODE mode,
        ROLOC::eLINEFINDER_FREQ frequency,
        int siglvl,
        double depth,
        int arrow,
        bool isPresent);
    void sendPresent(bool isPresent);

private:
    ROLOC_DBUS_API::eROLOC_FREQUENCY getFrequencyDBUS(ROLOC::eLINEFINDER_FREQ frequency);
    ROLOC_DBUS_API::eROLOC_MODE getModeDBUS(ROLOC::eLINEFINDER_MODE mode);
    ROLOC::eLINEFINDER_MODE getMode(int mode);
    ROLOC::eLINEFINDER_FREQ getFreq(int freq);
    QString getString(ROLOC_DBUS_API::eROLOC_MODE mode);
    QString getString(ROLOC_DBUS_API::eROLOC_FREQUENCY freq);

};

/**
 * @brief ROLOCcontroller::getString - convert dbus roloc mode to a string
 */
inline QString InspROLOCControllerDbus::getString(ROLOC_DBUS_API::eROLOC_MODE mode)
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
inline QString InspROLOCControllerDbus::getString(ROLOC_DBUS_API::eROLOC_FREQUENCY freq)
{
    return (freq == ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE     ? "512hz sonde"     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE     ? "640hz sonde"     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE    ? "50hz passive"    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE    ? "60hz passive"    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE  ? "32.5khz active"  :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE ? "32.5khz passive" : "???" ))))));
}

#endif // DBUSHANDLER_HPP
