#ifndef DBUSHANDLER_HPP
#define DBUSHANDLER_HPP

#include <QObject>

namespace ROLOC_DBUS_API
{
    enum eROLOC_ARROW
    {
        eLEFT_ARROW,
        eCENTER_ARROW,
        eRIGHT_ARROW,
        //
        eROLOC_ARROW_MAX
    };

    enum eROLOC_MODE
    {
        eROLOC_MODE_NOCHANGE,
        eROLOC_MODE_OFF,
        eROLOC_MODE_GET_SIGNAL_STRENGTH,
        eROLOC_MODE_GET_DEPTH_MEASUREMENT,
        eROLOC_MODE_CALIBRATION,
        eROLOC_MODE_CALIBRATION_TEST,
        eROLOC_MODE_BALANCE,
        //
        eROLOC_MODE_MAX
    };

    enum eROLOC_FREQUENCY
    {
       eROLOC_FREQ_NOCHANGE,
       eROLOC_FREQ_512HZ_SONDE,
       eROLOC_FREQ_640HZ_SONDE,
       eROLOC_FREQ_50HZ_PASSIVE,
       eROLOC_FREQ_60HZ_PASSIVE,
       eROLOC_FREQ_32_5KHZ_ACTIVE,
       eROLOC_FREQ_32_5KHZ_PASSIVE,
       //
       eROLOC_FREQ_MAX
    };

    enum eROLOC_VOLUME
    {
        eROLOC_VOLUME_OFF,
        eROLOC_VOLUME_MED,
        eROLOC_VOLUME_HIGH,
        //
        eROLOC_VOLUME_MAX
    };
};

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
    void setVolumeHandler(int lvl);
    void setParametersHandler(int mode, int freq);
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

    //dbus send signals -> client
    void sendVolume(int volume);
    void sendParameters(int mode, int freq);
    void sendDataReport(
        int mode,
        int frequency,
        int siglvl,
        double depth,
        int arrow,
        bool isPresent);
    void sendPresent(bool isPresent);
};

#endif // DBUSHANDLER_HPP
