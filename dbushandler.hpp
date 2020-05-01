#ifndef DBUSHANDLER_HPP
#define DBUSHANDLER_HPP

#include <QObject>

class DbusHandler : public QObject
{
    Q_OBJECT
public:
    DbusHandler();

    void init();

signals:
    //dbus signals
    void getSignalStrength();
    void getDepthMeasurement();
    void getStatus();
    void setVolume();

    //dbushandler needs internal signals to avoid
    //the thread pitfals of the glib main event loop
    void internalGetSignalStrength();
    void internalGetDepthMeasurement();
    void internalGetStatus();
    void internalSetVolume();

public slots:
    void sendSignalStrength();
    void sendDepthMeasurement();
    void sendGetStatus();
    void sendSetVolume();

private slots:
    void dbusSendSignalStrength();
    void dbusSendDepthMeasurement();
    void dbusSendStatus();
    void dbusSendVolume();
};

#endif // DBUSHANDLER_HPP
