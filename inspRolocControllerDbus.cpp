//#include <QtDBus/QDBusConnection>
#include "inspRolocControllerDbus.hpp"
#include "insproloccontroller_adaptor.h"
#include <QThread>

namespace
{
    const char* DBUS_OBJ_NAME       = "com.inspectron.inspROLOCcontroller";
    const char* DBUS_OBJ_PATH       = "/";
}

InspROLOCControllerDbus::InspROLOCControllerDbus()
{
}

InspROLOCControllerDbus::~InspROLOCControllerDbus()
{

}

void InspROLOCControllerDbus::init()
{
    new InspROLOCcontrollerAdaptor(this);

    bool status = QDBusConnection::systemBus().registerService(DBUS_OBJ_NAME);
    if(status == false)
    {
        qWarning() << "Failed to register dbus service " << DBUS_OBJ_NAME;
    }
    else
    {
        status = QDBusConnection::systemBus().registerObject(DBUS_OBJ_PATH, this);
        if (!status)
        {
            qWarning() << "Fail to register dbus object";
        }
        else
        {
            qDebug() << "Success registering inspROLOCcontroller service and object";
        }
    }
}

//sends volume signal dbux-> UI
void InspROLOCControllerDbus::sendVolume(int lvl)
{
    qDebug("DBus sendVolume: vol=%d", lvl);
    emit rolocUpdateVolume(lvl);
}

//rsends Parameters signal dbus -> UI
void InspROLOCControllerDbus::sendParameters(int mode, int freq)
{
    qDebug("DBus sendParameters: mode=%d freq=%d", mode, freq);
    emit rolocUpdateParameters(mode, freq);
}

// sends data report signal dbus -> UI
void InspROLOCControllerDbus::sendDataReport(
    int siglvl,
    double depth,
    int arrow,
    bool isConnected)
{
    qDebug("DBus sending report");
    emit rolocUpdateDataReport(siglvl, depth, arrow, isConnected);
}

// dbus methods UI -> roloc
void InspROLOCControllerDbus::rolocSetVolume(int lvl)
{
    qDebug("DBus setVolume: vol=%d", lvl);
    emit setVolumeHandler(lvl);
}

// dbus methods UI -> roloc
void InspROLOCControllerDbus::rolocSetParameters(int mode, int freq)
{
    qDebug("DBus setParameters: mode=%d freq=%d", mode, freq);
    emit setParametersHandler(mode, freq);
}

// dbus methods UI -> roloc
void InspROLOCControllerDbus::rolocGetDataReport()
{
    qDebug("DBus getDataReport");
    emit getDataReportHandler();
}
