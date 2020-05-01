#include <QtDBus/QDBusConnection>
#include "dbushandler.hpp"
#include "insproloccontroller_adaptor.h"
#include <QThread>

namespace
{
    const char* DBUS_OBJ_NAME       = "com.inspectron.inspROLOCcontroller";
    const char* DBUS_OBJ_PATH       = "/";
}

DbusHandler::DbusHandler()
{
}

void DbusHandler::init()
{
    QObject::connect(this, SIGNAL(internalGetSignalStrength()), this, SLOT(dbusSendSignalStrength()));
    QObject::connect(this, SIGNAL(internalGetDepthMeasurement()), this, SLOT(dbusSendDepthMeasurement()));

    new InspROLOCcontrollerAdaptor(this);

    bool status = QDBusConnection::systemBus().registerService(DBUS_OBJ_NAME);
    if(status == false)
    {
        //g_print("Failed to register dbus service\n");
    }
    else
    {
        status = QDBusConnection::systemBus().registerObject(DBUS_OBJ_PATH, this);
        if (status == false)
        {
            //g_print("Fail to register dbus object!\n");
        }
    }
}

void DbusHandler::sendSignalStrength()
{
    emit internalGetSignalStrength();
}

void DbusHandler::sendDepthMeasurement()
{
    emit internalGetDepthMeasurement();
}

void DbusHandler::sendGetStatus()
{
    emit internalGetStatus();
}

void DbusHandler::sendSetVolume()
{
    emit internalSetVolume();
}



//recieves internal indication and sends it out to dbus
void DbusHandler::dbusSendSignalStrength()
{
    emit getSignalStrength();
}

//recieves internal indication and sends it out to dbus
void DbusHandler::dbusSendDepthMeasurement()
{
    emit getDepthMeasurement();
}

//recieves internal indication and sends it out to dbus
void DbusHandler::dbusSendStatus()
{
    emit getStatus();
}

//recieves internal indication and sends it out to dbus
void DbusHandler::dbusSendVolume()
{
    emit setVolume();
}

