#include "rolocController.hpp"
#include "dbushandler.hpp"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication qCoreApp(argc, argv);
    DbusHandler dbushandler;
    ROLOCcontroller rolocController;

    QObject::connect(&rolocController, SIGNAL(getSignalStrength()), &dbushandler, SLOT(dbusSendSignalStrength()));
    QObject::connect(&rolocController, SIGNAL(getDepthMeasurement()), &dbushandler, SLOT(dbusSendDepthMeasurement()));
    QObject::connect(&rolocController, SIGNAL(getStatus()), &dbushandler, SLOT(dbusSendStatus()));
    QObject::connect(&rolocController, SIGNAL(sendSetVolume()), &dbushandler, SLOT(sendSetVolume()));

    dbushandler.init();
    rolocController.init(argc, argv);
    rolocController.start();

    return qCoreApp.exec();
}
