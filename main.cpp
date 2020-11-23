#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"
#include "include/inspcore.hpp"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    // set debug formatting
    InspCore::setFormattedDebugOutput("inspROLOCcontroller");

    qDebug() << "-------------------------------";
    qDebug() << "rolocController v11.23.2020"
    qDebug() << "-------------------------------";

    // start the core application
    QCoreApplication qCoreApp(argc, argv);
    ROLOCcontroller rolocController;

    rolocController.init();

    return qCoreApp.exec();
}
