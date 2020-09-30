#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"
#include "include/inspcore.hpp"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    // set debug formatting
    InspCore::setFormattedDebugOutput("inspROLOCcontroller");

    // start the core application
    QCoreApplication qCoreApp(argc, argv);
    ROLOCcontroller rolocController;

    rolocController.init(argc, argv);

    return qCoreApp.exec();
}
