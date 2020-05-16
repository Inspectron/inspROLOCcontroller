#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication qCoreApp(argc, argv);
    ROLOCcontroller rolocController;

    rolocController.init(argc, argv);

    return qCoreApp.exec();
}
