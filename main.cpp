#include "rolocController.hpp"
#include "inspRolocControllerDbus.hpp"
#include "include/inspcore.hpp"
#include <QCoreApplication>

#include <QDebug>
const int    PARAM_LF_FREQ_512HZ_SONDE                  = (0 << 11);
const int    PARAM_LF_FREQ_640HZ_SONDE                  = (1 << 11);
const int    PARAM_LF_FREQ_50HZ_PASSIVE                 = (2 << 11);
const int    PARAM_LF_FREQ_60HZ_PASSIVE                 = (3 << 11);
const int    PARAM_LF_FREQ_32_5KHZ_ACTIVE               = (8 << 11);
const int    PARAM_LF_FREQ_32_5KHZ_PASSIVE              = (9 << 11);


QString getHex(int num)
{
    return QString("0x%1").arg(QString::number(num, 16));
}


int main(int argc, char *argv[])
{
    // set debug formatting
    InspCore::setFormattedDebugOutput("inspROLOCcontroller");

    // start the core application
    QCoreApplication qCoreApp(argc, argv);
    //ROLOCcontroller rolocController;
    //
    //rolocController.init();

    qDebug() << "PARAM_LF_FREQ_512HZ_SONDE      = (0 << 11) ====> " << getHex(PARAM_LF_FREQ_512HZ_SONDE);
    qDebug() << "PARAM_LF_FREQ_640HZ_SONDE      = (1 << 11) ====> " << getHex(PARAM_LF_FREQ_640HZ_SONDE);
    qDebug() << "PARAM_LF_FREQ_50HZ_PASSIVE     = (2 << 11) ====> " << getHex(PARAM_LF_FREQ_50HZ_PASSIVE);
    qDebug() << "PARAM_LF_FREQ_60HZ_PASSIVE     = (3 << 11) ====> " << getHex(PARAM_LF_FREQ_60HZ_PASSIVE);
    qDebug() << "PARAM_LF_FREQ_32_5KHZ_ACTIVE   = (8 << 11) ====> " << getHex(PARAM_LF_FREQ_32_5KHZ_ACTIVE);
    qDebug() << "PARAM_LF_FREQ_32_5KHZ_PASSIVE  = (9 << 11) ====> " << getHex(PARAM_LF_FREQ_32_5KHZ_PASSIVE);

    return qCoreApp.exec();
}
