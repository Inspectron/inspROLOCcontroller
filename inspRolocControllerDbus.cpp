//#include <QtDBus/QDBusConnection>
#include "inspRolocControllerDbus.hpp"
#include "insproloccontroller_adaptor.h"
#include <QThread>
#define DBG_BLOCK 0

namespace
{
    const char* DBUS_OBJ_NAME       = "com.inspectron.inspROLOCcontroller";
    const char* DBUS_OBJ_PATH       = "/";
}

/**
 * @brief InspROLOCControllerDbus::InspROLOCControllerDbus - ctor
 */
InspROLOCControllerDbus::InspROLOCControllerDbus()
{
}

/**
 * @brief InspROLOCControllerDbus::~InspROLOCControllerDbus - dtor
 */
InspROLOCControllerDbus::~InspROLOCControllerDbus()
{
}

/**
 * @brief InspROLOCControllerDbus::init - init the interface
 */
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
void InspROLOCControllerDbus::sendVolume(ROLOC::eLINEFINDER_VOLUME lvl)
{
    qDebug("DBus sendVolume: vol=%d", lvl);
    emit rolocUpdateVolume(lvl);
}

// sends Parameters signal dbus -> UI
void InspROLOCControllerDbus::sendParameters(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq)
{
    qDebug("DBus sendParameters: mode=%d freq=%d", mode, freq);
    emit rolocUpdateParameters(getModeDBUS(mode), getFrequencyDBUS(freq));
}

// sends data report signal dbus -> UI
void InspROLOCControllerDbus::sendDataReport(
    ROLOC::eLINEFINDER_MODE mode,
    ROLOC::eLINEFINDER_FREQ frequency,
    int siglvl,
    double depth,
    int arrow,
    bool isPresent)
{
#if DBG_BLOCK
    qDebug("DBus sending report");
#endif
    emit rolocUpdateDataReport(getModeDBUS(mode), getFrequencyDBUS(frequency), siglvl, depth, arrow, isPresent);
}

// sends data preset signal dbus -> UI
void InspROLOCControllerDbus::sendPresent(bool isPresent)
{
#if DBG_BLOCK
    qDebug("DBus sending report");
#endif
    emit rolocUpdatePresent(isPresent);
}

// dbus methods UI -> roloc
void InspROLOCControllerDbus::rolocSetVolume(int lvl)
{
    qDebug("DBus setVolume: vol=%d", lvl);
    if (lvl >= ROLOC_DBUS_API::eROLOC_VOLUME_OFF && lvl <= ROLOC_DBUS_API::eROLOC_VOLUME_MAX)
    {
        emit setVolumeHandler(static_cast<ROLOC::eLINEFINDER_VOLUME>(lvl));
    }
    else
    {
        qWarning() << "illegal volume requested" << lvl << "!!! turn it off !!!";
        // invalid data. turn the volume off
        emit setVolumeHandler(ROLOC::eVOLUME_OFF);
    }
}

// dbus methods UI -> roloc
void InspROLOCControllerDbus::rolocSetParameters(int mode, int freq)
{
    ROLOC::eLINEFINDER_MODE rolocMode = getMode(mode);
    ROLOC::eLINEFINDER_FREQ rolocFreq = getFreq(freq);

    if ((rolocMode == ROLOC::eMODE_INVALID) || (rolocFreq == ROLOC::eFREQ_INVALID))
    {
        qWarning() << "invalid mode " << mode << "or frequency" << freq;
    }
    else
    {
        emit setParametersHandler(rolocMode, rolocFreq);
    }
}

/**
 * @brief InspROLOCControllerDbus::rolocSetMode - dbus slot to set the mode
 */
void InspROLOCControllerDbus::rolocSetMode(int mode)
{
    qCritical() << "request to set the mode to " << getString(static_cast<ROLOC_DBUS_API::eROLOC_MODE>(mode)); // TODO test and remove

    ROLOC::eLINEFINDER_MODE rolocMode = getMode(mode);

    if (rolocMode == ROLOC::eMODE_INVALID)
    {
        qWarning() << "invalid mode " << mode;
    }
    else
    {
        emit requestSetMode(rolocMode);
    }
}

/**
 * @brief InspROLOCControllerDbus::rolocSetFrequency - dbus slot to set the frequency
 */
void InspROLOCControllerDbus::rolocSetFrequency(int freq)
{
    qCritical() << "request to set the freq to " << getString(static_cast<ROLOC_DBUS_API::eROLOC_FREQUENCY>(freq)); // TODO test and remove

    ROLOC::eLINEFINDER_FREQ rolocFreq = getFreq(freq);

    if (rolocFreq == ROLOC::eFREQ_INVALID)
    {
        qWarning() << "invalid frequency" << freq;
    }
    else
    {
        emit requestSetFreq(rolocFreq);
    }
}


// dbus methods UI -> roloc
void InspROLOCControllerDbus::rolocGetDataReport()
{
    qDebug("DBus getDataReport");
    emit getDataReportHandler();
}

/**
 * @brief InspROLOCControllerDbus::getFrequencyDBUS -- convert the linefinder freq to dbus API
 */
ROLOC_DBUS_API::eROLOC_FREQUENCY InspROLOCControllerDbus::getFrequencyDBUS(ROLOC::eLINEFINDER_FREQ frequency)
{

    return (frequency == ROLOC::eFREQ_512HZ_SONDE     ? ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE     :
           (frequency == ROLOC::eFREQ_640HZ_SONDE     ? ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE     :
           (frequency == ROLOC::eFREQ_50HZ_PASSIVE    ? ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE    :
           (frequency == ROLOC::eFREQ_60HZ_PASSIVE    ? ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE    :
           (frequency == ROLOC::eFREQ_32_5KHZ_ACTIVE  ? ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE  :
                                                        ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE )))));
}

/**
 * @brief InspROLOCControllerDbus::getModeDBUS -- convert the linefinder mode to dbus API
 */
ROLOC_DBUS_API::eROLOC_MODE InspROLOCControllerDbus::getModeDBUS(ROLOC::eLINEFINDER_MODE mode)
{
    return (mode == ROLOC::eMODE_GET_SIGNAL_STRENGTH   ? ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH   :
           (mode == ROLOC::eMODE_GET_DEPTH_MEASUREMENT ? ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT :
           (mode == ROLOC::eMODE_CALIBRATION           ? ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION           :
           (mode == ROLOC::eMODE_CALIBRATION_TEST      ? ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST      :
           (mode == ROLOC::eMODE_BALANCE               ? ROLOC_DBUS_API::eROLOC_MODE_BALANCE               :
                                                         ROLOC_DBUS_API::eROLOC_MODE_OFF )))));
}

/**
 * @brief InspROLOCControllerDbus::getMode - convert dbus API to roloc data
 */
ROLOC::eLINEFINDER_MODE InspROLOCControllerDbus::getMode(int mode)
{
    return (mode == ROLOC_DBUS_API::eROLOC_MODE_GET_SIGNAL_STRENGTH   ? ROLOC::eMODE_GET_SIGNAL_STRENGTH   :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_GET_DEPTH_MEASUREMENT ? ROLOC::eMODE_GET_DEPTH_MEASUREMENT :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION           ? ROLOC::eMODE_CALIBRATION           :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_CALIBRATION_TEST      ? ROLOC::eMODE_CALIBRATION_TEST      :
           (mode == ROLOC_DBUS_API::eROLOC_MODE_BALANCE               ? ROLOC::eMODE_BALANCE               : ROLOC::eMODE_INVALID )))));
}

/**
 * @brief InspROLOCControllerDbus::getFreq - convert dbus API to roloc data
 */
ROLOC::eLINEFINDER_FREQ InspROLOCControllerDbus::getFreq(int freq)
{
    return (freq == ROLOC_DBUS_API::eROLOC_FREQ_512HZ_SONDE     ? ROLOC::eFREQ_512HZ_SONDE     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_640HZ_SONDE     ? ROLOC::eFREQ_640HZ_SONDE     :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_50HZ_PASSIVE    ? ROLOC::eFREQ_50HZ_PASSIVE    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_60HZ_PASSIVE    ? ROLOC::eFREQ_60HZ_PASSIVE    :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_ACTIVE  ? ROLOC::eFREQ_32_5KHZ_ACTIVE  :
           (freq == ROLOC_DBUS_API::eROLOC_FREQ_32_5KHZ_PASSIVE ? ROLOC::eFREQ_32_5KHZ_PASSIVE : ROLOC::eFREQ_INVALID ))))));
}
