#include "rolocarrows.hpp"

// anonymous namespace
namespace
{
    const int    LINEFINDER_LEFT_ARROW_BIT                  = 3;
    const int    LINEFINDER_RIGHT_ARROW_BIT                 = 2;
    const int    LINEFINDER_CENTER_ARROW_BIT                = 1;
}

/**
 * @brief ROLOCArrows::ROLOCArrows - ctor
 */
ROLOCArrows::ROLOCArrows()
: mStatusByte(0x00)
, mbLeftArrow  (false)
, mbRightArrow (false)
, mbCenterArrow(false)
{

}

/**
 * @brief ROLOCArrows::set - set the arrows from the i2c status byte
 */
void ROLOCArrows::set(quint8 statusByte)
{
    // reset the data
    reset();

    // acquire new data
    mStatusByte = statusByte;
    mbLeftArrow   = (statusByte >> LINEFINDER_LEFT_ARROW_BIT)   & 1;
    mbCenterArrow = (statusByte >> LINEFINDER_CENTER_ARROW_BIT) & 1;
    mbRightArrow  = (statusByte >> LINEFINDER_RIGHT_ARROW_BIT)  & 1;

    // if no direction is specified. we force it to center
    if (!mbLeftArrow || !mbCenterArrow || !mbRightArrow)
    {
        mbCenterArrow = true;
    }
}

/**
 * @brief ROLOCArrows::reset - reset the arrows values
 */
void ROLOCArrows::reset()
{
    mStatusByte   = 0x00;
    mbLeftArrow   = false;
    mbRightArrow  = false;
    mbCenterArrow = false;
}

/**
 * @brief ROLOCArrows::getDBusValue - use the arrow data to provide a dbus enumerated value
 */
ROLOC_DBUS_API::eROLOC_ARROW ROLOCArrows::getDBusValue()
{
    ROLOC_DBUS_API::eROLOC_ARROW arrow;

    // generate enumerated value from arrow bools
    if(mbCenterArrow)
    {
        arrow = ROLOC_DBUS_API::eCENTER_ARROW;
    }
    else if(mbRightArrow)
    {
        arrow = ROLOC_DBUS_API::eRIGHT_ARROW;
    }
    else
    {
        arrow = ROLOC_DBUS_API::eLEFT_ARROW;
    }

    return arrow;
}

