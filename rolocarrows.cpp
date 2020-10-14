#include "rolocarrows.hpp"

/**
 * @brief ROLOCArrows::ROLOCArrows - ctor
 */
ROLOCArrows::ROLOCArrows()
: mArrow()
{

}

/**
 * @brief ROLOCArrows::set - set the arrows from the i2c status byte
 */
void ROLOCArrows::set(quint8 statusByte)
{
    // set the data
    mArrow.status = statusByte;

    // if no direction is specified. we force it to center
    if (!mArrow.leftArrow || !mArrow.centerArrow || !mArrow.rightArrow)
    {
        mArrow.centerArrow = 1;
    }
}

/**
 * @brief ROLOCArrows::getDBusValue - use the arrow data to provide a dbus enumerated value
 */
ROLOC_DBUS_API::eROLOC_ARROW ROLOCArrows::getDBusValue()
{
    ROLOC_DBUS_API::eROLOC_ARROW arrow;

    // generate enumerated value from arrow bools
    if(mArrow.centerArrow)
    {
        arrow = ROLOC_DBUS_API::eCENTER_ARROW;
    }
    else if(mArrow.rightArrow)
    {
        arrow = ROLOC_DBUS_API::eRIGHT_ARROW;
    }
    else
    {
        arrow = ROLOC_DBUS_API::eLEFT_ARROW;
    }

    return arrow;
}

