#ifndef ROLOCARROWS_HPP
#define ROLOCARROWS_HPP

#include "inspRolocControllerDbus.hpp"

namespace ARROW
{

    // arrow status bitfield
    union tArrowBits
    {
        unsigned char status;
        struct
        {
            unsigned char reserved1   : 1;
            unsigned char centerArrow : 1;
            unsigned char rightArrow  : 1;
            unsigned char leftArrow   : 1;
            unsigned char reserved2   : 4;
        };

        /**
         * @brief tArrowBits - ctor
         */
        tArrowBits()
        {
            reset();
        }

        /**
         * @brief reset - reset the value
         */
        void reset()
        {
            status = 0;
        }
    };

}

/**
 * @brief The ROLOCArrows class - wrapper class around the arrow logic
 */
class ROLOCArrows
{
public:
    ROLOCArrows();
    void set(quint8 statusByte);
    ROLOC_DBUS_API::eROLOC_ARROW getDBusValue();
    QString getString();

private:
    ARROW::tArrowBits mArrow;
};

/**
 * @brief ROLOCArrows::getString - convert the arrow to a human readable string
 */
inline QString ROLOCArrows::getString()
{
    QString arrowStr = (mArrow.centerArrow ? "(center)"  :
                       (mArrow.leftArrow   ? "<-- left"  :
                       (mArrow.rightArrow  ? "right -->" : "???")));

    QString str = QString("statusByte = %1. arrow: %2")
            .arg( QString::number(mArrow.status, 16) )
            .arg(arrowStr);

    return str;
}


#endif // ROLOCARROWS_HPP
