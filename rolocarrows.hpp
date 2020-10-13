#ifndef ROLOCARROWS_HPP
#define ROLOCARROWS_HPP

#include "inspRolocControllerDbus.hpp"

/**
 * @brief The ROLOCArrows class - wrapper class around the arrow logic
 */
class ROLOCArrows
{
public:
    ROLOCArrows();
    void set(quint8 statusByte);
    void set(bool left, bool right, bool center, bool none);
    void reset();
    ROLOC_DBUS_API::eROLOC_ARROW getDBusValue();
    QString getString();

private:
    quint8 mStatusByte;
    bool mbLeftArrow;
    bool mbRightArrow;
    bool mbCenterArrow;

};

/**
 * @brief ROLOCArrows::getString - convert the arrow to a human readable string
 */
inline QString ROLOCArrows::getString()
{
    QString arrowStr = (mbCenterArrow ? "(center)"  :
                       (mbLeftArrow   ? "<-- left"  :
                       (mbRightArrow  ? "right -->" : "???")));

    QString str = QString("statusByte = %1. arrow: %2")
            .arg( QString::number(mStatusByte, 16) )
            .arg(arrowStr);

    return str;
}


#endif // ROLOCARROWS_HPP
