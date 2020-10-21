#ifndef ROLOCINFOPACKET_HPP
#define ROLOCINFOPACKET_HPP

#include <QString>
#include "roloctypes.hpp"

namespace ROLOC_PACKET
{
    // packet type
    enum eTYPE
    {
        ePKT_TYPE_SETUP,
        ePKT_TYPE_CALIBRATION,
        ePKT_TYPE_DEPTH,
        ePKT_TYPE_SIG_STRENGTH,
        ePKT_TYPE_UNKNOWN
    };

    // data structure of a packet
    union tPacket
    {
        unsigned short packetData; // full word of data

        struct
        {
            // data byte (content is reflected from the status)
            unsigned char data;

            // status byte
            unsigned char muteAudio     : 1;  // b0: 1 = audio is muted
            unsigned char centerArrow   : 1;  // b1: 1 = show center arrow
            unsigned char rightArrow    : 1;  // b2: 1 = show right arrow
            unsigned char leftArrow     : 1;  // b3: 1 = show left arrow
            unsigned char isDepth       : 1;  // b4: 1 = data byte = depth data; 0 = following byte is signal strength
            unsigned char isCalibration : 1;  // b5: 1 = data byte = calibration data; display in hexadecimal
            unsigned char isSetup       : 1;  // b6: 1 = data byte = setup (echo following setup change only)
            unsigned char resvd1        : 1;  // b7: (unused)
        };

        /**
         * @brief tPacket - ctor
         */
        tPacket()
        {
            reset();
        }

        /**
         * @brief reset - reset the value
         */
        void reset()
        {
            packetData = 0;
        }
    };
}

/**
 * @brief The RolocInfoPacket class - object for the information read from the roloc
 */
class RolocInfoPacket
{
public:
    RolocInfoPacket();
    void set(unsigned short packet);

    ROLOC_PACKET::eTYPE getType() { return mType; }
    ROLOC::eLINEFINDER_ARROW getArrow() { return mArrow; }

    QString getString();
    QString getString(ROLOC_PACKET::eTYPE type);

private:
    ROLOC_PACKET::tPacket mPacket;
    ROLOC_PACKET::eTYPE mType;
    ROLOC::eLINEFINDER_ARROW mArrow;
};

/**
 * @brief RolocInfoPacket::getString - convert this object to a string
 */
inline QString RolocInfoPacket::getString()
{
    QString arrowStr = (mArrow == ROLOC::eARROW_CENTER ? "(center)"  :
                       (mArrow == ROLOC::eARROW_LEFT   ? "<-- left"  :
                       (mArrow == ROLOC::eARROW_RIGHT  ? "right -->" : "???")));

    QString str = QString("packet: 0x%1\n")
            .arg(mPacket.packetData, 4, 16, QChar('0'));

    str += QString("   type: %1\n").arg( getString(mType) );
    str += QString("   arrow: %1\n").arg(arrowStr);
    str += QString("   mute audio: %1\n" ).arg(mPacket.muteAudio ? "TRUE" : "FALSE");

    str += QString("   data = 0x%1 (%2)\n")
            .arg(mPacket.data, 2, 16, QChar('0'))
            .arg(QString::number(mPacket.data));

    return str;
}

/**
 * @brief RolocInfoPacket::getString - convert the type to a string
 */
inline QString RolocInfoPacket::getString(ROLOC_PACKET::eTYPE type)
{
    return (type == ROLOC_PACKET::ePKT_TYPE_SETUP        ? "SETUP"       :
           (type == ROLOC_PACKET::ePKT_TYPE_CALIBRATION  ? "CALIBRATION" :
           (type == ROLOC_PACKET::ePKT_TYPE_DEPTH        ? "DEPTH"       :
           (type == ROLOC_PACKET::ePKT_TYPE_SIG_STRENGTH ? "SIG STREN"   : "???" ))));
}

#endif // ROLOCINFOPACKET_HPP
