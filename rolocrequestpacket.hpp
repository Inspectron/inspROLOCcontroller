#ifndef ROLOCREQUESTPACKET_HPP
#define ROLOCREQUESTPACKET_HPP

#include <QString>
#include "roloctypes.hpp"

namespace ROLOC_RQ_PACKET
{
    union tPacket
    {
        unsigned short packetData; // full word of data

        struct
        {
            // data byte (content is reflected from the status)
            unsigned char data;

            // status byte
            union
            {
               unsigned char status;
               struct
               {
                   unsigned char ctl0     : 1;  // b0: control 0
                   unsigned char rqDepth  : 1;  // b1: 1= make a depth measurement
                   unsigned char autoGain : 1;  // b2: 1 = automatic gain control mode
                   unsigned char freq     : 4;  // b3-6: frequency
                   unsigned char ctl1     : 1;  // b7: control 1
               };
            };

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
 * @brief The rolocRequestPacket class - object for requests to the ROLOC
 */
class RolocRequestPacket
{
public:
    RolocRequestPacket();
    void set(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq);
    unsigned char getData()   { return mPacket.data;   }
    unsigned char getStatus() { return mPacket.status; }
    unsigned char getDepth()  { return mPacket.rqDepth;}

    QString toString();
    QString getString(ROLOC::eLINEFINDER_FREQ freq);

private:
    void setMode(ROLOC::eLINEFINDER_MODE mode);
    void setFreq(ROLOC::eLINEFINDER_FREQ freq);

private:
    ROLOC_RQ_PACKET::tPacket mPacket;

};

/**
 * @brief RolocRequestPacket::toString - convert this object to a string
 */
inline QString RolocRequestPacket::toString()
{
    ROLOC::eLINEFINDER_FREQ freq = static_cast<ROLOC::eLINEFINDER_FREQ>(mPacket.freq << 11);
    QString freqStr = getString(freq);

    QString str = QString("packet: 0x%1\n")
            .arg(mPacket.packetData, 4, 16, QChar('0'));

    str += QString("   control      : %1%2  \n").arg(mPacket.ctl1).arg(mPacket.ctl0);
    str += QString("   requestDepth : %1    \n").arg(mPacket.rqDepth);
    str += QString("   autogain     : %1    \n").arg(mPacket.autoGain);
    str += QString("   frequency    : %1 %2 \n").arg(mPacket.freq, 4, 2, QChar('0')).arg(freqStr);
    str += QString("   data         : 0x%1  \n").arg(mPacket.data, 2, 16, QChar('0'));

    return str;
}

/**
 * @brief RolocRequestPacket::getString - convert freq to string
 */
inline QString RolocRequestPacket::getString(ROLOC::eLINEFINDER_FREQ freq)
{
    return (freq == ROLOC::eFREQ_512HZ_SONDE     ? "512Hz"           :
           (freq == ROLOC::eFREQ_640HZ_SONDE     ? "640Hz"           :
           (freq == ROLOC::eFREQ_50HZ_PASSIVE    ? "50Hz passive"    :
           (freq == ROLOC::eFREQ_60HZ_PASSIVE    ? "60Hz passive"    :
           (freq == ROLOC::eFREQ_32_5KHZ_ACTIVE  ? "32.5kHz active"  :
           (freq == ROLOC::eFREQ_32_5KHZ_PASSIVE ? "32.5kHz passive" : "???" ))))));
}

#endif // ROLOCREQUESTPACKET_HPP
