#include "rolocinfopacket.hpp"
#include <QDebug>

namespace
{
    const int MAX_SIG_DEPTH_VAL = 240;
    const int MIN_SIG_DEPTH_VAL = 10;
}

/**
 * @brief RolocInfoPacket::RolocInfoPacket
 */
RolocInfoPacket::RolocInfoPacket()
: mUnalteredPacketData(0)
, mPacket()
, mType(ROLOC_PACKET::ePKT_TYPE_UNKNOWN)
, mArrow(ROLOC::eARROW_CENTER)
{
}

/**
 * @brief RolocInfoPacket::set - set the packet data
 * @param packet - data read from i2c (SSSS, DDDD)
 * SSSS = status byte
 * DDDD = data byte
 */
void RolocInfoPacket::set(unsigned short packet)
{
    // set the pkt data
    mUnalteredPacketData = packet;
    mPacket.packetData = packet;

    // determine the type
    if (mPacket.isSetup)
    {
        mType = ROLOC_PACKET::ePKT_TYPE_SETUP;
    }
    else if(mPacket.isCalibration)
    {
        mType = ROLOC_PACKET::ePKT_TYPE_CALIBRATION;
    }
    else if(mPacket.isDepth)
    {
        mType = ROLOC_PACKET::ePKT_TYPE_DEPTH;
    }
    else
    {
        mType = ROLOC_PACKET::ePKT_TYPE_SIG_STRENGTH;
    }

    // set the arrow
    if (mPacket.centerArrow)
    {
        mArrow = ROLOC::eARROW_CENTER;
    }
    else if (mPacket.leftArrow)
    {
        mArrow = ROLOC::eARROW_LEFT;
    }
    else if (mPacket.rightArrow)
    {
        mArrow = ROLOC::eARROW_RIGHT;
    }
    else
    {
        // if no direction is specified. force it to center
        mArrow = ROLOC::eARROW_CENTER;
    }

    // range limit the data
    if ((mType == ROLOC_PACKET::ePKT_TYPE_SIG_STRENGTH) || (mType == ROLOC_PACKET::ePKT_TYPE_DEPTH))
    {
        if (mPacket.data > MAX_SIG_DEPTH_VAL)
        {
            mPacket.data = MAX_SIG_DEPTH_VAL;
        }
        else if (mPacket.data < MIN_SIG_DEPTH_VAL)
        {
            mPacket.data = 0;
        }
    }
}
