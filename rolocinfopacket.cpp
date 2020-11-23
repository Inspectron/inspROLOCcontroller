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

#if 0
    // DM: Nov 23. theres a problem with the data. dont range limit for now
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
#endif
}

// the following are also used as command masks
#define LINEFINDER_MODE_GET_SIGNAL_STRENGTH    (0x0000)
#define LINEFINDER_MODE_GET_DEPTH_MEASUREMENT  (0x0002)
#define LINEFINDER_MODE_CALIBRATION            (0x0001 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT)
#define LINEFINDER_MODE_CALIBRATION_TEST       (0x0080 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT)
#define LINEFINDER_MODE_BALANCE                (0x0081 | LINEFINDER_MODE_GET_DEPTH_MEASUREMENT)

#define LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED 0x0010
#define LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    0x0020
#define LINEFINDER_PING_DATA_RETURNED              0x0040
#define LINEFINDER_CAL_FAILURE_VALUE			253

// data just to compare against the oldschool 
// way of extracting the data
QString RolocInfoPacket::oldschoolData()
{
    unsigned short data = mUnalteredPacketData;
    QString str = "";
    bool depthOrCalTest;
    bool calOrBalance;
    bool ping;
    bool specialDataReceived;


    depthOrCalTest = (data & (LINEFINDER_DEPTH_OR_CAL_TEST_DATA_RETURNED << 8)); //   0x1000));
    calOrBalance   = (data & (LINEFINDER_CAL_OR_BALANCE_DATA_RETURNED    << 8));
    ping           = (data & (LINEFINDER_PING_DATA_RETURNED              << 8));
    specialDataReceived = (depthOrCalTest || calOrBalance || ping);

    str += QString("depthCal,calBal,ping,special = %1, %2, %3, %4 \n")
            .arg(depthOrCalTest)
            .arg(calOrBalance)
            .arg(ping)
            .arg(specialDataReceived);

    if (depthOrCalTest || calOrBalance)
    {
        str += QString("depthorCal||cal or bal ===> 0x%1 \n").arg((data & 0x00FF));
    }
    else if (ping)
    {
        str += QString(" PING \n");
#if 0
        // verify correct frequency setting
        // tbd:
        target = s_LFfreqMask[GetLFfreq()] >> 8;
        found  = (data & 0x0078);
        if (target != found)
            asm("nop;"); // jumped out of freq!!!!
#endif
    }
    else
    {
        //g_nLinefinderSignalStrength = data;
        str += QString("linefinder sig stren = %1").arg(data);
    }


    return str;
}

