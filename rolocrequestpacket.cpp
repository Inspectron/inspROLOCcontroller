#include "rolocrequestpacket.hpp"
#include <QDebug>

/**
 * @brief RolocRequestPacket::RolocRequestPacket - ctor
 */
RolocRequestPacket::RolocRequestPacket()
: mPacket()
{

}

/**
 * @brief RolocRequestPacket::set - set the packet data
 * @param mode - mode of the roloc
 * @param freq - freq of the roloc
 */
void RolocRequestPacket::set(ROLOC::eLINEFINDER_MODE mode, ROLOC::eLINEFINDER_FREQ freq)
{
    setMode(mode);
    setFreq(freq);

    // set the gain (automatic)
    mPacket.autoGain = 1;
}

/**
 * @brief RolocRequestPacket::setMode - set the mode of the roloc
 *
 * Mode truth table:
 *
 * |---------------------------------------------------------------------|
 * | Mode                 | ctl1 (b7) | freq (b6-b3)| AGC | depth | ctl0 |
 * |----------------------|-----------|-------------|-----|-------|------|
 * | sig strength (0x00)  |   0       |  xxxx       | x   |  0    | 0    |
 * | depth        (0x02)  |   0       |  xxxx       | x   |  1    | 0    |
 * | cal          (0x03)  |   0       |  xxxx       | x   |  1    | 1    |
 * | cal test     (0x82)  |   1       |  xxxx       | x   |  1    | 0    |
 * | balance      (0x83)  |   1       |  xxxx       | x   |  1    | 1    |
 * |---------------------------------------------------------------------|
 */
void RolocRequestPacket::setMode(ROLOC::eLINEFINDER_MODE mode)
{
    // the bits are set in each case since the enum constants
    // dont make sense & exceed the two bit range of permissible
    // values. CBU RFC0151 - SPECIFICATION LINE FINDER SPI INTERFACE
    switch (mode)
    {
    case ROLOC::eMODE_GET_SIGNAL_STRENGTH:
        // set the control bits
        mPacket.ctl0 = 0;
        mPacket.ctl1 = 0;
        // set the depth bit
        mPacket.rqDepth = 0;
        break;

    case ROLOC::eMODE_GET_DEPTH_MEASUREMENT:
        // set the control bits
        mPacket.ctl0 = 0;
        mPacket.ctl1 = 0;
        // set the depth bit
        mPacket.rqDepth = 1;
        break;

    case ROLOC::eMODE_CALIBRATION:
        // TODO implement
        mPacket.ctl0 = 1;
        mPacket.ctl1 = 0;
        // set the depth bit
        mPacket.rqDepth = 1;
        break;

    case ROLOC::eMODE_CALIBRATION_TEST:
        // set the control bits
        mPacket.ctl0 = 0;
        mPacket.ctl1 = 1;
        // set the depth bit
        mPacket.rqDepth = 1;
        break;

    case ROLOC::eMODE_BALANCE:
        // TODO implement
        mPacket.ctl0 = 1;
        mPacket.ctl1 = 1;
        // set the depth bit
        mPacket.rqDepth = 1;
        break;

    default:
        qFatal("invalid mode !!!");
        break;
    }
}

/**
 * @brief RolocRequestPacket::setFreq - set the frequency of the roloc
 */
void RolocRequestPacket::setFreq(ROLOC::eLINEFINDER_FREQ freq)
{
    // the bits are set in each case since the enum constants
    // have an uncessessary bit shift, which is corrected in the
    // i2c class. It looks like this was done to send the correct
    // endianness ???
    // TODO: once it all works, the bit shifts should be removed and fixed
    // correctly

    switch (freq)
    {
    case ROLOC::eFREQ_512HZ_SONDE    :
    case ROLOC::eFREQ_640HZ_SONDE    :
    case ROLOC::eFREQ_50HZ_PASSIVE   :
    case ROLOC::eFREQ_60HZ_PASSIVE   :
    case ROLOC::eFREQ_32_5KHZ_ACTIVE :
    case ROLOC::eFREQ_32_5KHZ_PASSIVE:
        mPacket.freq = (freq >> 11);
        break;
    default:
            qFatal("invalid freq !!!");
        break;
    }
}

