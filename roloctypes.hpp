#ifndef ROLOCTYPES_HPP
#define ROLOCTYPES_HPP

namespace ROLOC
{
    // modes of operation
    enum eLINEFINDER_MODE
    {
        // TODO these should be changed to plain enums
        eMODE_INVALID                    =  -1,
        eMODE_GET_SIGNAL_STRENGTH        = (0x0000),
        eMODE_GET_DEPTH_MEASUREMENT      = (0x0002),
        eMODE_CALIBRATION                = (0x0001 | eMODE_GET_DEPTH_MEASUREMENT),
        eMODE_CALIBRATION_TEST           = (0x0080 | eMODE_GET_DEPTH_MEASUREMENT),
        eMODE_BALANCE                    = (0x0081 | eMODE_GET_DEPTH_MEASUREMENT),
    };

    // volume levels
    enum eLINEFINDER_VOLUME
    {
        eVOLUME_OFF                     =   0x00,
        eVOLUME_MED                     =   0x01,
        eVOLUME_HIGH                    =   0x02,
    };

    // commands
    enum eLINEFINDER_CMD
    {
        eCMD_GET_ID                         =   0xDC,
        eCMD_VOLUME                         =   0x07,
        eCMD_INFO                           =   0xFA,
    };

    // TODO why is it shifted by 11 ? the CBU docs reserve 4 bits for this, not 16
    enum eLINEFINDER_FREQ
    {
        eFREQ_INVALID                      = -1,
        eFREQ_512HZ_SONDE                  = 0x0000, // (0 << 11);
        eFREQ_640HZ_SONDE                  = 0x0800, // (1 << 11);
        eFREQ_50HZ_PASSIVE                 = 0x1000, // (2 << 11);
        eFREQ_60HZ_PASSIVE                 = 0x1800, // (3 << 11);
        eFREQ_32_5KHZ_ACTIVE               = 0x4000, // (8 << 11);
        eFREQ_32_5KHZ_PASSIVE              = 0x4800, // (9 << 11);
    };

    // arrow types
    enum eLINEFINDER_ARROW
    {
        eARROW_LEFT,
        eARROW_CENTER,
        eARROW_RIGHT
    };

    // operational states
    enum eSTATE
    {
        eSTATE_DISCONNECTED,    // nothing is connected
        eSTATE_OPERATING,       // currently operating
        eSTATE_INITIALIZING,    // initializing the roloc after plug in
        eSTATE_BUSY             // busy between operations
    };
}

#endif // ROLOCTYPES_HPP
