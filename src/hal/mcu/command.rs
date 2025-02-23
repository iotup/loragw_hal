use std::fmt::Display;

#[repr(u8)]
pub enum OrderId {
    ReqPing            = 0x00,
    ReqGetStatus       = 0x01,
    ReqBootloaderMode  = 0x02,
    ReqReset           = 0x03,
    ReqWriteGpio       = 0x04,
    ReqMultipleSpi     = 0x05,

    AckPing            = 0x40,
    AckGetStatus       = 0x41,
    AckBootloaderMode  = 0x42,
    AckReset           = 0x43,
    AckWriteGpio       = 0x44,
    AckMultipleSpi     = 0x45,

    CmdError = 0xFF,
}


#[repr(u8)]
pub enum CommandOrderOffset {
    Id,
    SizeMsb,
    SizeLsb,
    Cmd,
    Data
}

#[repr(u8)]
pub enum ECmdOffsetAckPing {
    AckPingUniqueId0,  AckPingUniqueId1,  AckPingUniqueId2,  AckPingUniqueId3,
    AckPingUniqueId4,  AckPingUniqueId5,  AckPingUniqueId6,  AckPingUniqueId7,
    AckPingUniqueId8,  AckPingUniqueId9,  AckPingUniqueId10, AckPingUniqueId11,
    AckPingVersion0,   AckPingVersion1,   AckPingVersion2,   AckPingVersion3,   AckPingVersion4,
    AckPingVersion5,   AckPingVersion6,   AckPingVersion7,   AckPingVersion8,
    AckPingSize,
}



#[repr(u8)]
pub enum ECmdOffsetAckGetStatus {
    AckGetStatusSystemTime31_24, AckGetStatusSystemTime23_16, AckGetStatusSystemTime15_8, AckGetStatusSystemTime7_0,
    AckGetStatusTemperature15_8, AckGetStatusTemperature7_0,
    AckGetStatusSize,
}

#[repr(u8)]
pub enum ECmdOffsetReqWriteGpio {
    ReqWriteGpioPort,
    ReqWriteGpioPin,
    ReqWriteGpioState,
    ReqWriteGpioSize,
}

#[repr(u8)]
pub enum ECmdOffsetAckGpioWrite {
    AckGpioWriteStatus,
    AckGpioWriteSize,
}



pub const  MCU_SPI_REQ_TYPE_READ_WRITE :u8        = 0x01; /* Read/Write SPI request */
pub const  MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE:u8   = 0x02; /* Read-Modify-Write SPI request */



#[repr(u8)]
pub enum ESpiStatus {
    SpiStatusOk,
    SpiStatusFail,
    SpiStatusWrongParam,
    SpiStatusTimeout,
}

#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum ECmdSpiTarget
{
    MCU_SPI_TARGET_SX1302,  /* SX1302 + SX1250 */
    MCU_SPI_TARGET_SX1261   /* LBT/Spectral Scan additional radio */
}

impl Display for ESpiStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ESpiStatus::SpiStatusOk => write!(f, "SpiStatusOk"),
            ESpiStatus::SpiStatusFail => write!(f, "SpiStatusFail"),
            ESpiStatus::SpiStatusWrongParam => write!(f, "SpiStatusWrongParam"),
            ESpiStatus::SpiStatusTimeout => write!(f, "SpiStatusTimeout"),
        }
    }
}


#[derive(Debug,PartialEq)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum EComWriteMode {
    LGW_COM_WRITE_MODE_SINGLE,
    LGW_COM_WRITE_MODE_BULK,
    LGW_COM_WRITE_MODE_UNKNOWN
}
