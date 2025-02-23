
mod loragw_sx1250;
mod cal_fw;
mod loragw_agc_params;
mod arb_firmware;
mod loragw_reg;
pub mod mcu;
pub mod helper;
pub mod loragw_sx1302_timestamp;
pub mod error;
mod loragw_sx1302;
mod agc_firmware;
mod loragw_usb_com;
pub mod loragw_com;

use agc_firmware::{AGC_FIRMWARE_SX1250, AGC_FIRMWARE_SX125X};
use anyhow::{anyhow,Result};
use loragw_com::LgwComType;
use loragw_reg::*;
use loragw_sx1250::LoragwSx1250Trait;
use loragw_sx1302::{LorgwSx1302Trait, Sx1302, IF_FSK_STD, IF_LORA_MULTI, IF_LORA_STD, IF_UNDEFINED, SX1302_AGC_RADIO_GAIN_AUTO};
use loragw_sx1302_timestamp::lora_packet_time_on_air;
use mcu::McuTrait;
use serde::{Deserialize, Serialize};
use tracing::{debug, error, info, trace, warn};
use error::Error;



/* radio-specific parameters */
pub const LGW_XTAL_FREQU:u32 =      32000000;            /* frequency of the RF reference oscillator */
pub const LGW_RF_CHAIN_NB:u8 =     2;                   /* number of RF chains */
//pub const LGW_RF_RX_BANDWIDTH {1000000, 1000000}  /* bandwidth of the radios */

/* concentrator chipset-specific parameters */
pub const LGW_IF_CHAIN_NB:u8 =     10;      /* number of IF+modem RX chains */
pub const LGW_REF_BW: u32  =        125000;  /* typical bandwidth of data channel */
pub const LGW_MULTI_NB :u8 =       8;       /* number of LoRa 'multi SF' chains */
#[allow(dead_code)]
pub const LGW_MULTI_SF_EN:u8 =     0xFF;    /* bitmask to enable/disable SF for multi-sf correlators  (12 11 10 9 8 7 6 5) */


#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Modulation{
    UNDEFINED   =    0,
    CW          =    0x08,
    LORA        =    0x10,
    FSK         =    0x20,
}

impl std::fmt::Display for Modulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Modulation::UNDEFINED => write!(f, "Undefined"),
            Modulation::CW => write!(f, "CW"),
            Modulation::LORA => write!(f, "LoRa"),
            Modulation::FSK => write!(f, "FSK"),
        }
    }
}


const LGW_RF_RX_FREQ_MIN:u32=          100_000_000;
const LGW_RF_RX_FREQ_MAX:u32=          1_000_000_000;

/* values available for the 'bandwidth' parameters (LoRa & FSK) */
/* NOTE: directly encode FSK RX bandwidth, do not change */


pub const    BW_UNDEFINED:u8=    0;
pub const    BW_500KHZ:u8=     0x06;
pub const    BW_250KHZ:u8=      0x05;
pub const    BW_125KHZ:u8=      0x04;


/* values available for the 'datarate' parameters */
/* NOTE: LoRa values used directly to code SF bitmask in 'multi' modem, do not change */


pub const    DR_UNDEFINED:u32=    0;
pub const    DR_LORA_SF5:u32=    5;
pub const   DR_LORA_SF6:u32=     6;
pub const    DR_LORA_SF7:u32=     7;
pub const    DR_LORA_SF8:u32=     8;
pub const    DR_LORA_SF9:u32=     9;
pub const    DR_LORA_SF10:u32=    10;
pub const    DR_LORA_SF11:u32=    11;
pub const    DR_LORA_SF12:u32=    12;
pub const   DR_FSK_MIN:u32=      500;
pub const    DR_FSK_MAX:u32=      250000;

/* values available for the 'coderate' parameters (LoRa only) */
/* NOTE: arbitrary values */
pub const CR_UNDEFINED:u8=     0;   /* CR0 exists but is not recommended, so consider it as invalid */
pub const CR_LORA_4_5 :u8=     0x01;
pub const CR_LORA_4_6 :u8=     0x02;
pub const CR_LORA_4_7 :u8=     0x03;
pub const CR_LORA_4_8 :u8=     0x04;


pub const STAT_UNDEFINED:u8 =  0x00;
pub const STAT_NO_CRC:u8 =     0x01;
pub const STAT_CRC_BAD:u8 =    0x11;
pub const STAT_CRC_OK:u8 =     0x10;

const FW_VERSION_AGC_SX1250:u8=   10; /* Expected version of AGC firmware for sx1250 based gateway */
                                   /* v10 is same as v6 with improved channel check time for LBT */
const FW_VERSION_AGC_SX125X:u8=   6;  /* Expected version of AGC firmware for sx1255/sx1257 based gateway */
const FW_VERSION_ARB:u8=          2;  /* Expected version of arbiter firmware */



/* status code for TX_STATUS */
/* NOTE: arbitrary values */
pub const TX_STATUS_UNKNOWN:u8 =  0;
pub const TX_OFF:u8 =              1;    /* TX modem disabled, it will ignore commands */
pub const TX_FREE:u8 =             2;    /* TX modem is free, ready to receive a command */
pub const TX_SCHEDULED:u8 =        3;    /* TX modem is loaded, ready to send the packet after an event and/or delay */
pub const TX_EMITTING:u8 =         4;    /* TX modem is emitting */

/* status code for RX_STATUS */
/* NOTE: arbitrary values */
pub const RX_STATUS_UNKNOWN:u8 =   0;
pub const RX_OFF:u8 =              1;    /* RX modem is disabled, it will ignore commands  */
pub const RX_ON:u8 =               2;    /* RX modem is receiving */
pub const RX_SUSPENDED:u8 =        3;    /* RX is suspended while a TX is ongoing */

/* values available for the 'tx_mode' parameter */
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
#[allow(non_camel_case_types, dead_code)]
pub enum tx_mode_t {
    IMMEDIATE  =      0,
    TIMESTAMPED =     1,
    ON_GPS =         2,
}


/* Listen-Before-Talk */
pub const LGW_LBT_CHANNEL_NB_MAX:usize= 16; /* Maximum number of LBT channels */


/* Spectral Scan */
pub const LGW_SPECTRAL_SCAN_RESULT_SIZE:usize= 33; /* The number of results returned by spectral scan function, to be used for memory allocation */


const LGW_RF_RX_BANDWIDTH_125KHZ:u32 =  1600000;     /* for 125KHz channels */
const LGW_RF_RX_BANDWIDTH_250KHZ:u32 =  1600000;     /* for 250KHz channels */
const LGW_RF_RX_BANDWIDTH_500KHZ:u32 =  1600000;     /* for 500KHz channels */

#[derive(Debug,Clone, Copy, Serialize, Deserialize)]
#[derive(PartialEq)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum LgwRadioType {
    #[serde(rename = "NONE")]
    LGW_RADIO_TYPE_NONE,
    #[serde(rename = "SX1255")]
    LGW_RADIO_TYPE_SX1255,
    #[serde(rename = "SX1257")]
    LGW_RADIO_TYPE_SX1257,
    #[serde(rename = "SX1272")]
    LGW_RADIO_TYPE_SX1272,
    #[serde(rename = "SX1276")]
    LGW_RADIO_TYPE_SX1276,
    #[serde(rename = "SX1250")]
    LGW_RADIO_TYPE_SX1250
}

impl std::fmt::Display for LgwRadioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LgwRadioType::LGW_RADIO_TYPE_NONE => write!(f, "None"),
            LgwRadioType::LGW_RADIO_TYPE_SX1255 => write!(f, "SX1255"),
            LgwRadioType::LGW_RADIO_TYPE_SX1257 => write!(f, "SX1257"),
            LgwRadioType::LGW_RADIO_TYPE_SX1272 => write!(f, "SX1272"),
            LgwRadioType::LGW_RADIO_TYPE_SX1276 => write!(f, "SX1276"),
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => write!(f, "SX1250"),
        }
    }
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LgwRssiTcomp {
    pub coeff_a:f32,
    pub coeff_b:f32,
    pub coeff_c:f32,
    pub coeff_d:f32,
    pub coeff_e:f32,
}

impl Default for LgwRssiTcomp {
    fn default() -> Self {
        Self {
            coeff_a: 0.0,
            coeff_b: 0.0,
            coeff_c: 20.41,
            coeff_d: 2162.5599999999999,
            coeff_e: 0.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LgwConfRxrf {
    pub enable: bool,             //* enable or disable that RF chain */
    #[serde(rename="freq")]
    pub freq_hz: u32,           //* center frequency of the radio in Hz */
    pub rssi_offset: f32,        //* Board-specific RSSI correction factor */
    pub rssi_tcomp: LgwRssiTcomp,        //* Board-specific RSSI temperature compensation coefficients */
    #[serde(rename = "type")]
    pub _type:LgwRadioType,              //* Radio type for that RF chain (SX1255, SX1257....) */
    pub tx_enable:bool, 
    #[serde(default)]         //* enable or disable TX on that RF chain */
    pub single_input_mode: bool,  //* Configure the radio in single or differential input mode (SX1250 only) */
    #[serde(default)]
    pub tx_gain_lut: Option<Vec<LgwTxGain>>
}

impl Default for LgwConfRxrf {
    fn default() -> Self {
        Self { 
            enable: false, 
            freq_hz: 0, 
            rssi_offset: 0.0, 
            rssi_tcomp: LgwRssiTcomp { coeff_a: 0.0, coeff_b: 0.0, coeff_c: 0.0, coeff_d: 0.0, coeff_e: 0.0 }, 
            _type: LgwRadioType::LGW_RADIO_TYPE_SX1250, 
            tx_enable: false, 
            single_input_mode: false,
            tx_gain_lut: None
        }
    }
}

/**
@struct LgwConfRxIf
@brief Configuration structure for an IF chain
*/

#[derive(Debug, Clone, Copy,Serialize, Deserialize)]
pub struct LgwConfRxIf {
    pub        enable:bool,         //* enable or disable that IF chain */
    #[serde(rename="radio")]
    pub        rf_chain:u8,       //* to which RF chain is that IF chain associated */
    #[serde(rename="if")]
    pub        freq_hz:i32,        //* center frequ of the IF chain, relative to RF chain frequency */

    #[serde(default, with = "bandwidth_serde")]
    pub        bandwidth:u8,      //* RX bandwidth, 0 for default */
    #[serde(default)]
    pub        datarate:u32,       //* RX datarate, 0 for default */
    #[serde(default)]
    pub        sync_word_size:u8, //* size of FSK sync word (number of bytes, 0 for default) */
    #[serde(default)]
    pub        sync_word:u64,      //* FSK sync word (ALIGN RIGHT, eg. 0xC194C1) */
    #[serde(default)]
    pub        implicit_hdr:bool,               //* LoRa Service implicit header */
    #[serde(default)]
    pub        implicit_payload_length:u8,    //* LoRa Service implicit header payload length (number of bytes, 0 for default) */
    #[serde(default)]
    pub        implicit_crc_en:bool,            //* LoRa Service implicit header CRC enable */
    #[serde(default)]
    pub        implicit_coderate:u8,          //* LoRa Service implicit header coding rate  */
}
impl std::fmt::Display for LgwConfRxIf {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "LgwConfRxIf {{ enable: {}, rf_chain: {}, freq_hz: {}, bandwidth: {:?}, datarate: {}, sync_word_size: {}, sync_word: {}, implicit_hdr: {}, implicit_payload_length: {}, implicit_crc_en: {}, implicit_coderate: {} }}",
            self.enable,
            self.rf_chain,
            self.freq_hz,
            self.bandwidth,
            self.datarate,
            self.sync_word_size,
            self.sync_word,
            self.implicit_hdr,
            self.implicit_payload_length,
            self.implicit_crc_en,
            self.implicit_coderate
        )
    }
}


mod bandwidth_serde {
    use std::fmt;

    use serde::{Deserialize, Deserializer, Serializer};
    use serde::de;

    use crate::hal::{BW_125KHZ, BW_250KHZ, BW_500KHZ, BW_UNDEFINED};
    
    pub fn serialize<S>(value: &u8, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let v:u32;
        match *value {
            BW_125KHZ => v = 125000,
            BW_250KHZ => v = 250000,
            BW_500KHZ => v = 500000,
            _ => v = 0
        }
        serializer.serialize_u32(v)
        
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<u8, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct BandwidthVisitor;

        impl<'de> de::Visitor<'de> for BandwidthVisitor {
            type Value = u8;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a u32 value of 500000, 250000, 125000, or null")
            }

            fn visit_u32<E>(self, value: u32) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                match value {
                    500000 => Ok(BW_500KHZ),
                    250000 => Ok(BW_250KHZ),
                    125000 => Ok(BW_125KHZ),
                    0 => Ok(BW_UNDEFINED), // Default value
                    _ => Err(de::Error::custom(format!("Invalid bandwidth value: {}", value))),
                }
            }

            fn visit_none<E>(self) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(0) // Default value when missing
            }

            fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
            where
                D: Deserializer<'de>,
            {
                self.visit_u32(Deserialize::deserialize(deserializer)?)
            }
        }

        deserializer.deserialize_option(BandwidthVisitor)
    }
}

impl Default for LgwConfRxIf {
    fn default() -> Self {
        Self { 
            enable: false, 
            rf_chain: 0, 
            freq_hz: 0, 
            bandwidth: BW_UNDEFINED, 
            datarate: 0, 
            sync_word_size: 0, 
            sync_word: 0, 
            implicit_hdr: false, 
            implicit_payload_length: 0, 
            implicit_crc_en: false, 
            implicit_coderate: 0 
        }
    }
}

pub const TX_GAIN_LUT_SIZE_MAX:usize = 16;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LgwTxGain {
    pub     rf_power: i8,   //* measured TX power at the board connector, in dBm */
    #[serde(default)]
    pub     dig_gain:u8,   //* (sx125x) 2 bits: control of the digital gain of SX1302 */
    #[serde(default)]
    pub     pa_gain:u8,    //* (sx125x) 2 bits: control of the external PA (SX1302 I/O)
                        //     (sx1250) 1 bits: enable/disable the external PA (SX1302 I/O) */
    #[serde(default)]
    pub     dac_gain:u8,   //* (sx125x) 2 bits: control of the radio DAC */
    #[serde(default = "default_mix_gain")]
    pub     mix_gain:u8,   //* (sx125x) 4 bits: control of the radio mixer */
    #[serde(default)]
    pub     offset_i:i8,    //* (sx125x) calibrated I offset */
    #[serde(default)]
    pub     offset_q:i8,    //* (sx125x) calibrated Q offset */
    #[serde(default)]
    pub     pwr_idx:u8    //* (sx1250) 6 bits: control the radio power index to be used for configuration */
}

fn default_mix_gain() -> u8 {
    8
}

impl Default for LgwTxGain {
    fn default() -> Self {
        Self { 
            rf_power: 0, 
            dig_gain: 0, 
            pa_gain: 0, 
            dac_gain: 0, 
            mix_gain: 8, 
            offset_i: 0, 
            offset_q: 0, 
            pwr_idx: 0 
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LgwTxGainLut {
    pub     lut: [LgwTxGain; TX_GAIN_LUT_SIZE_MAX],  //* Array of Tx gain struct */
    pub     size:usize                      //* Number of LUT indexes */
}

impl Default for LgwTxGainLut {
    fn default() -> Self {
        Self { 
            lut: [Default::default(); TX_GAIN_LUT_SIZE_MAX], 
            size: 0 
        }
    }
}
/**
@struct LgwConfigBoard
@brief Configuration structure for board specificities
*/
#[derive(Debug, Clone,Serialize, Deserialize)]
pub struct LgwConfigBoard {
    pub            lorawan_public: bool, //* Enable ONLY for *public* networks using the LoRa MAC protocol */
    pub            clksrc:u8,         //* Index of RF chain which provides clock to concentrator */
    pub            full_duplex:bool,    //* Indicates if the gateway operates in full duplex mode or not */
    pub            com_type:LgwComType,       //* The COMmunication interface (SPI/USB) to connect to the SX1302 */
    pub            com_path:String   //* Path to access the COM device to connect to the SX1302 */
}

impl Default for LgwConfigBoard {
    fn default() -> Self {
        Self { 
            lorawan_public: false, 
            clksrc: 0, 
            full_duplex: false, 
            com_type: LgwComType::LGW_COM_USB, 
            com_path: String::new() 
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LgwConfDemod {
    pub     multisf_datarate:u8   //* bitmask to enable spreading-factors for correlators (SF12 - SF5) */
}

impl Default for LgwConfDemod {
    fn default() -> Self {
        Self { multisf_datarate: 0 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum LgwFtimeMode {
    #[serde(rename = "high_capacity")]
    LGW_FTIME_MODE_HIGH_CAPACITY,   //* fine timestamps for SF5 -> SF10 */
    #[serde(rename = "all_sf")]
    LGW_FTIME_MODE_ALL_SF           //* fine timestamps for SF5 -> SF12 */
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LgwConfigFtime {
    pub enable: bool,              //* Enable / Disable fine timestamping */
    pub mode:LgwFtimeMode    //* Fine timestamping mode */
}

impl Default for LgwConfigFtime {
    fn default() -> Self {
        Self { enable: false, mode: LgwFtimeMode::LGW_FTIME_MODE_HIGH_CAPACITY }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum LgwLbtScanTime {
    LGW_LBT_SCAN_TIME_128_US    = 128,
    LGW_LBT_SCAN_TIME_5000_US   = 5000,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LgwConfigChanLbt{
    pub freq_hz:u32,          /* LBT channel frequency */
    pub bandwidth:u8,         /* LBT channel bandwidth */
    pub scan_time_us:LgwLbtScanTime,      /* LBT channel carrier sense time */
    pub transmit_time_ms:u16  /* LBT channel transmission duration when allowed */
}

impl Default for LgwConfigChanLbt {
    fn default() -> Self {
        Self { freq_hz: 0, bandwidth: 0, scan_time_us: LgwLbtScanTime::LGW_LBT_SCAN_TIME_128_US, transmit_time_ms: 0 }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LgwConfLbt {
    pub   enable:bool,            /* enable or disable LBT */
    pub   rssi_target:i8,        /* RSSI threshold to detect if channel is busy or not (dBm) */
    pub   nb_channel:u8,        /* number of LBT channels */
    pub   channels:[LgwConfigChanLbt;LGW_LBT_CHANNEL_NB_MAX as usize]  /* LBT channels configuration */
}

impl Default for LgwConfLbt {
    fn default() -> Self {
        Self { 
            enable: false, 
            rssi_target: 0, 
            nb_channel: 0, 
            channels: [Default::default(); LGW_LBT_CHANNEL_NB_MAX as usize] 
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LgwConfSx1261 {
    #[serde(default)]
    pub enable:  bool,           //* enable or disable SX1261 radio */
    pub spi_path: Option<String>,       //* Path to access the SPI device to connect to the SX1261 (not used for USB com type) */
    pub rssi_offset: f32,        //* value to be applied to the sx1261 RSSI value (dBm) */
    #[serde(default)]
    pub lbt_conf:LgwConfLbt           //* listen-before-talk configuration */
}

impl Default for LgwConfSx1261 {
    fn default() -> Self {
        Self { 
            enable: false, 
            spi_path: None, 
            rssi_offset: 0.0f32, 
            lbt_conf: Default::default() 
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ConfRefPayload {
    pub id:u32,
    pub payload:[u8;255],
    pub prev_cnt:u32
}

impl Default for ConfRefPayload {
    fn default() -> Self {
        Self { id: 0, payload: [0; 255], prev_cnt: 0 }
    }
}

#[derive(Debug, Clone)]
pub struct LgwConfDebug {
    pub nb_ref_payload:u8,
    pub ref_payload:[ConfRefPayload;16],
    pub log_file_name:String
}

impl Default for LgwConfDebug {
    fn default() -> Self {
        Self { 
            nb_ref_payload: 0, 
            ref_payload: [Default::default(); 16], 
            log_file_name: String::new() 
        }
    }
}


/**
@struct LgwPktRx
@brief Structure containing the metadata of a packet that was received and a pointer to the payload
*/
#[derive(Debug)]
pub struct LgwPktRx {
    pub freq_hz: u32,        /* central frequency of the IF chain */
    pub freq_offset: i32,
    pub if_chain: u8,       /* by which IF chain was packet received */
    pub status: u8,         /* status of the received packet */
    pub count_us: u32,       /* internal concentrator counter for timestamping, 1 microsecond resolution */
    pub rf_chain: u8,       /* through which RF chain the packet was received */
    pub modem_id: u8,
    pub modulation: Modulation,     /* modulation used by the packet */
    pub bandwidth: u8,      /* modulation bandwidth (LoRa only) */
    pub datarate: u32,       /* RX datarate of the packet (SF for LoRa) */
    pub coderate: u8,       /* error-correcting code of the packet (LoRa only) */
    pub rssic: f32,        /* average RSSI of the channel in dB */
    pub rssis: f32,        /* average RSSI of the signal in dB */
    pub snr: f32,          /* average packet SNR, in dB (LoRa only) */
    pub snr_min: f32,      /* minimum packet SNR, in dB (LoRa only) */
    pub snr_max: f32,      /* maximum packet SNR, in dB (LoRa only) */
    pub crc: u16,        /* CRC that was received in the payload */
    pub size: u16,           /* payload size in bytes */
    pub payload: [u8; 256],   /* buffer containing the payload */
    pub ftime_received: bool, /* a fine timestamp has been received */
    pub ftime: u32          /* packet fine timestamp (nanoseconds since last PPS) */
}
impl std::fmt::Display for LgwPktRx {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "LgwPktRx {{ freq_hz: {}, freq_offset: {}, if_chain: {}, status: {}, count_us: {}, rf_chain: {}, modem_id: {}, modulation: {}, bandwidth: {}, datarate: {}, coderate: {}, rssic: {}, rssis: {}, snr: {}, snr_min: {}, snr_max: {}, crc: {}, size: {}, payload: {:02X?}, ftime_received: {}, ftime: {} }}",
            self.freq_hz,
            self.freq_offset,
            self.if_chain,
            self.status,
            self.count_us,
            self.rf_chain,
            self.modem_id,
            self.modulation,
            self.bandwidth,
            self.datarate,
            self.coderate,
            self.rssic,
            self.rssis,
            self.snr,
            self.snr_min,
            self.snr_max,
            self.crc,
            self.size,
            &self.payload[..self.size as usize],
            self.ftime_received,
            self.ftime
        )
    }
}
impl LgwPktRx {
    pub fn new() -> Self {
        Self {
            freq_hz: 0,
            freq_offset: 0,
            if_chain: 0,
            status: 0,
            count_us: 0,
            rf_chain: 0,
            modem_id: 0,
            modulation: Modulation::UNDEFINED,
            bandwidth: 0,
            datarate: 0,
            coderate: 0,
            rssic: 0.0,
            rssis: 0.0,
            snr: 0.0,
            snr_min: 0.0,
            snr_max: 0.0,
            crc: 0,
            size: 0,
            payload: [0; 256],
            ftime_received: false,
            ftime: 0,
        }
    }
}

#[derive(Debug,Clone, Copy)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum TxMode {
    IMMEDIATE =      0,
    TIMESTAMPED =    1,
    ON_GPS  =        2
}


/**
@struct LgwPktTx
@brief Structure containing the configuration of a packet to send and a pointer to the payload
*/
#[derive(Debug,Clone, Copy)]
pub struct LgwPktTx {
    pub freq_hz: u32,        /* center frequency of TX */
    pub tx_mode: TxMode,        /* select on what event/time the TX is triggered */
    pub count_us: u32,       /* timestamp or delay in microseconds for TX trigger */
    pub rf_chain: u8,       /* through which RF chain will the packet be sent */
    pub rf_power: i8,       /* TX power, in dBm */
    pub modulation: Modulation,     /* modulation to use for the packet */
    pub freq_offset: i8,    /* frequency offset from Radio Tx frequency (CW mode) */
    pub bandwidth: u8,      /* modulation bandwidth (LoRa only) */
    pub datarate: u32,       /* TX datarate (baudrate for FSK, SF for LoRa) */
    pub coderate: u8,       /* error-correcting code of the packet (LoRa only) */
    pub invert_pol: bool,     /* invert signal polarity, for orthogonal downlinks (LoRa only) */
    pub f_dev: u8,          /* frequency deviation, in kHz (FSK only) */
    pub preamble: u16,       /* set the preamble length, 0 for default */
    pub no_crc: bool,         /* if true, do not send a CRC in the packet */
    pub no_header: bool,      /* if true, enable implicit header mode (LoRa), fixed length (FSK) */
    pub size: u16,           /* payload size in bytes */
    pub payload: [u8; 256],   /* buffer containing the payload */
}

impl Default for LgwPktTx {
    fn default() -> Self {
        Self {
            freq_hz: 0,
            tx_mode: TxMode::IMMEDIATE,
            count_us: 0,
            rf_chain: 0,
            rf_power: 0,
            modulation: Modulation::UNDEFINED,
            freq_offset: 0,
            bandwidth: BW_UNDEFINED,
            datarate: DR_UNDEFINED,
            coderate: CR_UNDEFINED,
            invert_pol: false,
            f_dev: 0,
            preamble: 0,
            no_crc: false,
            no_header: false,
            size: 0,
            payload: [0; 256],
        }
    }
}

#[derive(Debug, Clone)]
pub struct LgwContext {
    /* Global context */
    pub is_started: bool,          //* is the LoRa concentrator started ? */;
    pub board_cfg: LgwConfigBoard,  //* Basic system configuration */;
    /* RX context */
    pub       rf_chain_cfg:[LgwConfRxrf;LGW_RF_CHAIN_NB as usize],
    pub       if_chain_cfg:[LgwConfRxIf;LGW_IF_CHAIN_NB as usize],
    pub       demod_cfg:LgwConfDemod,  //* demodulation configuration */;
    pub       lora_service_cfg:LgwConfRxIf,                       /* LoRa service channel config parameters */
    pub       fsk_cfg:LgwConfRxIf,                                /* FSK channel config parameters */
    /* TX context */
    pub       tx_gain_lut:[LgwTxGainLut;LGW_RF_CHAIN_NB as usize],          /* TX gain tables */
    /* Misc */
    pub      ftime_cfg:LgwConfigFtime,                              /* Fine timestamp configuration */
    pub      sx1261_cfg:LgwConfSx1261,                             /* SX1261 configuration */
    /* Debug */
    pub      debug_cfg:LgwConfDebug,                              /* Debug configuration */
}

impl Default for LgwContext {
    fn default() -> Self {
        Self { 
            is_started: false, 
            board_cfg: Default::default(), 
            rf_chain_cfg: [
                LgwConfRxrf {
                    enable: true,
                    freq_hz: 923600000,
                    rssi_offset: -210.0,
                    rssi_tcomp: Default::default(),
                    _type: LgwRadioType::LGW_RADIO_TYPE_SX1250,
                    tx_enable: true,
                    single_input_mode: false,
                    ..Default::default()
                },
                LgwConfRxrf {
                    enable: true,
                    freq_hz: 924550000,
                    rssi_offset: -210.0,
                    rssi_tcomp: Default::default(),
                    _type: LgwRadioType::LGW_RADIO_TYPE_SX1250,
                    tx_enable: false,
                    single_input_mode: false,
                    ..Default::default()
                }
            ], 
            if_chain_cfg: Default::default(), 
            demod_cfg: Default::default(), 
            lora_service_cfg: Default::default(), 
            fsk_cfg: Default::default(), 
            tx_gain_lut: Default::default(), 
            ftime_cfg: Default::default(), 
            sx1261_cfg: Default::default(), 
            debug_cfg: Default::default() 
        }
    }
}


#[derive(Debug)]
pub struct Hal {
    pub mcu: mcu::Mcu,
    pub ctx:LgwContext,
    pub sx1302: Sx1302,
}

impl Hal {

    pub fn new() -> Self {
        Self {
            mcu: mcu::Mcu::new(),
            ctx: Default::default(),
            sx1302: Sx1302::new(),
        }
    }

}

pub trait LgwHal {
    fn lgw_receive(&mut self) -> Result<Vec<LgwPktRx>>;
    fn lgw_start(&mut self) -> Result<()>;
    fn lgw_board_setconf(&mut self, conf:&LgwConfigBoard) -> Result<()>;
    fn lgw_rxrf_setconf(&mut self,  rf_chain:u8,   conf:&LgwConfRxrf) -> Result<()>;
    fn lgw_rxif_setconf(&mut self,  if_chain:u8, conf:&LgwConfRxIf) ->Result<()>;
    fn lgw_demod_setconf(&mut self, conf: &LgwConfDemod);
    fn lgw_get_temperature(&mut self) -> Result<f32>;
    fn lgw_send(&mut self, pkt_data:&LgwPktTx) -> Result<()>;
    fn lgw_get_instcnt(&mut self) -> Result<u32>;
    fn lgw_txgain_setconf(&mut self,  rf_chain:u8, conf: &[LgwTxGain]) -> Result<()>;
    fn lgw_stop(&mut self) -> Result<()> ;
    fn lgw_abort_tx(&mut self, rf_chain: u8) -> Result<()>;
}

impl LgwHal for Hal {
    

    fn lgw_abort_tx(&mut self, rf_chain: u8) -> Result<()> {
       

        /* check input variables */
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: NOT A VALID RF_CHAIN NUMBER");
            return Err(Error::LGW_HAL_ERROR.into());
        }

        /* Abort current TX */
        self.sx1302_tx_abort(rf_chain)

    }

    fn lgw_stop(&mut self) -> Result<()> {
       
        if self.ctx.is_started == false {
            info!("Note: LoRa concentrator was not started...");
            return Ok(());
        }

        /* Abort current TX if needed */
        for i in 0 .. LGW_RF_CHAIN_NB {
            debug!("INFO: aborting TX on chain {}", i);
            if let Err(e) = self.lgw_abort_tx(i){
                warn!(e=?e, "WARNING: failed to get abort TX on chain {}", i);
            }
        }

        info!("INFO: Disconnecting");
        if let Err(e) = self.mcu.lgw_disconnect() {
            error!(e=?e, "ERROR: failed to disconnect concentrator");
            return Err(e)
        }

        self.ctx.is_started = false;


        return Ok(());
    }
    fn lgw_send(&mut self, pkt_data:&LgwPktTx) -> Result<()>{

        //let lbt_tx_allowed:bool;
        /* performances variables */
        let ctx = self.ctx.clone();

        debug!(" --- {:}\n", "IN");

        /* check if the concentrator is running */
        if self.ctx.is_started == false {
            error!("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
            return Err(Error::LGW_HAL_ERROR.into());
        }

        /* check input range (segfault prevention) */
        if pkt_data.rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
            return Err(Error::LGW_HAL_ERROR.into());
        }

        /* check input variables */
        if ctx.rf_chain_cfg[pkt_data.rf_chain as usize].tx_enable == false {
            error!("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
            return Err(Error::LGW_HAL_ERROR.into());
        }
        if ctx.rf_chain_cfg[pkt_data.rf_chain as usize].enable == false {
            error!("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
            return Err(Error::LGW_HAL_ERROR.into());
        }

        match pkt_data.modulation {
            Modulation::LORA => {
                if !Hal::is_lora_bw(pkt_data.bandwidth) {
                    error!("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
                if !Hal::is_lora_dr(pkt_data.datarate) {
                    error!("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
                if !Hal::is_lora_cr(pkt_data.coderate) {
                    error!("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
                if pkt_data.size > 255 {
                    error!("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
            },
            Modulation::FSK => {
                if (pkt_data.f_dev < 1) || (pkt_data.f_dev > 200) {
                    error!("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
                if !Hal::is_fsk_dr(pkt_data.datarate) {
                    error!("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
                if pkt_data.size > 255 {
                    error!("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
                    return Err(Error::LGW_HAL_ERROR.into());
                }
            },
            Modulation::CW => {
            /* do nothing */
            },
            _ => {
                error!("ERROR: INVALID TX MODULATION\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
        }
    
        /* Start Listen-Before-Talk 
        if (ctx.sx1261_cfg.lbt_conf.enable == true) {
            err = lgw_lbt_start(&CONTEXT_SX1261, pkt_data);
            if (err != 0) {
                error!("ERROR: failed to start LBT\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
        }
        */
        /* Send the TX request to the concentrator */
        let mut tx = pkt_data.clone();
        if let Err(err) =  self.sx1302_send(ctx.rf_chain_cfg[pkt_data.rf_chain as usize]._type, &ctx.tx_gain_lut[pkt_data.rf_chain as usize], ctx.board_cfg.lorawan_public, &ctx.fsk_cfg, &mut tx) {
   
            error!(e=%err, "ERROR: Failed to send packet\n");
            /* 
            if (CONTEXT_SX1261.lbt_conf.enable == true) {
                err = lgw_lbt_stop();
                if (err != 0) {
                    error!("ERROR: %s: Failed to stop LBT\n", __FUNCTION__);
                }
            }
            */
            return Err(Error::LGW_HAL_ERROR.into());
        }

       // _meas_time_stop(1, tm, __FUNCTION__);

        /* Stop Listen-Before-Talk 
        if (CONTEXT_SX1261.lbt_conf.enable == true) {
            err = lgw_lbt_tx_status(pkt_data.rf_chain, &lbt_tx_allowed);
            if (err != 0) {
                error!("ERROR: %s: Failed to get LBT TX status, TX aborted\n", __FUNCTION__);
                err = sx1302_tx_abort(pkt_data.rf_chain);
                if (err != 0) {
                    error!("ERROR: %s: Failed to abort TX\n", __FUNCTION__);
                }
                err = lgw_lbt_stop();
                if (err != 0) {
                    error!("ERROR: %s: Failed to stop LBT\n", __FUNCTION__);
                }
                return Err(Error::LGW_HAL_ERROR.into());
            }
            if (lbt_tx_allowed == true) {
                error!("LBT: packet is allowed to be transmitted\n");
            } else {
                error!("LBT: (ERROR) packet is NOT allowed to be transmitted\n");
            }

            err = lgw_lbt_stop();
            if (err != 0) {
                error!("ERROR: %s: Failed to stop LBT\n", __FUNCTION__);
                return Err(Error::LGW_HAL_ERROR.into());
            }
            
        }
        */
        debug!(" --- {}\n", "OUT");

        Ok(())
    }

    fn lgw_get_instcnt(&mut self) -> Result<u32> {
        self.sx1302_timestamp_counter(false)
    }

    fn lgw_receive(&mut self) -> Result<Vec<LgwPktRx>> {

        let mut pkts = Vec::<LgwPktRx>::new();
        let nb_pkg_fetched = self.sx1302_fetch();
        if nb_pkg_fetched.is_err() {
            error!("ERROR: failed to fetch packets from SX1302\n");
            return Err(anyhow!("LGW_HAL_ERROR"));
        }
        let nb_pkg_fetched = nb_pkg_fetched.unwrap();

        /* Update internal counter */
        /* WARNING: this needs to be called regularly by the upper layer */
        self.sx1302_update()?;

        let _temp = self.lgw_get_temperature()?;

        for _i in 0 .. nb_pkg_fetched {
            if let  Ok(mut pkt) = self.sx1302_parse(){
                pkt.rssic += self.ctx.rf_chain_cfg[pkt.rf_chain as usize].rssi_offset;
                pkt.rssis += self.ctx.rf_chain_cfg[pkt.rf_chain as usize].rssi_offset;
                debug!(pkt=%pkt);
                pkts.push(pkt);
            }
        }
        Ok(pkts)
    }

    fn lgw_start(&mut self) -> Result<()> {

        let ctx = self.ctx.clone() ;

        if ctx.is_started {
            return Ok(())
        }

        self.lgw_connect()?;

        self.sx1302_set_gpio(0)?;

        self.sx1302_radio_calibrate(&ctx.rf_chain_cfg, ctx.board_cfg.clksrc, &ctx.tx_gain_lut)?;


        /* Setup radios for RX */
        for i in 0 .. LGW_RF_CHAIN_NB {
            if self.ctx.rf_chain_cfg[i as usize].enable == true {
                /* Reset the radio */
                if let Err(e) = self.sx1302_radio_reset(i, self.ctx.rf_chain_cfg[i as usize]._type) {
                    error!("ERROR: failed to reset radio {}\n", i);
                    return Err(anyhow!("LGW_HAL_ERROR: {}", e));
                }

                
                /* Setup the radio */
                match self.ctx.rf_chain_cfg[i as usize]._type {
                    LgwRadioType:: LGW_RADIO_TYPE_SX1250 => {
                        if let Err(_) = self.sx1250_setup(i, self.ctx.rf_chain_cfg[i as usize].freq_hz, self.ctx.rf_chain_cfg[i as usize].single_input_mode){
                            error!("ERROR: failed to setup SX1250: {}", i);
                            return Err(anyhow!("LGW_HAL_ERR"));
                        }
                    }
                    /* 
                    case LGW_RADIO_TYPE_SX1255:
                    case LGW_RADIO_TYPE_SX1257:
                        if let Err(err) = sx125x_setup(i, CONTEXT_BOARD.clksrc, true, ctx.rf_chain_cfg[i].type, ctx.rf_chain_cfg[i].freq_hz);
                        break;
                    */
                    _ =>{
                        error!("ERROR: RADIO TYPE NOT SUPPORTED (RF_CHAIN {:})\n", i);
                        return Err(anyhow!("RADIO TYPE NOT SUPPORTED"));
                    }
                }
                
                

                /* Set radio mode */
                if let Err(_) = self.sx1302_radio_set_mode(i, self.ctx.rf_chain_cfg[i as usize]._type){
             
                    error!("ERROR: failed to set mode for radio {:}\n", i);
                    return Err(anyhow!("LGW_HAL_ERROR"))
                }
            }
        }

        /* Select the radio which provides the clock to the sx1302 */
        self.sx1302_radio_clock_select(self.ctx.board_cfg.clksrc).map_err(|e| anyhow!("LGW_HAL_ERR: failed to get clock from radio {}",e))?;
        

        /* Release host control on radio (will be controlled by AGC) */
        self.sx1302_radio_host_ctrl(false).map_err(|e| anyhow!("ERROR: failed to release control over radios {}",e))?;
    

        /* Basic initialization of the sx1302 */
        let ftime_cfg = self.ctx.ftime_cfg.clone();

        if let Err(err) = self.sx1302_init(&ftime_cfg){
            error!("ERROR: failed to initialize SX1302: {}", err);
            return Err(anyhow!("LGW_HAL_ERROR"));
        }
        
        /* Configure PA/LNA LUTs */
        let board_cfg = self.ctx.board_cfg.clone();
        if let Err(err) = self.sx1302_pa_lna_lut_configure(&board_cfg){
 
            error!("ERROR: failed to configure SX1302 PA/LNA LUT: {}", err);
            return Err(anyhow!("LGW_HAL_ERROR"));
        }


        /* Configure Radio FE */
        if let Err(_) = self.sx1302_radio_fe_configure() {
            error!("ERROR: failed to configure SX1302 radio frontend\n");
            return Err(anyhow!("LGW_HAL_ERROR"));
        }

        /* Configure the Channelizer */
        let if_chain_cfg = self.ctx.if_chain_cfg;
        if let Err(_) = self.sx1302_channelizer_configure(&if_chain_cfg, false){
            error!("ERROR: failed to configure SX1302 channelizer\n");
            return Err(anyhow!("LGW_HAL_ERROR"));
        }
        
        let demod_cfg = self.ctx.demod_cfg;
        /* configure LoRa 'multi-sf' modems */
        if let Err(_) = self.sx1302_lora_correlator_configure(&if_chain_cfg, &demod_cfg)
        {
            error!("ERROR: failed to configure SX1302 LoRa modem correlators\n");
            return Err(anyhow!("LGW_HAL_ERR"))
        }

        let rf_chain_cfg = &self.ctx.rf_chain_cfg;

        if let Err(_) = self.sx1302_lora_modem_configure(rf_chain_cfg[0].freq_hz)
        {
            error!("ERROR: failed to configure SX1302 LoRa modems\n");
            return Err(anyhow!("LGW_HAL_ERR"))
        }

        /* configure LoRa 'single-sf' modem */
        if if_chain_cfg[8].enable == true {
            let lora_service_cfg = self.ctx.lora_service_cfg;
            if let Err(e) = self.sx1302_lora_service_correlator_configure(&lora_service_cfg) {
                error!(e=%e, "ERROR: failed to configure SX1302 LoRa Service modem correlators\n");
                return Err(anyhow!("LGW_HAL_ERR"));
            }

            if let Err(e) =self.sx1302_lora_service_modem_configure(&lora_service_cfg, self.ctx.rf_chain_cfg[0].freq_hz){
                error!(e=%e,"ERROR: failed to configure SX1302 LoRa Service modem\n");
                return Err(anyhow!("LGW_HAL_ERR"));
            }
        }

        /* configure FSK modem */
        if if_chain_cfg[9].enable == true {
            let fsk_cfg = self.ctx.fsk_cfg;

            if let Err(_) =self.sx1302_fsk_configure(&fsk_cfg){
                error!("ERROR: failed to configure SX1302 FSK modem\n");
                return Err(anyhow!("LGW_HAL_ERR"));
            }
        }

        /* configure syncword */
        if let Err(_) =self.sx1302_lora_syncword(self.ctx.board_cfg.lorawan_public, self.ctx.lora_service_cfg.datarate as u8){
            error!("ERROR: failed to configure SX1302 LoRa syncword\n");
            return Err(anyhow!("LGW_HAL_ERR"));
        }

        /* enable demodulators - to be done before starting AGC/ARB */
        if let Err(_) =self.sx1302_modem_enable(){
            error!("ERROR: failed to enable SX1302 modems\n");
            return Err(anyhow!("LGW_HAL_ERR"));
        }
        
        let fw_version_agc:u8;

         /* Load AGC firmware */
         match self.ctx.rf_chain_cfg[self.ctx.board_cfg.clksrc as usize]._type {
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => {
                debug!("Loading AGC fw for sx1250\n");
                if let Err(e) = self.sx1302_agc_load_firmware(&AGC_FIRMWARE_SX1250){
                    error!("ERROR: failed to load AGC firmware for sx1250: {}\n", e);
                    return Err(anyhow!("LGW_HAL_ERROR"));
                }
                fw_version_agc = FW_VERSION_AGC_SX1250;
            }
            LgwRadioType:: LGW_RADIO_TYPE_SX1255 |
            LgwRadioType:: LGW_RADIO_TYPE_SX1257  => 
            {
                debug!("Loading AGC fw for sx125x\n");
                if let Err(err) = self.sx1302_agc_load_firmware(&AGC_FIRMWARE_SX125X){
                    error!("ERROR: failed to load AGC firmware for sx125x {}\n", err);
                    return Err(anyhow!("LGW_HAL_ERROR"));
                }
                fw_version_agc = FW_VERSION_AGC_SX125X;
                
            }
                
            _ =>{
                error!("ERROR: failed to load AGC firmware, radio type not supported ({:})\n", self.ctx.rf_chain_cfg[self.ctx.board_cfg.clksrc as usize]._type);
                return Err(anyhow!("LGW_HAL_ERROR"));
            }
        }

        if let Err(err) = self.sx1302_agc_start(fw_version_agc, self.ctx.rf_chain_cfg[self.ctx.board_cfg.clksrc as usize]._type, SX1302_AGC_RADIO_GAIN_AUTO, SX1302_AGC_RADIO_GAIN_AUTO, self.ctx.board_cfg.full_duplex, self.ctx.sx1261_cfg.lbt_conf.enable){
            error!("ERROR: failed to start AGC firmware: {}", err);
            return Err(anyhow!("LGW_HAL_ERROR"));
        }

        /* Load ARB firmware */
        info!("Loading ARB fw\n");
        if let Err(err) = self.sx1302_arb_load_firmware(&arb_firmware::ARB_FIRMWARE)
        {
            error!("ERROR: failed to load ARB firmware: {}", err);
            return Err(anyhow!("LGW_HAL_ERROR"));
        }
        
        let ftime_cfg = self.ctx.ftime_cfg;
        if let Err(_) = self.sx1302_arb_start(FW_VERSION_ARB, &ftime_cfg){
            error!("ERROR: failed to start ARB firmware\n");
            return Err(anyhow!("LGW_HAL_ERROR"));
        }

        
        if let Err(_) = self.sx1302_tx_configure(self.ctx.rf_chain_cfg[self.ctx.board_cfg.clksrc as usize]._type){

            error!("ERROR: failed to configure SX1302 TX path\n");
            return Err(anyhow!("LGW_HAL_ERROR"));
        }


        if let Err(err) = self.sx1302_gps_enable(true){

            error!("ERROR: failed to enable GPS on sx1302: {}", err);
            return Err(anyhow!("LGW_HAL_ERROR"));
        }
        

        /* Set CONFIG_DONE GPIO to 1 (turn on the corresponding LED) */
        if let Err(_) = self.sx1302_set_gpio(0x01){
            
            error!("ERROR: failed to set CONFIG_DONE GPIO\n");
            return Err(anyhow!("LGW_HAL_ERROR"));   
        }

        self.ctx.is_started = true;


        info!("lgw_start done");
        return Ok(())
    }



    fn lgw_board_setconf(&mut self, conf:&LgwConfigBoard) -> Result<()> {

        if self.ctx.is_started {
            error!("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION");
            return Err(anyhow!("CONCENTRATOR IS RUNNING"));
        }

        /* Check input parameters */
        if (conf.com_type != LgwComType::LGW_COM_SPI) && (conf.com_type != LgwComType::LGW_COM_USB) {
            error!("ERROR: WRONG COM TYPE\n");
            return Err(anyhow!("ERROR: WRONG COM TYPE"))
        }

        /* set internal config according to parameters */
        self.ctx.board_cfg.lorawan_public = conf.lorawan_public;
        self.ctx.board_cfg.clksrc = conf.clksrc;
        self.ctx.board_cfg.full_duplex = conf.full_duplex;
        self.ctx.board_cfg.com_type = conf.com_type;
        self.ctx.board_cfg.com_path = conf.com_path.to_owned();
        

       Ok(())
    }

    fn lgw_rxrf_setconf(&mut self,  rf_chain:u8,   conf:&LgwConfRxrf) -> Result<()>{

        if self.ctx.is_started {
            error!("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION");
            return Err(anyhow!("CONCENTRATOR IS RUNNING"));
        }

        if conf.enable == false {
            /* nothing to do */
            debug!("Note: rf_chain {:} disabled\n", rf_chain);
            return Ok(());
        }

        /* check input range (segfault prevention) */
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
            return Err(anyhow!("NOT A VALID RF_CHAIN NUMBER"));
        }

        /* check if radio type is supported */
        if (conf._type != LgwRadioType::LGW_RADIO_TYPE_SX1255) && (conf._type != LgwRadioType::LGW_RADIO_TYPE_SX1257) && (conf._type != LgwRadioType::LGW_RADIO_TYPE_SX1250) {
            error!("ERROR: NOT A VALID RADIO TYPE {:}", conf._type);
            return Err(anyhow!("NOT A VALID RADIO TYPE"));
        }

        /* check if the radio central frequency is valid */
        if (conf.freq_hz < LGW_RF_RX_FREQ_MIN) || (conf.freq_hz > LGW_RF_RX_FREQ_MAX) {
            error!("ERROR: NOT A VALID RADIO CENTER FREQUENCY, PLEASE CHECK IF IT HAS BEEN GIVEN IN HZ ({:})\n", conf.freq_hz);
            return Err(anyhow!("NOT A VALID RADIO CENTER FREQUENCY"));
        }

        let ctx_rf_chain = &mut self.ctx.rf_chain_cfg[rf_chain as usize];

        /* set internal config according to parameters */
        ctx_rf_chain.enable = conf.enable;
        ctx_rf_chain.freq_hz = conf.freq_hz;
        ctx_rf_chain.rssi_offset = conf.rssi_offset;
        ctx_rf_chain.rssi_tcomp.coeff_a = conf.rssi_tcomp.coeff_a;
        ctx_rf_chain.rssi_tcomp.coeff_b = conf.rssi_tcomp.coeff_b;
        ctx_rf_chain.rssi_tcomp.coeff_c = conf.rssi_tcomp.coeff_c;
        ctx_rf_chain.rssi_tcomp.coeff_d = conf.rssi_tcomp.coeff_d;
        ctx_rf_chain.rssi_tcomp.coeff_e = conf.rssi_tcomp.coeff_e;
        ctx_rf_chain._type = conf._type;
        ctx_rf_chain.tx_enable = conf.tx_enable;
        ctx_rf_chain.single_input_mode = conf.single_input_mode;

        debug!("Note: rf_chain {:} configuration; en:{:} freq:{:} rssi_offset:{:} radio_type:{:} tx_enable:{:} single_input_mode:{:}\n",  rf_chain,
                                                                                                                    ctx_rf_chain.enable,
                                                                                                                    ctx_rf_chain.freq_hz,
                                                                                                                    ctx_rf_chain.rssi_offset,
                                                                                                                    ctx_rf_chain._type,
                                                                                                                    ctx_rf_chain.tx_enable,
                                                                                                                    ctx_rf_chain.single_input_mode);

        Ok(())
    }

    fn lgw_txgain_setconf(&mut self,  rf_chain:u8, conf: &[LgwTxGain]) -> Result<()> {

        let ctx = &mut self.ctx.tx_gain_lut;
        /* Check LUT size */
        if (conf.len() < 1) || (conf.len() > TX_GAIN_LUT_SIZE_MAX) {
            error!("ERROR: TX gain LUT must have at least one entry and  maximum {} entries\n", TX_GAIN_LUT_SIZE_MAX);
            return Err(Error::LGW_HAL_ERROR.into());
        }
    
        ctx[rf_chain as usize].size = conf.len();
    
        for  (i, lut) in conf.iter().enumerate() {
            /* Check gain range */
            if lut.dig_gain > 3 {
                error!("ERROR: TX gain LUT: SX1302 digital gain must be between 0 and 3\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
            if lut.dac_gain > 3 {
                error!("ERROR: TX gain LUT: SX1257 DAC gains must not exceed 3\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
            if (lut.mix_gain < 5) || (lut.mix_gain > 15) {
                error!("ERROR: TX gain LUT: SX1257 mixer gain must be betwen [5..15]\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
            if lut.pa_gain > 3 {
                error!("ERROR: TX gain LUT: External PA gain must not exceed 3\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
            if lut.pwr_idx > 22 {
                error!("ERROR: TX gain LUT: SX1250 power index must not exceed 22\n");
                return Err(Error::LGW_HAL_ERROR.into());
            }
    
            /* Set internal LUT */
            ctx[rf_chain as usize].lut[i].rf_power = lut.rf_power;
            ctx[rf_chain as usize].lut[i].dig_gain = lut.dig_gain;
            ctx[rf_chain as usize].lut[i].pa_gain  = lut.pa_gain;
            /* sx125x */
            ctx[rf_chain as usize].lut[i].dac_gain = lut.dac_gain;
            ctx[rf_chain as usize].lut[i].mix_gain = lut.mix_gain;
            ctx[rf_chain as usize].lut[i].offset_i = 0; /* To be calibrated */
            ctx[rf_chain as usize].lut[i].offset_q = 0; /* To be calibrated */
    
            /* sx1250 */
            ctx[rf_chain as usize].lut[i].pwr_idx = lut.pwr_idx;
        }
    
        Ok(())
    }

    fn lgw_rxif_setconf(&mut self,  if_chain:u8, conf:&LgwConfRxIf) ->Result<()> {
        let  bw_hz: i32;
        let  rf_rx_bandwidth:u32;


        /* check if the concentrator is running */
        if self.ctx.is_started {
            error!("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION");
            return Err(anyhow!("CONCENTRATOR IS RUNNING"));
        }

        /* check input range (segfault prevention) */
        if if_chain >= LGW_IF_CHAIN_NB {
            error!("ERROR: {:} NOT A VALID IF_CHAIN NUMBER\n", if_chain);
            return Err(anyhow!("Invalid IF_CHAIN"));
        }

        let ctx_if_chain = &mut self.ctx.if_chain_cfg[if_chain as usize];

        /* if chain is disabled, don't care about most parameters */
        if conf.enable == false {
            ctx_if_chain.enable = false;
            ctx_if_chain.freq_hz = 0;
            debug!("Note: if_chain {:} disabled\n", if_chain);
            return Ok(())
        }

        /* check 'general' parameters */
        if <Hal as LorgwSx1302Trait>::sx1302_get_ifmod_config(if_chain) == IF_UNDEFINED as u8 {
            error!("ERROR: IF CHAIN {:} NOT CONFIGURABLE\n", if_chain);
            return Err(anyhow!("IF CHAIN NOT CONFIGURABLE"))
        }

        if conf.rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN\n");
            return Err(anyhow!("INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN"))
        }

        /* check if IF frequency is optimal based on channel and radio bandwidths */
        match conf.bandwidth {
            BW_250KHZ => {
                rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_250KHZ; /* radio bandwidth */
            }

            BW_500KHZ => {
                rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_500KHZ; /* radio bandwidth */
            }
            _ => {
                rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_125KHZ; /* radio bandwidth */
            }
        }

        bw_hz = Hal::lgw_bw_getval(conf.bandwidth); /* channel bandwidth */

        if (conf.freq_hz + if bw_hz == -1 { LGW_REF_BW as i32 } else { bw_hz } / 2) > (rf_rx_bandwidth as i32 / 2) {
            error!("ERROR: IF FREQUENCY {:} TOO HIGH\n", conf.freq_hz);
            return Err(anyhow!("IF FREQUENCY TOO HIGH"));
        } else if (conf.freq_hz - if bw_hz == -1 { LGW_REF_BW as i32 } else { bw_hz } / 2) < -(rf_rx_bandwidth as i32 / 2) {
            error!("ERROR: IF FREQUENCY {:} TOO LOW\n", conf.freq_hz);
            return Err(anyhow!("IF FREQUENCY TOO LOW"));
        }

        /* check parameters according to the type of IF chain + modem,
        fill default if necessary, and commit configuration if everything is OK */
        match <Hal as LorgwSx1302Trait>::sx1302_get_ifmod_config(if_chain) {
            IF_LORA_STD => {
                let mut bandwidth = conf.bandwidth;
                let mut datarate = conf.datarate;
                debug!(conf=%conf);
                /* fill default parameters if needed */
                if bandwidth == BW_UNDEFINED {
                    bandwidth = BW_250KHZ;
                }
                if datarate == DR_UNDEFINED as u32 {
                    datarate = DR_LORA_SF7 as u32;
                }

                /* check BW & DR */
                if !Hal::is_lora_bw(bandwidth) {
                    error!("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN");
                    return Err(anyhow!("BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN"));
                }

                if !Hal::is_lora_dr(datarate) {
                    error!("ERROR: DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN: {:}\n", conf.datarate);
                    return Err(anyhow!("DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN"));
                }

                /* set internal configuration  */
                ctx_if_chain.enable = conf.enable;
                ctx_if_chain.rf_chain = conf.rf_chain;
                ctx_if_chain.freq_hz = conf.freq_hz;
                let mut ctx_lora_service = self.ctx.lora_service_cfg;

                ctx_lora_service.bandwidth = bandwidth;
                ctx_lora_service.datarate = datarate;
                ctx_lora_service.implicit_hdr = conf.implicit_hdr;
                ctx_lora_service.implicit_payload_length = conf.implicit_payload_length;
                ctx_lora_service.implicit_crc_en   = conf.implicit_crc_en;
                ctx_lora_service.implicit_coderate = conf.implicit_coderate;

                debug!("Note: LoRa 'std' if_chain {:} configuration; en:{:} freq:{:} bw:{:?} dr:{:}\n", if_chain,
                    ctx_if_chain.enable,
                    ctx_if_chain.freq_hz,
                    ctx_lora_service.bandwidth,
                    ctx_lora_service.datarate);
            }

            IF_LORA_MULTI => {
                let mut bandwidth = conf.bandwidth;
                let mut datarate = conf.datarate;
                /* fill default parameters if needed */
                if bandwidth == BW_UNDEFINED {
                    bandwidth = BW_125KHZ;
                }
                if datarate ==  DR_UNDEFINED {
                    datarate = DR_LORA_SF7;
                }
                /* check BW & DR */
                if bandwidth != BW_125KHZ {
                    error!("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                    return Err(anyhow!("BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN"))
                }

                if !Hal::is_lora_dr(datarate) {
                    error!("ERROR: DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                    return Err(anyhow!("DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN"));
                }

                /* set internal configuration  */
                ctx_if_chain.enable = conf.enable;
                ctx_if_chain.rf_chain = conf.rf_chain;
                ctx_if_chain.freq_hz = conf.freq_hz;

                debug!("Note: LoRa 'multi' if_chain {:} configuration; en:{:} freq:{:}\n",   if_chain,
                    ctx_if_chain.enable,
                    ctx_if_chain.freq_hz);
            }

            IF_FSK_STD =>{
                let mut bandwidth = conf.bandwidth;
                let mut datarate = conf.datarate;
                /* fill default parameters if needed */
                if bandwidth == BW_UNDEFINED {
                    bandwidth = BW_250KHZ;
                }
                if datarate == DR_UNDEFINED as u32 {
                    datarate = 64000; /* default datarate */
                }

                /* check BW & DR */
                if !Hal::is_fsk_bw(bandwidth as u8) {
                    error!("ERROR: BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN\n");
                    return Err(anyhow!("BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN"));
                }

                if !Hal::is_fsk_dr(datarate) {
                    error!("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
                    return Err(anyhow!("DATARATE NOT SUPPORTED BY FSK IF CHAIN"));
                }
                let mut ctx_fsk = self.ctx.fsk_cfg;
                /* set internal configuration  */
                ctx_if_chain.enable = conf.enable;
                ctx_if_chain.rf_chain = conf.rf_chain;
                ctx_if_chain.freq_hz = conf.freq_hz;
                ctx_fsk.bandwidth = bandwidth;
                ctx_fsk.datarate = datarate;

                if conf.sync_word > 0 {
                    ctx_fsk.sync_word_size = conf.sync_word_size;
                    ctx_fsk.sync_word = conf.sync_word;
                }
                debug!("Note: FSK if_chain {:} configuration; en:{:} freq:{:} bw:{:} dr:{:} ({:} real dr) sync:0x{:02x?}: {:?}", 
                    if_chain,
                    ctx_if_chain.enable,
                    ctx_if_chain.freq_hz,
                    ctx_fsk.bandwidth,
                    ctx_fsk.datarate,
                    LGW_XTAL_FREQU/(LGW_XTAL_FREQU/ctx_fsk.datarate),
                    2*ctx_fsk.sync_word_size,
                    ctx_fsk.sync_word);
                
            }
            _ =>{
                error!("ERROR: IF CHAIN {:} TYPE NOT SUPPORTED\n", if_chain);
                return Err(anyhow!("IF CHAIN TYPE NOT SUPPORTED"));
            }
                
        }

        Ok(())
    }


    fn lgw_demod_setconf(&mut self, conf: &LgwConfDemod) {
        self.ctx.demod_cfg.multisf_datarate = conf.multisf_datarate
    }

    fn lgw_get_temperature(&mut self) -> Result<f32> {
        
        let status = self.mcu.get_mcu_status()?;
        Ok(status.temperature)
 
    }
}


impl Hal {
    pub fn lgw_bw_getval( bw: u8)->i32 {
        match bw {
            BW_500KHZ => { return 500000 }
            BW_250KHZ => { return 250000 }
            BW_125KHZ => { return 125000 }
            _ =>  { return -1 }
        }
    }


    fn is_lora_bw(bw: u8) -> bool {
        match bw {
            BW_125KHZ | BW_250KHZ | BW_500KHZ => {
                true
            }
            _ => { false }
        }
    }

    fn is_lora_dr(dr: u32) -> bool {
        match dr {
            DR_LORA_SF5 |
            DR_LORA_SF6  |
            DR_LORA_SF7 |
            DR_LORA_SF8  |
            DR_LORA_SF9 |
            DR_LORA_SF10  |
            DR_LORA_SF11 |
            DR_LORA_SF12  => true,
            _=>  false 
        } 
    }

    fn is_lora_cr(cr: u8) -> bool {
        cr == CR_LORA_4_5 || cr == CR_LORA_4_6 || cr == CR_LORA_4_7 || cr == CR_LORA_4_8
    }

    fn is_fsk_bw(bw: u8) -> bool {
        bw >= 1 && bw <= 7
    }

    fn is_fsk_dr(dr: u32) -> bool {
        dr >= DR_FSK_MIN && dr <= DR_FSK_MAX
    }

}

pub fn lgw_time_on_air(packet: &LgwPktTx, sync_word_size: u32) -> u32 {
    let t_fsk:f64;
    let mut toa_ms:u32 = 0;
    //let mut toa_us:u32 = 0;


    if packet.modulation == Modulation::LORA {
        if let Ok((toa_us, _, _, _)) = lora_packet_time_on_air(packet.bandwidth as u8, packet.datarate as u8, packet.coderate, packet.preamble, packet.no_header, packet.no_crc, packet.size as u8){
            toa_ms = ( toa_us as f64 / 1000.0 + 0.5 ) as u32;
            trace!("INFO: LoRa packet ToA: {:} ms\n", toa_ms);
        }
        
    } else if packet.modulation == Modulation::FSK {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        t_fsk = (8 as f64 * (packet.preamble as u32 + sync_word_size as u32 + 1 + packet.size as u32 + (if packet.no_crc == true { 0 } else { 2 } )) as f64 / packet.datarate as f64 ) * 1E3;

        /* Duration of packet */
        toa_ms = t_fsk as u32 + 1; /* add margin for rounding */
    } else {
        toa_ms = 0;
        error!("ERROR: Cannot compute time on air for this packet, unsupported modulation (0x{})\n", packet.modulation);
    }

    return toa_ms;
}