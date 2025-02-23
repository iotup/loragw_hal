#![allow(unused_macros)]
#![allow(non_snake_case)]

use anyhow::{anyhow,Result};
use tracing::{debug, error, info, trace, warn};
use crate::hal::{ cal_fw::CAL_FIRMWARE_SX125X, LgwFtimeMode, loragw_agc_params::{AGC_PARAMS_SX1250, AGC_PARAMS_SX125X}, loragw_sx1250::LoragwSx1250Trait, loragw_sx1302_timestamp::timestamp_counter_correction, mcu::McuTrait, Modulation, TxMode, BW_125KHZ, BW_250KHZ, BW_500KHZ, BW_UNDEFINED, CR_LORA_4_5, CR_LORA_4_6, CR_LORA_4_7, CR_LORA_4_8, CR_UNDEFINED, DR_UNDEFINED, LGW_MULTI_NB, STAT_CRC_BAD, STAT_CRC_OK, STAT_NO_CRC, STAT_UNDEFINED};
use super::{LgwConfigBoard, LgwConfDemod, LgwConfigFtime, LgwConfRxIf, LgwConfRxrf, LgwPktRx, LgwPktTx, LgwRadioType, LgwTxGainLut, loragw_sx1302_timestamp::{TimestampCounter, TimestampPpsHistory, SX1302TimestampTrait, MAX_TIMESTAMP_PPS_HISTORY}, mcu::command::EComWriteMode, Hal, LoragwRegTrait, DR_LORA_SF10, DR_LORA_SF11, DR_LORA_SF12, DR_LORA_SF5, DR_LORA_SF6, DR_LORA_SF7, DR_LORA_SF8, DR_LORA_SF9, LGW_IF_CHAIN_NB, LGW_RF_CHAIN_NB, RX_STATUS_UNKNOWN, TX_EMITTING, TX_FREE, TX_SCHEDULED, TX_STATUS_UNKNOWN};
use crate::hal::helper::wait_ms;
use super::error::Error;
use libm::{ceil, fabs};
use crate::hal::loragw_reg::*;

/* Default values */
pub const SX1302_AGC_RADIO_GAIN_AUTO:u8=  0xFF;
pub const TX_START_DELAY_DEFAULT:u32=      1500;    /* Calibrated value for 500KHz BW */

/* type of if_chain + modem */
pub const IF_UNDEFINED:u8=                0;
pub const IF_LORA_STD:u8=                 0x10;    /* if + standard single-SF LoRa modem */
pub const IF_LORA_MULTI:u8=               0x11;    /* if + LoRa receiver with multi-SF capability */
pub const IF_FSK_STD:u8=                  0x20;    /* if + standard FSK modem */

const AGC_RADIO_A_INIT_DONE:u8 =   0x80;
const AGC_RADIO_B_INIT_DONE:u8 =   0x20;

const MCU_AGC :u8 =                0x01;
const MCU_ARB :u8 =                0x02;

const AGC_MEM_ADDR:u16 =           0x0000;
const ARB_MEM_ADDR:u16  =          0x2000;

const MCU_FW_SIZE:usize =             8192; /* size of the firmware IN BYTES (= twice the number of 14b words) */

const FW_VERSION_CAL:u8 =          1;/* Expected version of calibration firmware */

const RSSI_FSK_POLY_0:f32=         90.636423; /* polynomiam coefficients to linearize FSK RSSI */
const RSSI_FSK_POLY_1:f32=          0.420835;
const RSSI_FSK_POLY_2:f32=          0.007129;
const RSSI_FSK_POLY_3:f32=          -0.000026;

const FREQ_OFFSET_LSB_125KHZ:f32=   0.11920929;     /* 125000 * 32 / 2^6 / 2^19 */
const FREQ_OFFSET_LSB_250KHZ:f32=   0.238418579;    /* 250000 * 32 / 2^6 / 2^19 */
const FREQ_OFFSET_LSB_500KHZ:f32=   0.476837158;    /* 500000 * 32 / 2^6 / 2^19 */


/* constant arrays defining hardware capability */
pub const IFMOD_CONFIG:[u8;LGW_IF_CHAIN_NB as usize] = [
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_MULTI, 
    IF_LORA_STD, 
    IF_FSK_STD
];

const MIN_LORA_PREAMBLE:u8=   6;
const STD_LORA_PREAMBLE:u8=    8;
const MIN_FSK_PREAMBLE:u8=     3;
const STD_FSK_PREAMBLE :u8=    5;

const GPIO_CFG_REGISTER:u8=                0x00;
const GPIO_CFG_AGC:u8=                     0x01;
const GPIO_CFG_ARB:u8=                     0x02;
const GPIO_CFG_SPI_EXP_1:u8=               0x03;
const GPIO_CFG_CSN_SPI_EXP:u8=             0x04;
const GPIO_CFG_SPI_EXP_2:u8=               0x05;
const GPIO_CFG_UART:u8=                    0x06;
const GPIO_CFG_SX1255_IQ:u8=               0x07;
const GPIO_CFG_SX1261_IQ:u8=               0x08;
const GPIO_CFG_STATUS:u8=                  0x09;
const GPIO_CFG_MBIST:u8=                   0x0A;
const GPIO_CFG_OTP :u8=                    0x0B;


const    RX_FREQ_TRACK_OFF:i32  = 0x00;
const    RX_FREQ_TRACK_ON:i32   = 0x01;
const    RX_FREQ_TRACK_AUTO:i32 = 0x03;



const    RX_FINE_TIMING_MODE_ABS:i32     = 0x01;
const    RX_FINE_TIMING_MODE_LINEAR:i32  = 0x02;
const    RX_FINE_TIMING_MODE_AUTO:i32    = 0x03;



pub const    RX_DFT_PEAK_MODE_DISABLED:u8    = 0x00;
pub const    RX_DFT_PEAK_MODE_FULL:u8         = 0x01;
pub const    RX_DFT_PEAK_MODE_TRACK:u8        = 0x02;
pub const    RX_DFT_PEAK_MODE_AUTO:u8         = 0x03;


pub const   CHIP_MODEL_ID_SX1302:u8 = 0x02; /* SX1302 can be 0x00 or 0x02 */
pub const   CHIP_MODEL_ID_SX1303:u8 = 0x03;
pub const   CHIP_MODEL_ID_UNKNOWN:u8 = 0xFF;



macro_rules! take_n_bits_from {
    ($b:expr, $p:expr, $n:expr) => {
        ((($b as u32) >> $p) & ((1 << $n) - 1))
    };
}


macro_rules! SX1302_PKT_PAYLOAD_LENGTH { ($buffer:expr, $start_index:expr)  =>  {  take_n_bits_from!($buffer[$start_index +  2], 0, 8) } }

macro_rules! SX1302_PKT_CHANNEL { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  3], 0, 8) } }
macro_rules! SX1302_PKT_CRC_EN { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  4], 0, 1) } }
macro_rules! SX1302_PKT_CODING_RATE { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  4], 1, 3) } }
macro_rules! SX1302_PKT_DATARATE { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  4], 4, 4) } }
macro_rules! SX1302_PKT_MODEM_ID { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  5], 0, 8) } }
macro_rules! SX1302_PKT_FREQ_OFFSET_7_0 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  6], 0, 8) } }
macro_rules! SX1302_PKT_FREQ_OFFSET_15_8 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  7], 0, 8) } }
macro_rules! SX1302_PKT_FREQ_OFFSET_19_16 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  8], 0, 4) } }
macro_rules! SX1302_PKT_CRC_ERROR { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  9], 0, 1) } }
macro_rules! SX1302_PKT_SYNC_ERROR { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  9], 2, 1) } }
macro_rules! SX1302_PKT_HEADER_ERROR { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  9], 3, 1) } }
macro_rules! SX1302_PKT_TIMING_SET { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index +  9], 4, 1) } }
macro_rules! SX1302_PKT_SNR_AVG { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 10], 0, 8) } }
macro_rules! SX1302_PKT_RSSI_CHAN { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 11], 0, 8) } }
macro_rules! SX1302_PKT_RSSI_SIG { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 12], 0, 8) } }
macro_rules! SX1302_PKT_RSSI_CHAN_MAX_NEG_DELTA { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 13], 0, 4) } }
macro_rules! SX1302_PKT_RSSI_CHAN_MAX_POS_DELTA { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 13], 4, 4) } }
macro_rules! SX1302_PKT_RSSI_SIG_MAX_NEG_DELTA { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 14], 0, 4) } }
macro_rules! SX1302_PKT_RSSI_SIG_MAX_POS_DELTA { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 14], 4, 4) } }
macro_rules! SX1302_PKT_TIMESTAMP_7_0 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 15], 0, 8) } }
macro_rules! SX1302_PKT_TIMESTAMP_15_8 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 16], 0, 8) } }
macro_rules! SX1302_PKT_TIMESTAMP_23_16 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 17], 0, 8) } }
macro_rules! SX1302_PKT_TIMESTAMP_31_24 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 18], 0, 8) } }
macro_rules! SX1302_PKT_CRC_PAYLOAD_7_0 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 19], 0, 8) } }
macro_rules! SX1302_PKT_CRC_PAYLOAD_15_8 { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 20], 0, 8) } }
macro_rules! SX1302_PKT_NUM_TS_METRICS { ($buffer:expr, $start_index:expr) => { take_n_bits_from!($buffer[$start_index + 21], 0, 8) } }



macro_rules! SX1302_REG_TX_TOP_TX_TRIG_TX_FSM_CLR {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_TRIG_TX_FSM_CLR
        } else {
            SX1302_REG_TX_TOP_B_TX_TRIG_TX_FSM_CLR
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_GPS
        } else {
            SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_GPS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_DELAYED
        } else {
            SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE
        } else {
            SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG
        } else {
            SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG
        } else {
            SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG
        } else {
            SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG
        } else {
            SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_START_DELAY_MSB_TX_START_DELAY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY
        } else {
            SX1302_REG_TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_START_DELAY_LSB_TX_START_DELAY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY
        } else {
            SX1302_REG_TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER
        } else {
            SX1302_REG_TX_TOP_B_TX_CTRL_WRITE_BUFFER
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RAMP_DURATION_TX_RAMP_DURATION {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RAMP_DURATION_TX_RAMP_DURATION
        } else {
            SX1302_REG_TX_TOP_B_TX_RAMP_DURATION_TX_RAMP_DURATION
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE
        } else {
            SX1302_REG_TX_TOP_B_GEN_CFG_0_MODULATION_TYPE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TEST_0_TX_ACTIVE_CTRL {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TEST_0_TX_ACTIVE_CTRL
        } else {
            SX1302_REG_TX_TOP_B_TEST_0_TX_ACTIVE_CTRL
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TEST_0_TX_ACTIVE_SEL {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TEST_0_TX_ACTIVE_SEL
        } else {
            SX1302_REG_TX_TOP_B_TEST_0_TX_ACTIVE_SEL
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_FLAG_TX_TIMEOUT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_FLAG_TX_TIMEOUT
        } else {
            SX1302_REG_TX_TOP_B_TX_FLAG_TX_TIMEOUT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_FLAG_PKT_DONE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_FLAG_PKT_DONE
        } else {
            SX1302_REG_TX_TOP_B_TX_FLAG_PKT_DONE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_AGC_TX_BW_AGC_TX_BW {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_AGC_TX_BW_AGC_TX_BW
        } else {
            SX1302_REG_TX_TOP_B_AGC_TX_BW_AGC_TX_BW
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_AGC_TX_PWR_AGC_TX_PWR {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_AGC_TX_PWR_AGC_TX_PWR
        } else {
            SX1302_REG_TX_TOP_B_AGC_TX_PWR_AGC_TX_PWR
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT
        } else {
            SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT
        } else {
            SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT
        } else {
            SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_FSM_STATUS_TX_STATUS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_FSM_STATUS_TX_STATUS
        } else {
            SX1302_REG_TX_TOP_B_TX_FSM_STATUS_TX_STATUS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_DUMMY_CONTROL_DUMMY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_DUMMY_CONTROL_DUMMY
        } else {
            SX1302_REG_TX_TOP_B_DUMMY_CONTROL_DUMMY
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_PLL_DIV_CTRL {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_CLK_EDGE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_MODE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_DST {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_IQ_GAIN_IQ_GAIN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_IQ_GAIN_IQ_GAIN
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_IQ_GAIN_IQ_GAIN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_I_OFFSET_I_OFFSET {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_Q_OFFSET_Q_OFFSET {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET
        }
    };
}



macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_H_FREQ_RF { ($rf_chain:expr) => { 
        if ($rf_chain == 0) 
        { 
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF 
        } 
        else{
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_H_FREQ_RF
        }
    }
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_M_FREQ_RF {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_M_FREQ_RF
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_L_FREQ_RF {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_L_FREQ_RF
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_RFFE_IF_TEST_MOD_FREQ {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_RFFE_IF_TEST_MOD_FREQ
        } else {
            SX1302_REG_TX_TOP_B_TX_RFFE_IF_TEST_MOD_FREQ
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_DUMMY_MODULATOR_DUMMY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_DUMMY_MODULATOR_DUMMY
        } else {
            SX1302_REG_TX_TOP_B_DUMMY_MODULATOR_DUMMY
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_PKT_LEN_PKT_LENGTH {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_PKT_LEN_PKT_LENGTH
        } else {
            SX1302_REG_TX_TOP_B_FSK_PKT_LEN_PKT_LENGTH
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_CFG_0_TX_CONT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_CFG_0_TX_CONT
        } else {
            SX1302_REG_TX_TOP_B_FSK_CFG_0_TX_CONT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_CFG_0_CRC_IBM {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_CFG_0_CRC_IBM
        } else {
            SX1302_REG_TX_TOP_B_FSK_CFG_0_CRC_IBM
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_CFG_0_DCFREE_ENC {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_CFG_0_DCFREE_ENC
        } else {
            SX1302_REG_TX_TOP_B_FSK_CFG_0_DCFREE_ENC
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_CFG_0_CRC_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_CFG_0_CRC_EN
        } else {
            SX1302_REG_TX_TOP_B_FSK_CFG_0_CRC_EN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_CFG_0_PKT_MODE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_CFG_0_PKT_MODE
        } else {
            SX1302_REG_TX_TOP_B_FSK_CFG_0_PKT_MODE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE
        } else {
            SX1302_REG_TX_TOP_B_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE
        } else {
            SX1302_REG_TX_TOP_B_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_BIT_RATE_MSB_BIT_RATE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_BIT_RATE_MSB_BIT_RATE
        } else {
            SX1302_REG_TX_TOP_B_FSK_BIT_RATE_MSB_BIT_RATE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_BIT_RATE_LSB_BIT_RATE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_BIT_RATE_LSB_BIT_RATE
        } else {
            SX1302_REG_TX_TOP_B_FSK_BIT_RATE_LSB_BIT_RATE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_SIZE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_SIZE
        } else {
            SX1302_REG_TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_SIZE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_MOD_FSK_PREAMBLE_SEQ {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_MOD_FSK_PREAMBLE_SEQ
        } else {
            SX1302_REG_TX_TOP_B_FSK_MOD_FSK_PREAMBLE_SEQ
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_EN
        } else {
            SX1302_REG_TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_EN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_SELECT_BT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_SELECT_BT
        } else {
            SX1302_REG_TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_SELECT_BT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_EN
        } else {
            SX1302_REG_TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_EN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
        }
    };
}



macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
        } else {
            SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_DUMMY_GSFK_DUMMY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_DUMMY_GSFK_DUMMY
        } else {
            SX1302_REG_TX_TOP_B_DUMMY_GSFK_DUMMY
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_BW {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_SF {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_SF
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_1_CODING_RATE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_1_CODING_RATE
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_1_CODING_RATE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_2_FINE_SYNCH_EN
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_2_FINE_SYNCH_EN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_2_MODEM_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_2_MODEM_EN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_2_CADRXTX {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CADRXTX
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_2_IMPLICIT_HEADER {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_2_IMPLICIT_HEADER
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_2_CRC_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CRC_EN
        }
    };
}



macro_rules! SX1302_REG_TX_TOP_TXRX_CFG0_3_PAYLOAD_LENGTH {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG0_3_PAYLOAD_LENGTH
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_0_INT_STEP_ORIDE_EN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE_EN
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE_EN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_0_INT_STEP_ORIDE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_1_MODEM_START {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_1_MODEM_START
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_1_HEADER_DIFF_MODE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_1_HEADER_DIFF_MODE
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_1_HEADER_DIFF_MODE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_1_ZERO_PAD {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_1_ZERO_PAD
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_1_ZERO_PAD
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_2_PREAMBLE_SYMB_NB {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_3_PREAMBLE_SYMB_NB {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_4_AUTO_ACK_INT_DELAY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_INT_DELAY
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_INT_DELAY
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_4_AUTO_ACK_RX {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_RX
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_RX
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TXRX_CFG1_4_AUTO_ACK_TX {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_TX
        } else {
            SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_TX
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG0_0_PPM_OFFSET_SIG {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG0_0_PPM_OFFSET_SIG
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG0_0_PPM_OFFSET_SIG
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG0_0_CONTCHIRP {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTCHIRP
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTCHIRP
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_INVERT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_INVERT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG0_0_CONTINUOUS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTINUOUS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG0_1_POWER_RANGING {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG0_1_POWER_RANGING
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG0_1_POWER_RANGING
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG1_0_FRAME_NB {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG1_0_FRAME_NB
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG1_0_FRAME_NB
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG1_1_HOP_CTRL {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG1_1_HOP_CTRL
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG1_1_HOP_CTRL
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_TX_CFG1_1_IFS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_TX_CFG1_1_IFS
        } else {
            SX1302_REG_TX_TOP_B_TX_CFG1_1_IFS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_0_AUTO_SCALE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_AUTO_SCALE
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_AUTO_SCALE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_0_DROP_ON_SYNCH {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_DROP_ON_SYNCH
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_DROP_ON_SYNCH
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_0_GAIN {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_GAIN
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_GAIN
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_PEAK1_POS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_1_FINETIME_ON_LAST {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_FINETIME_ON_LAST
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_FINETIME_ON_LAST
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_1_TIMEOUT_OPT {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_TIMEOUT_OPT
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_TIMEOUT_OPT
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS
        } else {
            SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_PEAK2_POS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_LORA_TX_STATE_STATUS {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS
        } else {
            SX1302_REG_TX_TOP_B_LORA_TX_STATE_STATUS
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_LORA_TX_FLAG_FRAME_DONE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE
        } else {
            SX1302_REG_TX_TOP_B_LORA_TX_FLAG_FRAME_DONE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_LORA_TX_FLAG_CONT_DONE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_LORA_TX_FLAG_CONT_DONE
        } else {
            SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_LORA_TX_FLAG_PLD_DONE {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_LORA_TX_FLAG_PLD_DONE
        } else {
            SX1302_REG_TX_TOP_B_LORA_TX_FLAG_PLD_DONE
        }
    };
}


macro_rules! SX1302_REG_TX_TOP_DUMMY_LORA_DUMMY {
    ($rf_chain:expr) => {
        if $rf_chain == 0 {
            SX1302_REG_TX_TOP_A_DUMMY_LORA_DUMMY
        } else {
            SX1302_REG_TX_TOP_B_DUMMY_LORA_DUMMY
        }
    };
}

macro_rules! SX1302_FREQ_TO_REG {
    ($f: expr) => {
        ($f as u64 * (1 << 18) / 32000000u64) as u32
    };
} 

macro_rules! SX1250_FREQ_TO_REG {
    ($f: expr) => {
        ($f as u64 * (1 << 25) / 32000000u64) as u32
    } 
}
/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* RX buffer packet structure */
const SX1302_PKT_SYNCWORD_BYTE_0:u8 =  0xA5;
const SX1302_PKT_SYNCWORD_BYTE_1:u8 =  0xC0;
const SX1302_PKT_HEAD_METADATA:u8 =    9;
const SX1302_PKT_TAIL_METADATA:u8 =    14;

/* modem IDs */
const SX1302_LORA_MODEM_ID_MAX:u8 =    15;
const SX1302_LORA_STD_MODEM_ID:u8 =    16;
const SX1302_FSK_MODEM_ID:u8 =         17;


fn reg_select(rf_chain: u8, a: u16, b: u16) -> u16 {
    if rf_chain == 0 { a } else { b }
}
  

fn IF_HZ_TO_REG(f: i32)-> i32 {
    (f * 32) / 15625
}


pub fn SET_PPM_ON(bw: u8, dr:u32) -> bool {
    ((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12))
}  

fn calculate_freq_to_time_drift( freq_hz:u32,  bw:u8 )-> Result<(u16, u8)> {
    let mut mantissa_u64: u64;
    let mut exponent:u8 = 0;
    let bw_hz:i32;

   
    bw_hz = Hal::lgw_bw_getval(bw);
    if bw_hz < 0 {
        error!("ERROR: Unsupported bandwidth for frequency to time drift calculation");
        return Err(anyhow!("LGW_HAL_ERR"));
    }

    mantissa_u64 = bw_hz as u64 * (2 << (20-1)) / freq_hz as u64;

    while mantissa_u64 < 2048 {
        exponent += 1;
        mantissa_u64 <<= 1;
    }

    Ok((mantissa_u64 as u16, exponent))
    
}

#[derive(Debug)]
pub struct RxPacket {
    pub rxbytenb_modem: u8,
    pub rx_channel_in: u8,
    pub crc_en: bool,
    pub coding_rate: u8,                /* LoRa only */
    pub rx_rate_sf: u8,                 /* LoRa only */
    pub modem_id: u8,
    pub frequency_offset_error: i32,     /* LoRa only */
    pub payload: [u8; 255],
    pub payload_crc_error: bool,
    pub sync_error: bool,                 /* LoRa only */
    pub header_error: bool,               /* LoRa only */
    pub timing_set: bool,                 /* LoRa only */
    pub snr_average: i8,                /* LoRa only */
    pub rssi_chan_avg: u8,
    pub rssi_signal_avg: u8,            /* LoRa only */
    pub rssi_chan_max_neg_delta: u8,
    pub rssi_chan_max_pos_delta: u8,
    pub rssi_sig_max_neg_delta: u8,     /* LoRa only */
    pub rssi_sig_max_pos_delta: u8,     /* LoRa only */
    pub timestamp_cnt: u32,
    pub rx_crc16_value: u16,             /* LoRa only */
    pub num_ts_metrics_stored: u8,      /* LoRa only */
    pub timestamp_avg: [i8; 255],         /* LoRa only */
    pub timestamp_stddev: [i8; 255],      /* LoRa only */
    pub packet_checksum: u8,
}

impl RxPacket {
    pub fn new() -> Self {
        Self {
            rxbytenb_modem: 0,
            rx_channel_in: 0,
            crc_en: false,
            coding_rate: 0,
            rx_rate_sf: 0,
            modem_id: 0,
            frequency_offset_error: 0,
            payload: [0; 255],
            payload_crc_error: false,
            sync_error: false,
            header_error: false,
            timing_set: false,
            snr_average: 0,
            rssi_chan_avg: 0,
            rssi_signal_avg: 0,
            rssi_chan_max_neg_delta: 0,
            rssi_chan_max_pos_delta: 0,
            rssi_sig_max_neg_delta: 0,
            rssi_sig_max_pos_delta: 0,
            timestamp_cnt: 0,
            rx_crc16_value: 0,
            num_ts_metrics_stored: 0,
            timestamp_avg: [0; 255],
            timestamp_stddev: [0; 255],
            packet_checksum: 0,
        }
    }
}

#[derive(Debug)]
pub struct RxBuffer {
    pub buffer:[u8;4096],   /* byte array to hald the data fetched from the RX buffer */
    pub buffer_size: usize,   /* The number of bytes currently stored in the buffer */
    pub buffer_index:usize,       /* Current parsing index in the buffer */
    pub buffer_pkt_nb:u8
} 



impl RxBuffer {
    pub fn new()-> Self {
        Self {
            buffer: [0u8; 4096],
            buffer_size: 0,
            buffer_index: 0,
            buffer_pkt_nb: 0
        }
    }

    pub fn clear(&mut self) {
        self.buffer.fill(0);
        self.buffer_size = 0;
        self.buffer_index = 0;
        self.buffer_pkt_nb = 0;
    }

    pub fn del(&mut self) {
        
        /* Reset index & size */
        self.buffer_size = 0;
        self.buffer_index = 0;
        self.buffer_pkt_nb = 0;
    
    }

    pub fn pop(&mut self) -> Result<RxPacket> {
        let mut pkt = RxPacket::new();

        /* Is there any data to be parsed ? */
        if self.buffer_index >= self.buffer_size  {
            error!("INFO: No more data to be parsed\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
    
        /* Get pkt sync words */
        if (self.buffer[self.buffer_index] != SX1302_PKT_SYNCWORD_BYTE_0) || (self.buffer[self.buffer_index + 1] != SX1302_PKT_SYNCWORD_BYTE_1) {
            return Err(anyhow!("LGW_REG_ERR"));
        }

        trace!("INFO: pkt syncword found at index {:}", self.buffer_index);
    
        /* Get payload length */
        pkt.rxbytenb_modem = SX1302_PKT_PAYLOAD_LENGTH!(self.buffer, self.buffer_index) as u8;
    
        /* Get fine timestamp metrics */
        pkt.num_ts_metrics_stored = SX1302_PKT_NUM_TS_METRICS!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) as u8;
    
        /* Calculate the total number of bytes in the packet */
        let pkt_num_bytes = SX1302_PKT_HEAD_METADATA + pkt.rxbytenb_modem + SX1302_PKT_TAIL_METADATA + (2 * pkt.num_ts_metrics_stored);
    
        /* Check if we have a complete packet in the rx buffer fetched */
        if(self.buffer_index + pkt_num_bytes as usize) > self.buffer_size {
            warn!("WARNING: aborting truncated message (size={:})\n", self.buffer_size);
            return Err(anyhow!("LGW_REG_ERR"));
        }
    
        /* Get the checksum as received in the RX buffer */
        let checksum_idx = pkt_num_bytes - 1;
        let checksum_rcv = self.buffer[self.buffer_index + pkt_num_bytes as usize - 1];
        
        let mut checksum_calc:u32 = 0;

        debug!(checksum_idx=%checksum_idx, "do checksum calc");
        /* Calculate the checksum from the actual payload bytes received */
        for i in 0 .. checksum_idx  as usize {
            checksum_calc += self.buffer[self.buffer_index + i] as u32;
        }
    
        /* Check if the checksum is correct */
        if checksum_rcv != checksum_calc as u8 {
            warn!("WARNING: checksum failed (got:0x{:02X} calc:0x{:02X})\n", checksum_rcv, checksum_calc);
            return Err(anyhow!("LGW_REG_ERR"));
        } else {
            trace!("Packet checksum OK (0x{:02X})\n", checksum_rcv);
        }
    
        /* Parse packet metadata */
        pkt.modem_id = SX1302_PKT_MODEM_ID!(self.buffer, self.buffer_index) as u8;
        pkt.rx_channel_in = SX1302_PKT_CHANNEL!(self.buffer, self.buffer_index) as u8;
        pkt.crc_en = SX1302_PKT_CRC_EN!(self.buffer, self.buffer_index) != 0;
        pkt.payload_crc_error = SX1302_PKT_CRC_ERROR!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) != 0;
        pkt.sync_error = SX1302_PKT_SYNC_ERROR!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) != 0;
        pkt.header_error = SX1302_PKT_HEADER_ERROR!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) != 0;
        pkt.timing_set = SX1302_PKT_TIMING_SET!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) != 0;
        pkt.coding_rate = SX1302_PKT_CODING_RATE!(self.buffer, self.buffer_index) as u8;
        pkt.rx_rate_sf = SX1302_PKT_DATARATE!(self.buffer, self.buffer_index) as u8;
        pkt.rssi_chan_avg = SX1302_PKT_RSSI_CHAN!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) as u8;
        pkt.rssi_signal_avg = SX1302_PKT_RSSI_SIG!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) as u8;
        pkt.rx_crc16_value  = ((SX1302_PKT_CRC_PAYLOAD_7_0!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) <<  0) & 0x00FF) as u16;
        pkt.rx_crc16_value |= ((SX1302_PKT_CRC_PAYLOAD_15_8!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) <<  8) & 0xFF00) as u16;
        pkt.snr_average = SX1302_PKT_SNR_AVG!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) as i8;
    
        pkt.frequency_offset_error = ((SX1302_PKT_FREQ_OFFSET_19_16!(self.buffer, self.buffer_index) << 16) | (SX1302_PKT_FREQ_OFFSET_15_8!(self.buffer, self.buffer_index) << 8) | (SX1302_PKT_FREQ_OFFSET_7_0!(self.buffer, self.buffer_index) << 0)) as i32;
        if pkt.frequency_offset_error >= (1<<19) { /* Handle signed value on 20bits */
            pkt.frequency_offset_error = pkt.frequency_offset_error - (1<<20);
        }
    
        /* Packet timestamp (32MHz ) */
        pkt.timestamp_cnt  = ((SX1302_PKT_TIMESTAMP_7_0!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) <<  0) & 0x000000FF) as u32;
        pkt.timestamp_cnt |= ((SX1302_PKT_TIMESTAMP_15_8!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) <<  8) & 0x0000FF00) as u32;
        pkt.timestamp_cnt |= ((SX1302_PKT_TIMESTAMP_23_16!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) << 16) & 0x00FF0000) as u32;
        pkt.timestamp_cnt |= ((SX1302_PKT_TIMESTAMP_31_24!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize) << 24) & 0xFF000000) as u32;
    
        /* TS metrics: it is expected the nb_symbols parameter is set to 0 here */
        for i in 0 .. (pkt.num_ts_metrics_stored * 2) as usize{
            pkt.timestamp_avg[i] = SX1302_PKT_NUM_TS_METRICS!(self.buffer, self.buffer_index + pkt.rxbytenb_modem as usize + 1 + i) as i8;
            pkt.timestamp_stddev[i] = 0; /* no stddev when nb_symbols == 0 */
        }
        trace!(pkt=?pkt, "parsed pkt:");

        /* Sanity checks: check the range of few metadata */
        if pkt.modem_id > SX1302_FSK_MODEM_ID {
            error!("ERROR: modem_id is out of range - {:}\n", pkt.modem_id);
            return Err(anyhow!("LGW_REG_ERROR"));
        } else {
            if pkt.modem_id <= SX1302_LORA_STD_MODEM_ID { /* LoRa modems */
                if pkt.rx_channel_in > 9 {
                    error!("ERROR: channel is out of range - {:}\n", pkt.rx_channel_in);
                    return Err(anyhow!("LGW_REG_ERROR"));
                }
                if (pkt.rx_rate_sf < 5) || (pkt.rx_rate_sf > 12) {
                    error!("ERROR: SF is out of range - {:}\n", pkt.rx_rate_sf);
                    return Err(anyhow!("LGW_REG_ERROR"));
                }
            } else { /* FSK modem */
                /* TODO: not checked */
            }
        }
        /* Parse & copy payload in packet struct */
        pkt.payload[.. pkt.rxbytenb_modem as usize].copy_from_slice(&self.buffer[(self.buffer_index + SX1302_PKT_HEAD_METADATA as usize) .. (self.buffer_index + SX1302_PKT_HEAD_METADATA as usize +   pkt.rxbytenb_modem as usize) ]);
        
    
        /* Move buffer index toward next message */
        self.buffer_index += (SX1302_PKT_HEAD_METADATA + pkt.rxbytenb_modem + SX1302_PKT_TAIL_METADATA + (2 * pkt.num_ts_metrics_stored)) as usize;
    
        /* Update the number of packets currently stored in the rx_buffer */
        self.buffer_pkt_nb -= 1;
    
        Ok(pkt)
    }
}

#[derive(Debug)]
pub struct Sx1302 {
    /* Buffer to hold RX data */
    pub rx_buffer: RxBuffer,

    /* Internal timestamp counter */
    pub counter_us: TimestampCounter,

    pub timestamp_pps_history: TimestampPpsHistory
}

impl Sx1302 {
    pub fn new() -> Self {
        Self {
            rx_buffer: RxBuffer::new(),
            counter_us: TimestampCounter::new(),
            timestamp_pps_history: TimestampPpsHistory::new()
        }
    }
}

pub trait LorgwSx1302Trait {
    fn sx1302_rx_status(&mut self,  rf_chain: u8) -> u8;

    fn sx1302_init(&mut self, ftime_context: &LgwConfigFtime) -> Result<()>;
    
    fn sx1302_set_gpio(&mut self, gpio_reg_val:u8) -> Result<()>;
    
    fn sx1302_radio_calibrate(&mut self, context_rf_chain: &[LgwConfRxrf],  clksrc:u8, txgain_lut: &[LgwTxGainLut])->Result<()>;

    fn sx1302_radio_reset(&mut self, radio:u8,  radio_type:LgwRadioType) -> Result<()>;

    fn sx1302_radio_set_mode(&mut self, radio:u8,  radio_type:LgwRadioType) -> Result<()>;

    fn sx1302_radio_clock_select(&mut self, clksrc:u8) -> Result<()>;

    fn sx1302_agc_load_firmware(&mut self, firmware: &[u8]) -> Result<()>;

    fn sx1302_cal_start(&mut self, fw_version:u8, context_rf_chain: &[LgwConfRxrf], txgain_lut: &[LgwTxGainLut])->Result<()>;

    fn sx1302_get_ifmod_config(if_chain:u8)-> u8;

    fn sx1302_radio_host_ctrl(&mut self, host_ctrl: bool)->Result<()>;

    fn sx1302_get_model_id(&mut self) -> Result<u8>;

    fn sx1302_config_gpio(&mut self) -> Result<()>; 

    fn sx1302_pa_lna_lut_configure(&mut self, context_board: &LgwConfigBoard) -> Result<()>;

    fn sx1302_radio_fe_configure(&mut self) -> Result<()>;

    fn sx1302_channelizer_configure(&mut self, if_cfg: &[LgwConfRxIf],  fix_gain: bool)-> Result<()>; 

    fn sx1302_lora_correlator_configure(&mut self,  if_cfg: &[LgwConfRxIf],  demod_cfg:&LgwConfDemod) -> Result<()>;

    fn sx1302_lora_modem_configure(&mut self,  radio_freq_hz:u32)-> Result<()>;

    fn sx1302_lora_service_correlator_configure(&mut self,  cfg: &LgwConfRxIf) -> Result<()>;

    fn sx1302_lora_service_modem_configure(&mut self, cfg: &LgwConfRxIf, radio_freq_hz: u32) -> Result<()>;

    fn sx1302_fsk_configure(&mut self, cfg: &LgwConfRxIf) -> Result<()>; 

    fn sx1302_lora_syncword(&mut self,  public: bool,  lora_service_sf: u8) -> Result<()>; 

    fn sx1302_modem_enable(&mut self)->Result<()>;

    fn sx1302_agc_start(&mut self, version: u8,  radio_type:LgwRadioType,  ana_gain: u8,  dec_gain:u8,  full_duplex:bool,  lbt_enable: bool) -> Result<()>;

    fn sx1302_agc_wait_status(&mut self,  status: u8) -> Result<()>;

    fn sx1302_agc_status(&mut self) -> Result<u8>;

    fn sx1302_agc_mailbox_read(&mut self, mailbox:u8 ) ->Result<u8>;

    fn sx1302_agc_mailbox_write(&mut self,  mailbox:u8,  value:u8) -> Result<()> ;

    fn sx1302_arb_load_firmware(&mut self,  firmware:&[u8])-> Result<()>;

    fn sx1302_arb_start(&mut self, version: u8,  ftime_context: &LgwConfigFtime) -> Result<()>;

    fn sx1302_arb_wait_status(&mut self,  status: u8) -> Result<()>;

    fn sx1302_arb_status(&mut self)->Result<u8>;

    fn sx1302_arb_debug_read(&mut self,  reg_id:u8)-> Result<u8>;

    fn sx1302_arb_set_debug_stats(&mut self,  enable:bool,  sf: u8) -> Result<()>;
    
    fn sx1302_arb_debug_write(&mut self,  reg_id: u8,  value:u8) -> Result<()>;

    fn sx1302_tx_configure(&mut self,  radio_type:LgwRadioType) -> Result<()>;

    fn sx1302_gps_enable(&mut self,  enable: bool) -> Result<()>;

    fn sx1302_fetch(&mut self)->Result<u8>;

    fn rx_buffer_fetch(&mut self) -> Result<()>;

    fn sx1302_update(&mut self) -> Result<()>;

    fn timestamp_counter_get( &mut self) -> Result<(u32,u32)>;

    fn sx1302_parse(&mut self) -> Result<LgwPktRx>;

    fn precise_timestamp_calculate(&mut self, ts_metrics_nb: u8,  ts_metrics: &[i8],  timestamp_cnt:u32,  sf:u8,  if_freq_hz:i32,  pkt_freq_error:f64) -> Result<u32> ;

    fn sx1302_send(&mut self,  radio_type: LgwRadioType,  tx_lut: &LgwTxGainLut,  lwan_public: bool, context_fsk: &LgwConfRxIf,  pkt_data: &mut LgwPktTx) -> Result<()>;

    fn sx1302_tx_set_start_delay(&mut self,  rf_chain: u8,  radio_type: LgwRadioType,  modulation: Modulation,  bandwidth: u8,  chirp_lowpass: u8) -> Result<u16>;

    fn sx1302_timestamp_counter(&mut self, pps: bool) -> Result<u32>;

    fn sx1302_tx_abort(&mut self,  rf_chain:u8) -> Result<()>;

    fn sx1302_tx_status(&mut self, rf_chain:u8) -> u8;
}   




impl LorgwSx1302Trait for Hal {


    fn sx1302_tx_status(&mut self, rf_chain:u8) -> u8 {
        let read_value;
        match self.lgw_reg_r(SX1302_REG_TX_TOP_TX_FSM_STATUS_TX_STATUS!(rf_chain)) {
            Ok(value) => {
                read_value = value as u8;
            }
            Err(_) => {
                return TX_STATUS_UNKNOWN;
            }
        }

        if read_value == 0x80 {
            return TX_FREE;
        } else if (read_value == 0x30) || (read_value == 0x50) || (read_value == 0x60) || (read_value == 0x70) {
            return TX_EMITTING;
        } else if (read_value == 0x91) || (read_value == 0x92) {
            return TX_SCHEDULED;
        } else {
            error!("ERROR: UNKNOWN TX STATUS 0x{:02X}", read_value);
            return TX_STATUS_UNKNOWN;
        }
    }

    fn sx1302_rx_status(&mut self,  _rf_chain: u8) -> u8 {

        return RX_STATUS_UNKNOWN;
    }


    fn sx1302_tx_abort(&mut self,  rf_chain:u8) -> Result<()> {


        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE!(rf_chain), 0x00)?;
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED!(rf_chain), 0x00)?;
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS!(rf_chain), 0x00)?;

        let mut tx_status;

        let start = std::time::Instant::now();
        
        loop {
           
            let to = std::time::Duration::from_millis(1000);
            /* handle timeout */
            if start.elapsed().ge(&to) {
                error!("ERROR: TIMEOUT on TX abort\n");
                return Err(Error::LGW_REG_ERROR.into());
            }

            /* get tx status */
            tx_status = self.sx1302_tx_status(rf_chain);
            wait_ms(1);

            if tx_status == TX_FREE {
                break;
            }
        }

        Ok(())
    }

    fn sx1302_timestamp_counter(&mut self, pps: bool) -> Result<u32> {
        let (inst_cnt, pps_cnt) = self.timestamp_counter_get()?;
        if pps == true {
            Ok(pps_cnt)
        } else{
            Ok(inst_cnt)
        } 
    }

    fn sx1302_tx_set_start_delay(&mut self,  rf_chain: u8,  radio_type: LgwRadioType,  modulation: Modulation,  bandwidth: u8,  chirp_lowpass: u8) -> Result<u16> {

        let mut tx_start_delay: u16 = TX_START_DELAY_DEFAULT as u16 * 32 ;
        let mut radio_bw_delay:u16 ;
        let filter_delay: u16 ;
        let modem_delay: u16 ;
        let bw_hz: i32 = Hal::lgw_bw_getval(bandwidth);
        let mut buff=[0u8;2]; /* for 16 bits register write operation */
    
        /* tx start delay only necessary for beaconing (LoRa) */
        if modulation != Modulation::LORA {
            return Ok(0);
        }
    
        /* Adjust with radio type and bandwidth */
        match radio_type {
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => {
                if bandwidth == BW_125KHZ {
                    radio_bw_delay = 19;
                } else if bandwidth == BW_250KHZ {
                    radio_bw_delay = 24;
                } else if bandwidth == BW_500KHZ {
                    radio_bw_delay = 21;
                } else {
                    error!("ERROR: bandwidth not supported\n");
                    return Err(Error::LGW_REG_ERROR.into());
                }
            },
            LgwRadioType::LGW_RADIO_TYPE_SX1255 | LgwRadioType::LGW_RADIO_TYPE_SX1257 => {
                radio_bw_delay = 3*32 + 4;
                if bandwidth == BW_125KHZ {
                    radio_bw_delay += 0;
                } else if bandwidth == BW_250KHZ {
                    radio_bw_delay += 6;
                } else if bandwidth == BW_500KHZ {
                    radio_bw_delay += 0;
                } else {
                    error!("ERROR: bandwidth not supported\n");
                    return Err(Error::LGW_REG_ERROR.into());
                }
            },
            _ => {
                error!("ERROR: radio type not supported\n");
                return Err(Error::LGW_REG_ERROR.into());
            }
        }
    
        /* Adjust with modulation */
        filter_delay = (((1 << chirp_lowpass) - 1) * 1000000 / bw_hz) as u16;
        modem_delay = (8 * (32000000 / (32 * bw_hz))) as u16; /* if bw=125k then modem freq=4MHz */
    
        /* Compute total delay */
        tx_start_delay -= radio_bw_delay  + filter_delay + modem_delay;
    
        debug!("INFO: tx_start_delay={} ({}, radio_bw_delay={}, filter_delay={}, modem_delay={})\n", tx_start_delay as u16, TX_START_DELAY_DEFAULT*32, radio_bw_delay, filter_delay, modem_delay);
    
        buff[0] = (tx_start_delay >> 8) as u8;
        buff[1] = (tx_start_delay >> 0) as u8;
        self.lgw_reg_wb(SX1302_REG_TX_TOP_TX_START_DELAY_MSB_TX_START_DELAY!(rf_chain), &buff, 2)?;
         
        
    
        /* return tx_start_delay */
        let delay = tx_start_delay;
    
        Ok(delay)
    }
    
   
    

    fn sx1302_send(&mut self,  radio_type: LgwRadioType,  tx_lut: &LgwTxGainLut,  lwan_public: bool, context_fsk: &LgwConfRxIf,  pkt_data: &mut LgwPktTx) -> Result<()> {
        
        let freq_reg:u32;
        let mut fdev_reg:u32;
        let freq_dev:u32;
        let fsk_br_reg:u32;
        let fsk_sync_word_reg:u64;
        let mem_addr:u16;
        let count_us:u32;
        let power:u8;
        let mod_bw:u8;
        let pa_en:u8;
        let mut chirp_lowpass:u8 = 0;
        let mut buff = [0u8;2]; /* for 16-bits register write operation */

    
        /* Setting BULK write mode (to speed up configuration on USB) */
        self.mcu.mcu_set_write_mode(EComWriteMode::LGW_COM_WRITE_MODE_BULK);
    
        /* Select the proper modem */
        match pkt_data.modulation {
             Modulation::CW =>{
                self.lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE!(pkt_data.rf_chain) , 0x00)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC!(pkt_data.rf_chain), 0x00)?;
            },
            Modulation::LORA => {
                self.lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE!(pkt_data.rf_chain), 0x00)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC!(pkt_data.rf_chain), 0x01)?;
                
            }
            Modulation::FSK => {
                self.lgw_reg_w(SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE!(pkt_data.rf_chain), 0x01)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC!(pkt_data.rf_chain), 0x02)?;

            }
            _ => {
                error!("ERROR: modulation type not supported\n");
                return Err(Error::LGW_REG_ERROR.into());
            }
        }

        let mut pow_index = tx_lut.size - 1;
        while pow_index > 0 && pow_index < tx_lut.lut.len() {
            if tx_lut.lut[pow_index].rf_power <= pkt_data.rf_power {
                break
            }
            pow_index -= 1;
        }
        
        
        info!("INFO: selecting TX Gain LUT index {:}\n", pow_index);
    
        /* loading calibrated Tx DC offsets */
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_I_OFFSET_I_OFFSET!(pkt_data.rf_chain), tx_lut.lut[pow_index].offset_i as i32)?;
        
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_Q_OFFSET_Q_OFFSET!(pkt_data.rf_chain), tx_lut.lut[pow_index].offset_q as i32)?;
        
    
        trace!("INFO: Applying IQ offset (i:{:}, q:{:})\n", tx_lut.lut[pow_index].offset_i, tx_lut.lut[pow_index].offset_q);
    
        /* Set the power parameters to be used for TX */
        match radio_type {
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => {
                pa_en = if tx_lut.lut[pow_index].pa_gain > 0 { 1 } else { 0 }; /* only 1 bit used to control the external PA */
                power = (pa_en << 6) | tx_lut.lut[pow_index].pwr_idx;
            },
            LgwRadioType::LGW_RADIO_TYPE_SX1255 | LgwRadioType::LGW_RADIO_TYPE_SX1257 => {
                power = (tx_lut.lut[pow_index].pa_gain << 6) | (tx_lut.lut[pow_index].dac_gain << 4) | tx_lut.lut[pow_index].mix_gain;
            },
            _ => {
                error!("ERROR: radio type not supported\n");
                return Err(Error::LGW_REG_ERROR.into());
            }
        }
        self.lgw_reg_w(SX1302_REG_TX_TOP_AGC_TX_PWR_AGC_TX_PWR!(pkt_data.rf_chain), power as i32)?;
        
    
        /* Set digital gain */
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_IQ_GAIN_IQ_GAIN!(pkt_data.rf_chain), tx_lut.lut[pow_index].dig_gain as i32)?;
        
    
        /* Set Tx frequency */
        if radio_type == LgwRadioType::LGW_RADIO_TYPE_SX1255 {
            freq_reg = SX1302_FREQ_TO_REG!(pkt_data.freq_hz * 2);
        } else {
            freq_reg = SX1302_FREQ_TO_REG!(pkt_data.freq_hz);
        }
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_H_FREQ_RF!(pkt_data.rf_chain), ((freq_reg >> 16) & 0xFF) as i32)?;
        
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_M_FREQ_RF!(pkt_data.rf_chain), ((freq_reg >> 8) & 0xFF) as i32)?;
        
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_L_FREQ_RF!(pkt_data.rf_chain), ((freq_reg >> 0) & 0xFF) as i32)?;
        
    
        /* Set AGC bandwidth and modulation type*/
        mod_bw = match pkt_data.modulation {
            Modulation::LORA => pkt_data.bandwidth,
            Modulation::CW | Modulation::FSK => (0x01 << 7) | pkt_data.bandwidth,
            _ => {
                println!("ERROR: Modulation not supported");
                return Err(Error::LGW_REG_ERROR.into());
            }
        };

        self.lgw_reg_w(SX1302_REG_TX_TOP_AGC_TX_BW_AGC_TX_BW!(pkt_data.rf_chain), mod_bw as i32)?;
        
    
        /* Configure modem */
        match pkt_data.modulation {
            Modulation::CW => {
                /* Set frequency deviation */
                freq_dev = ceil(fabs( (pkt_data.freq_offset as f64) / 10f64) ) as u32 * 10000;
                debug!("CW: f_dev {:} Hz\n", (freq_dev as i32));
                fdev_reg = SX1302_FREQ_TO_REG!(freq_dev);
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV!(pkt_data.rf_chain), ((fdev_reg  >>  8) & 0xFF) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV!(pkt_data.rf_chain), ((fdev_reg >>  0) & 0xFF) as i32)?;
                
    
                /* Send frequency deviation to AGC fw for radio config */
                fdev_reg = SX1250_FREQ_TO_REG!(freq_dev);
                self.lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA, ((fdev_reg >> 16) & 0xFF) as i32)?; /* Needed by AGC to configure the sx1250 */
                
                self.lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA, ((fdev_reg >>  8) & 0xFF) as i32)?; /* Needed by AGC to configure the sx1250 */
                
                self.lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA, ((fdev_reg >>  0) & 0xFF) as i32)?; /* Needed by AGC to configure the sx1250 */
                
    
                /* Set the frequency offset (ratio of the frequency deviation)*/
                debug!("CW: IF test mod freq {:}\n", (pkt_data.freq_offset as f32 * 1000.0 * 64.0 / freq_dev as f32) as i32);
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_TEST_MOD_FREQ!(pkt_data.rf_chain), (((pkt_data.freq_offset as f32 * 1000.0 * 64.0) / (freq_dev as f32))) as i32)?;
                                            
            },
            Modulation::LORA => {
                /* Set bandwidth */
                freq_dev = Hal::lgw_bw_getval(pkt_data.bandwidth) as u32 / 2;
                fdev_reg = SX1302_FREQ_TO_REG!(freq_dev);
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV!(pkt_data.rf_chain), ((fdev_reg >>  8) & 0xFF) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV!(pkt_data.rf_chain), ((fdev_reg >>  0) & 0xFF) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_BW!(pkt_data.rf_chain), pkt_data.bandwidth as i32)?;
                
    
                /* Preamble length */
                if pkt_data.preamble == 0 { /* if not explicit, use recommended LoRa preamble size */
                    pkt_data.preamble = STD_LORA_PREAMBLE as u16;
                } else if pkt_data.preamble < MIN_LORA_PREAMBLE as u16 { /* enforce minimum preamble size */
                    pkt_data.preamble = MIN_LORA_PREAMBLE as u16;
                    debug!("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
                }
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_3_PREAMBLE_SYMB_NB!(pkt_data.rf_chain), ((pkt_data.preamble >> 8) & 0xFF) as i32)?; /* MSB */
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_2_PREAMBLE_SYMB_NB!(pkt_data.rf_chain), ((pkt_data.preamble >> 0) & 0xFF) as i32)?; /* LSB */
                
    
                /* LoRa datarate */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_SF!(pkt_data.rf_chain), pkt_data.datarate as i32)?;
                
    
                /* Chirp filtering */
                chirp_lowpass = if pkt_data.datarate < 10  { 6 } else { 7 };
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS!(pkt_data.rf_chain), chirp_lowpass as i32)?;
                
    
                /* Coding Rate */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_CODING_RATE!(pkt_data.rf_chain), pkt_data.coderate as i32)?;
                
    
                /* Start LoRa modem */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_MODEM_EN!(pkt_data.rf_chain), 1)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_CADRXTX!(pkt_data.rf_chain), 2)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG1_1_MODEM_START!(pkt_data.rf_chain), 1)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CONTINUOUS!(pkt_data.rf_chain), 0)?;
                
    
                /* Modulation options */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_INVERT!(pkt_data.rf_chain), if pkt_data.invert_pol { 1 } else { 0 })?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_IMPLICIT_HEADER!(pkt_data.rf_chain), if pkt_data.no_header { 1 } else { 0 })?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_CRC_EN!(pkt_data.rf_chain), if pkt_data.no_crc { 0 } else{ 1 })?;
                
    
                /* Syncword */
                if (lwan_public == false) || (pkt_data.datarate == DR_LORA_SF5) || (pkt_data.datarate == DR_LORA_SF6) {
                    debug!("Setting LoRa syncword 0x12\n");
                    self.lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS!(pkt_data.rf_chain), 2)?;
                    
                    self.lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS!(pkt_data.rf_chain), 4)?;
                    
                } else {
                    debug!("Setting LoRa syncword 0x34\n");
                    self.lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS!(pkt_data.rf_chain), 6)?;
                    
                    self.lgw_reg_w(SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS!(pkt_data.rf_chain), 8)?;
                    
                }
    
                /* Set Fine Sync for SF5/SF6 */
                if (pkt_data.datarate == DR_LORA_SF5) || (pkt_data.datarate == DR_LORA_SF6) {
                    debug!("Enable Fine Sync\n");
                    self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN!(pkt_data.rf_chain), 1)?;
                    
                } else {
                    debug!("Disable Fine Sync\n");
                    self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN!(pkt_data.rf_chain), 0)?;
                    
                }
    
                /* Set Payload length */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_3_PAYLOAD_LENGTH!(pkt_data.rf_chain), pkt_data.size as i32)?;
                
    
                /* Set PPM offset (low datarate optimization) */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL!(pkt_data.rf_chain), 0)?;
                
                if SET_PPM_ON(pkt_data.bandwidth, pkt_data.datarate) {
                    debug!("Low datarate optimization ENABLED\n");
                    self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET!(pkt_data.rf_chain), 1)?;
                    
                } else {
                    debug!("Low datarate optimization DISABLED\n");
                    self.lgw_reg_w(SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET!(pkt_data.rf_chain), 0)?;
                    
                }
            },
            Modulation::FSK => {
                /* Set frequency deviation */
                freq_dev = pkt_data.f_dev as u32 * 1000u32;
                fdev_reg = SX1302_FREQ_TO_REG!(freq_dev);
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV!(pkt_data.rf_chain), ((fdev_reg >>  8) & 0xFF) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV!(pkt_data.rf_chain), ((fdev_reg >>  0) & 0xFF) as i32)?;
                
    
                /* Send frequency deviation to AGC fw for radio config */
                fdev_reg = SX1250_FREQ_TO_REG!(freq_dev);
                self.lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA, ((fdev_reg >> 16) & 0xFF) as i32)?; /* Needed by AGC to configure the sx1250 */
                
                self.lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA, ((fdev_reg >>  8) & 0xFF) as i32)?; /* Needed by AGC to configure the sx1250 */
                
                self.lgw_reg_w(SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA, ((fdev_reg >>  0) & 0xFF) as i32)?; /* Needed by AGC to configure the sx1250 */
                
    
                /* Modulation parameters */
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_PKT_MODE!(pkt_data.rf_chain), 1)?; /* Variable length */
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_CRC_EN!(pkt_data.rf_chain), if pkt_data.no_crc { 0 } else{ 1 })?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_CRC_IBM!(pkt_data.rf_chain), 0)?; /* CCITT CRC */
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_CFG_0_DCFREE_ENC!(pkt_data.rf_chain), 2)?; /* Whitening Encoding */
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_EN!(pkt_data.rf_chain), 1)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_SELECT_BT!(pkt_data.rf_chain), 2)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_EN!(pkt_data.rf_chain), 1)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_SIZE!(pkt_data.rf_chain), context_fsk.sync_word_size as i32 - 1)?;
                
    
                /* Syncword */
                fsk_sync_word_reg = context_fsk.sync_word << (8 * (8 - context_fsk.sync_word_size));
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 0) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 8) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 16) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 24) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 32) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 40) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 48) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN!(pkt_data.rf_chain), (fsk_sync_word_reg >> 56) as i32)?;
                
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_MOD_FSK_PREAMBLE_SEQ!(pkt_data.rf_chain), 0)?;
                
    
                /* Set datarate */
                fsk_br_reg = 32000000 / pkt_data.datarate;
                buff[0] = (fsk_br_reg >> 8) as u8;
                buff[1] = (fsk_br_reg >> 0) as u8;
                self.lgw_reg_wb(SX1302_REG_TX_TOP_FSK_BIT_RATE_MSB_BIT_RATE!(pkt_data.rf_chain), &buff, 2)?;
                
    
                /* Preamble length */
                if pkt_data.preamble == 0 { /* if not explicit, use LoRaWAN preamble size */
                    pkt_data.preamble = STD_FSK_PREAMBLE as u16;
                } else if pkt_data.preamble < MIN_FSK_PREAMBLE as u16 { /* enforce minimum preamble size */
                    pkt_data.preamble = MIN_FSK_PREAMBLE as u16;
                    debug!("Note: preamble length adjusted to respect minimum FSK preamble size\n");
                }
                buff[0] = (pkt_data.preamble >> 8) as u8;
                buff[1] = (pkt_data.preamble >> 0) as u8;
                self.lgw_reg_wb(SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE!(pkt_data.rf_chain), &buff, 2)?;
                
    
                /* Set Payload length */
                self.lgw_reg_w(SX1302_REG_TX_TOP_FSK_PKT_LEN_PKT_LENGTH!(pkt_data.rf_chain), pkt_data.size as i32)?;
                
            },
            _ => {
                debug!("ERROR: Modulation not supported\n");
                return Err(Error::LGW_REG_ERROR.into());
            }
        }
        /* Set TX start delay */
        let tx_start_delay = self.sx1302_tx_set_start_delay(pkt_data.rf_chain, radio_type, pkt_data.modulation, pkt_data.bandwidth, chirp_lowpass)?;
        
    
        /* Write payload in transmit buffer */
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER!(pkt_data.rf_chain), 0x01)?;
        
        mem_addr = reg_select(pkt_data.rf_chain, 0x5300, 0x5500);
        if pkt_data.modulation == Modulation::FSK {
            buff[0] = pkt_data.size as u8;
            self.lgw_mem_wb(mem_addr, &buff, 1)?; /* insert payload size in the packet for FSK variable mode (1 byte) */
            
            self.lgw_mem_wb(mem_addr+1, &pkt_data.payload, pkt_data.size as usize)?;
            
        } else {
            self.lgw_mem_wb(mem_addr, &pkt_data.payload, pkt_data.size as usize)?;
            
        }
        self.lgw_reg_w(SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER!(pkt_data.rf_chain), 0x00)?;
        
    
        /* Trigger transmit */
        trace!("Start Tx: Freq:{} {}{} size:{} preamb:{}\n", pkt_data.freq_hz, if pkt_data.modulation == Modulation::LORA { "SF" } else { "DR:" }, pkt_data.datarate, pkt_data.size, pkt_data.preamble);
        match pkt_data.tx_mode {
            TxMode::IMMEDIATE => {
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE!(pkt_data.rf_chain), 0x00)?; /* reset state machine */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE!(pkt_data.rf_chain), 0x01)?;
            },
            TxMode::TIMESTAMPED => {
                count_us = (( pkt_data.count_us as u64 * 32 ) - tx_start_delay as u64 ) as u32;
                trace!("--> programming trig delay at {} ({})\n", pkt_data.count_us - (tx_start_delay as u32 / 32), count_us);
                self.lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG!(pkt_data.rf_chain), ((count_us >> 0) & 0x000000FF) as i32)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG!(pkt_data.rf_chain), ((count_us >> 8) & 0x000000FF) as i32)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG!(pkt_data.rf_chain), ((count_us >> 16) & 0x000000FF) as i32)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG!(pkt_data.rf_chain), ((count_us >> 24) & 0x000000FF) as i32)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED!(pkt_data.rf_chain), 0x00)?; /* reset state machine */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED!(pkt_data.rf_chain), 0x01)?;
            },
            TxMode::ON_GPS => {
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS!(pkt_data.rf_chain), 0x00)?; /* reset state machine */
                self.lgw_reg_w(SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS!(pkt_data.rf_chain), 0x01)?;
            }
        }
    
        /* Flush write (USB BULK mode) */
        self.mcu.lgw_flush()?;
        
    
        /* Setting back to SINGLE BULK write mode */
        self.mcu.mcu_set_write_mode(EComWriteMode::LGW_COM_WRITE_MODE_SINGLE);
        
    
        Ok(())
    }
    
    fn timestamp_counter_get( &mut self ) -> Result<(u32,u32)> {
        
        let mut buff = [0u8;8];
        let mut buff_wa=[0u8;8];
       

        /* Get the freerun and pps 32MHz timestamp counters - 8 bytes
                0 -> 3 : PPS counter
                4 -> 7 : Freerun counter (inst)
        */
        if let Err(e) = self.lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS, &mut buff, 8){
            error!("ERROR: Failed to get timestamp counter value: {}", e);
            return Err(anyhow!("LGW_HAL_ERROR"))
        }

        /* Workaround concentrator chip issue:
            - read MSB again
            - if MSB changed, read the full counter again
        */
        if let Err(e) = self.lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS, &mut buff_wa, 8){
      
            error!(e=%e, "ERROR: Failed to get timestamp counter MSB value");
            return Err(anyhow!("LGW_HAL_ERROR"))
        }

        if (buff[0] != buff_wa[0]) || (buff[4] != buff_wa[4]) {
            if let Err(e) = self.lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS, &mut buff_wa, 8){
                error!(e=%e, "ERROR: Failed to get timestamp counter MSB value\n");
                return Err(anyhow!("LGW_HAL_ERROR"))
            }
            buff.copy_from_slice(&buff_wa); /* use the new read value */
        }

        let mut counter_pps_us_raw_27bits_now  = ((buff[0] as u32) << 24) | ((buff[1] as u32) << 16) | ((buff[2] as u32) << 8) | buff[3] as u32;
        let mut counter_inst_us_raw_27bits_now = ((buff[4] as u32) <<24) | ((buff[5] as u32)<<16) | ((buff[6] as u32)<<8) | buff[7] as u32;

        /* Store PPS counter to history, for fine timestamp calculation */
        self.sx1302.timestamp_pps_history.save(counter_pps_us_raw_27bits_now);

        /* Scale to 1MHz */
        counter_pps_us_raw_27bits_now /= 32;
        counter_inst_us_raw_27bits_now /= 32;

        /* Update counter wrapping status */
        self.sx1302.counter_us.update( counter_pps_us_raw_27bits_now, counter_inst_us_raw_27bits_now);

        /* Convert 27-bits counter to 32-bits counter */
        let inst = self.sx1302.counter_us.expand( false, counter_inst_us_raw_27bits_now);
        let pps  = self.sx1302.counter_us.expand( true, counter_pps_us_raw_27bits_now);

        Ok((inst, pps))
    }


    fn rx_buffer_fetch(&mut self) -> Result<()> {

        //let rx_buffer = &mut self.sx1302.rx_buffer;

        let mut data_buff = [0u8;4096];

        let mut buff=[0u8;2];
    
        /* Check if there is data in the FIFO */
        self.lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, &mut buff,  2)?;
        let nb_bytes_1 = ((buff[0] as u16) << 8 )| ((buff[1] as u16) << 0);
    
        /* Workaround for multi-byte read issue: read again and ensure new read is not lower than the previous one */
        self.lgw_reg_rb(SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, &mut buff,  2)?;
        let nb_bytes_2 = ((buff[0] as u16) << 8 )| ((buff[1] as u16) << 0);
    
        let data_size = if nb_bytes_2 > nb_bytes_1 { nb_bytes_2 as usize }  else { nb_bytes_1 as usize };
        trace!(data_size=%data_size, "fifo data");
        /* Fetch bytes from fifo if any */
        if data_size > 0 {
            debug!("-----------------\n");
            debug!("rx_buffer_fetch: nb_bytes to be fetched: {:} ({:} {:})\n", data_size, buff[1], buff[0]);


            if let Err(err) = self.lgw_mem_rb(0x4000, &mut data_buff, data_size, true){

                error!("ERROR: Failed to read RX buffer, SPI error: {}", err);
                return Err(anyhow!("LGW_HAL_ERR"));
            }
    
            /* print debug info */
            debug!("RX_BUFFER: {:02X?}", &data_buff[0 .. data_size].to_vec());

            /* Sanity check: is there at least 1 complete packet in the buffer */
            if data_size < (SX1302_PKT_HEAD_METADATA + SX1302_PKT_TAIL_METADATA) as usize {
                warn!("WARNING: not enough data to have a complete packet, discard rx_buffer\n");
                return Ok(());
            }

             /* Sanity check: is there a syncword at 0 ? If not, move to the first syncword found */
            let mut idx: usize = 0;
            while idx <= (data_size - 2) {
                if (data_buff[idx] == SX1302_PKT_SYNCWORD_BYTE_0) && (data_buff[idx + 1] == SX1302_PKT_SYNCWORD_BYTE_1) {
                    debug!("INFO: syncword found at idx {:}\n", idx);
                    break;
                }
                else{
                    idx += 1;
                }
            }

            if idx > data_size - 2 {
                debug!("WARNING: no syncword found, discard rx_buffer\n");
                return Ok(())
            }

            if idx != 0 {
                debug!("INFO: re-sync rx_buffer at idx {:}\n", idx);
                self.sx1302.rx_buffer.buffer[.. (data_size - idx)].copy_from_slice(&data_buff[ idx .. (data_size - idx)]);
                
                self.sx1302.rx_buffer.buffer_size = data_size - idx;
            }
            else{
                self.sx1302.rx_buffer.buffer[ .. data_size ].copy_from_slice(&data_buff[0 .. data_size]);
                self.sx1302.rx_buffer.buffer_size = data_size;
            }

            /* Rewind and parse buffer to get the number of packet fetched */
            idx = 0;
            while idx < self.sx1302.rx_buffer.buffer_size {
                if self.sx1302.rx_buffer.buffer[idx] != SX1302_PKT_SYNCWORD_BYTE_0 || (self.sx1302.rx_buffer.buffer[idx + 1] != SX1302_PKT_SYNCWORD_BYTE_1) {
                    debug!("WARNING: syncword not found at idx {:}, discard the rx_buffer\n", idx);
                    self.sx1302.rx_buffer.del();
                    return Ok(());
                }

                /* One packet found in the buffer */
                self.sx1302.rx_buffer.buffer_pkt_nb += 1;

                /* Compute the number of bytes for this packet */
                let payload_len = SX1302_PKT_PAYLOAD_LENGTH!(self.sx1302.rx_buffer.buffer, idx) as u8;
                let next_pkt_idx =  SX1302_PKT_HEAD_METADATA +
                                payload_len +
                                SX1302_PKT_TAIL_METADATA +
                                (2 * SX1302_PKT_NUM_TS_METRICS!(self.sx1302.rx_buffer.buffer, idx + payload_len as usize)) as u8;

                /* Move to next packet */
                idx += next_pkt_idx as usize;
            }
        }
            
    
        Ok(())
    }

    fn sx1302_update(&mut self) -> Result<()> {
        
    
        /* Disabled because it brings latency on USB, for low value. TODO: do this less frequently ?
       
        // Check MCUs parity errors 
        let val = self.lgw_reg_r(SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR)?;
        if val != 0 {
            error!("ERROR: Parity error check failed on AGC firmware\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR)?;
        if (val != 0) {
            error!("ERROR: Parity error check failed on ARB firmware\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
        */
    
        /* Update internal timestamp counter wrapping status */
        self.timestamp_counter_get()?;
    
        Ok(())
    }

    fn sx1302_fetch(&mut self)->Result<u8> {
    
        /* Fetch packets from sx1302 if no more left in RX buffer */
        if self.sx1302.rx_buffer.buffer_pkt_nb == 0 {
            /* Initialize RX buffer */
            self.sx1302.rx_buffer.clear();

            /* Fetch RX buffer if any data available */
            if let Err(err) = self.rx_buffer_fetch(){
          
                error!("ERROR: Failed to fetch RX buffer: {}", err);
                return Err(anyhow!("LGW_REG_ERR"));
            }
            else{
                trace!("Note: fetch {:} pkts from sx1302", self.sx1302.rx_buffer.buffer_pkt_nb);
            }
        } else {
            debug!("Note: remaining {:} packets in RX buffer, do not fetch sx1302 yet...\n", self.sx1302.rx_buffer.buffer_pkt_nb);
        }

        /* Return the number of packet fetched */
        
       
        
        let nb_pkt = self.sx1302.rx_buffer.buffer_pkt_nb;
        Ok(nb_pkt)
    }

    fn sx1302_gps_enable(&mut self,  enable: bool) -> Result<()> {
       
        if enable == true {
            self.lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 1)?;
            self.lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_POL, 1)?; /* invert polarity for PPS */
        } else {
            self.lgw_reg_w(SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 0)?;
        }

        Ok(())
    }

    fn sx1302_tx_configure(&mut self,  radio_type:LgwRadioType) -> Result<()> {
       
        /* Select the TX destination interface */
        match radio_type {
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => {
                /* Let AGC control PLL DIV (sx1250 only) */
                self.lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC, 1)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC, 1)?;
    
                /* SX126x Tx RFFE */
                self.lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01)?;
            },
            LgwRadioType::LGW_RADIO_TYPE_SX1255 | LgwRadioType::LGW_RADIO_TYPE_SX1257 => {
                /* SX1255/57 Tx RFFE */
                self.lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x00)?;
                self.lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST, 0x00)?;
            },
            _ => {
                debug!("ERROR: radio type not supported");
                return Err(anyhow!("LGW_REG_ERR"));
            }
        }
    
        /* Configure the TX mode of operation */
        self.lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE, 0x01)?; /* Modulation */
        self.lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE, 0x01)?; /* Modulation */
    
        /* Configure the output data clock edge */
        self.lgw_reg_w(SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x00)?; /* Data on rising edge */
        self.lgw_reg_w(SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE, 0x00)?; /* Data on rising edge */
    
        Ok(())
    }

    fn sx1302_arb_status(&mut self)->Result<u8> {

        let val = self.lgw_reg_r(SX1302_REG_ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS)?;
        Ok(val as u8)
    }

    fn sx1302_arb_wait_status(&mut self,  status: u8) -> Result<()> {
        
        loop {
            let val = self.sx1302_arb_status()?;
            if val == status {
                return Ok(());
            }
            /* TODO: add timeout */
        } 
    }


    fn sx1302_arb_debug_read(&mut self,  reg_id:u8)-> Result<u8> {
        
        /* Check parameters */
        if reg_id > 15 {
            error!("ERROR: invalid ARB debug register ID\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        let reg = SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0 + reg_id as u16;
        let val = self.lgw_reg_r(reg)?;
        Ok(val as u8)
    }

    fn sx1302_arb_set_debug_stats(&mut self,  enable:bool,  sf: u8) -> Result<()> {
        if enable == true {
            debug!("ARB: Debug stats enabled for SF{:}", sf);
            self.lgw_reg_w(SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0, sf as i32)?;
        } else {
            debug!("ARB: Debug stats disabled\n");
        }

        Ok(())
    }

    fn sx1302_arb_debug_write(&mut self,  reg_id: u8,  value:u8) -> Result<()> {
        
        /* Check parameters */
        if reg_id > 3 {
            error!("ERROR: invalid ARB debug register ID");
            return Err(anyhow!("LGW_REG_ERR"));
        }
    
        let reg = SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0 + reg_id as u16;
        self.lgw_reg_w(reg, value as i32)?;
    
        Ok(())
    }

    fn sx1302_arb_start(&mut self, version: u8,  ftime_context: &LgwConfigFtime) -> Result<()> {
        
        /* Wait for ARB fw to be started, and VERSION available in debug registers */
        self.sx1302_arb_wait_status(0x01)?;

        /* Get firmware VERSION */
        let val = self.sx1302_arb_debug_read(0)?;
        if val != version{
            error!("ERROR: wrong ARB fw version ({})\n", val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        debug!("ARB FW VERSION: {:}\n", val);

        /* Enable/disable ARB detect/modem alloc stats for the specified SF */
        self.sx1302_arb_set_debug_stats(true, DR_LORA_SF7 as u8)?;

        /* Enable/Disable double demod for different timing set (best timestamp / best demodulation) - 1 bit per SF (LSB=SF5, MSB=SF12) => 0:Disable 1:Enable */
        if ftime_context.enable == false {
            debug!("ARB: dual demodulation disabled for all SF\n");
            self.sx1302_arb_debug_write(3, 0x00)?; /* double demod disabled for all SF */
        } else {
            if ftime_context.mode == LgwFtimeMode::LGW_FTIME_MODE_ALL_SF {
                debug!("ARB: dual demodulation enabled for all SF\n");
                self.sx1302_arb_debug_write(3, 0xFF)?; /* double demod enabled for all SF */
            } else if ftime_context.mode == LgwFtimeMode::LGW_FTIME_MODE_HIGH_CAPACITY {
                debug!("ARB: dual demodulation enabled for SF5 -> SF10\n");
                self.sx1302_arb_debug_write(3, 0x3F)?; /* double demod enabled for SF10 <- SF5 */
            } else {
                error!("ERROR: fine timestamp mode is not supported ({})\n", ftime_context.mode as u8);
                return Err(anyhow!("LGW_REG_ERR"));
            }
        }

        /* Set double detect packet filtering threshold [0..3] */
        self.sx1302_arb_debug_write(2, 3)?;

        /* Notify ARB that it can resume */
        self.sx1302_arb_debug_write(1, 1)?;

        /* Wait for ARB to acknoledge */
        self.sx1302_arb_wait_status(0x00)?;

        debug!("ARB: started\n");

        Ok(())
    }

    fn sx1302_arb_load_firmware(&mut self,  firmware:&[u8])-> Result<()> {
        let mut fw_check = [0u8;MCU_FW_SIZE];


        /* Take control over ARB MCU */
       self.lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x01)?;
       self.lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x01)?;
       self.lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00)?;

        /* Write ARB fw in ARB MEM */
       self.lgw_mem_wb(ARB_MEM_ADDR, firmware, MCU_FW_SIZE)?;

        /* Read back and check */
       self.lgw_mem_rb(ARB_MEM_ADDR, &mut fw_check, MCU_FW_SIZE, false)?;

        if firmware != fw_check {
            error!("ERROR: ARB fw read/write check failed\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* Release control over ARB MCU */
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x00)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x00)?;

        let val = self.lgw_reg_r(SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR)?;
        if val != 0 {
            error!("ERROR: Failed to load ARB fw: parity error check failed\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("ARB fw loaded\n");

        Ok(())
    }

    fn sx1302_agc_mailbox_write(&mut self,  mailbox:u8,  value:u8) -> Result<()> {
        
        /* Check parameters */
        if mailbox > 3 {
            error!("ERROR: invalid AGC mailbox ID\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
    
        let reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA - mailbox as u16;
        if let Err(_) = self.lgw_reg_w(reg, value as i32) {
            error!("ERROR: failed to write AGC mailbox\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
    
       Ok(())
    }

    fn sx1302_agc_mailbox_read(&mut self, mailbox:u8 ) ->Result<u8> {

    
        /* Check parameters */
        if mailbox > 3 {
            error!("ERROR: invalid AGC mailbox ID");
            return Err(anyhow!("LGW_REG_ERR"));
        }
    
        let reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA - mailbox as u16;
        if let Ok(val) = self.lgw_reg_r(reg)  {
            return Ok(val as u8)
        }
        else{
            error!("ERROR: failed to read AGC mailbox {}", mailbox);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        
    }

    fn sx1302_agc_status(&mut self) -> Result<u8> {
    
        if let Ok(val) = self.lgw_reg_r(SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS){
            Ok(val as u8)
        }
        else{
            Err(anyhow!("LGW_REG_ERR"))
        }

    }

    fn sx1302_agc_wait_status(&mut self, status: u8) -> Result<()>{

        loop {
            if let Ok(val) = self.sx1302_agc_status() {
                if val == status {
                    break;
                }
            }
            else{
                return Err(anyhow!("LGW_HAL_ERR"));
            }
        }

        Ok(())
    }

    fn sx1302_agc_start(&mut self, version: u8,  radio_type:LgwRadioType,  ana_gain: u8,  dec_gain:u8,  full_duplex:bool,  lbt_enable: bool) -> Result<()> {

        
        let fdd_mode = if full_duplex == true { 1 } else {0};

        /* Check parameters */
        if (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1255) && (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1257) && (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1250) {
            error!("ERROR: invalid radio type\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* Wait for AGC fw to be started, and VERSION available in mailbox */
        self.sx1302_agc_wait_status(0x01)?; /* fw has started, VERSION is ready in mailbox */

        let val = self.sx1302_agc_mailbox_read(0)?;

        if val != version {
            error!("ERROR: wrong AGC fw version ({:})", val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC FW VERSION: {:}\n", val);
        
        
        /* Configure Radio A gains */
        self.sx1302_agc_mailbox_write(0, ana_gain)?; /* 0:auto agc*/
        self.sx1302_agc_mailbox_write(1, dec_gain)?;

        if radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1250 {
            error!("AGC: setting fdd_mode to {:}\n", fdd_mode);
            self.sx1302_agc_mailbox_write(2, fdd_mode)?;
        }

        /* notify AGC that gains has been set to mailbox for Radio A */
        self.sx1302_agc_mailbox_write(3, AGC_RADIO_A_INIT_DONE)?;

        /* Wait for AGC to acknoledge it has received gain settings for Radio A */
        self.sx1302_agc_wait_status(0x02)?;

        /* Check ana_gain setting */
        let val = self.sx1302_agc_mailbox_read(0)?;
         
        if val != ana_gain {
            error!("ERROR: Analog gain of Radio A has not been set properly\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
        
        

        /* Check dec_gain setting */
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != dec_gain {
            error!("ERROR: Decimator gain of Radio A has not been set properly\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* Check FDD mode setting */
        let val = self.sx1302_agc_mailbox_read(2)?;
        if val != fdd_mode {
            error!("ERROR: FDD mode of Radio A has not been set properly\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: Radio A config done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure Radio B gains */
        self.sx1302_agc_mailbox_write(0, ana_gain)?; /* 0:auto agc*/
        self.sx1302_agc_mailbox_write(1, dec_gain)?;
        if radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1250 {
            self.sx1302_agc_mailbox_write(2, fdd_mode)?;
        }

        /* notify AGC that gains has been set to mailbox for Radio B */
        self.sx1302_agc_mailbox_write(3, AGC_RADIO_B_INIT_DONE)?;

        /* Wait for AGC to acknoledge it has received gain settings for Radio B */
        self.sx1302_agc_wait_status(0x03)?;

        /* Check ana_gain setting */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != ana_gain {
            error!("ERROR: Analog gain of Radio B has not been set properly\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* Check dec_gain setting */
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != dec_gain {
            error!("ERROR: Decimator gain of Radio B has not been set properly\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* Check FDD mode setting */
        let val = self.sx1302_agc_mailbox_read(2)?;
        if val != fdd_mode {
            error!("ERROR: FDD mode of Radio B has not been set properly\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: Radio B config done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure AGC gains */
        let agc_params =  if radio_type == LgwRadioType::LGW_RADIO_TYPE_SX1250 { AGC_PARAMS_SX1250 } else{ AGC_PARAMS_SX125X };

        /* Configure analog gain min/max */
        self.sx1302_agc_mailbox_write(0, agc_params.ana_min)?;
        self.sx1302_agc_mailbox_write(1, agc_params.ana_max)?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x03)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x04)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != agc_params.ana_min {
            error!("ERROR: wrong ana_min (w:{:} r:{:})\n", agc_params.ana_min, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != agc_params.ana_max {
            error!("ERROR: ana_max (w:{:} r:{:})\n", agc_params.ana_max, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of analog gain min/max done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure analog thresholds */
        self.sx1302_agc_mailbox_write(0, agc_params.ana_thresh_l)?;
        self.sx1302_agc_mailbox_write(1, agc_params.ana_thresh_h)?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x04)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x05)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != agc_params.ana_thresh_l {
            error!("ERROR: wrong ana_thresh_l (w:{:} r:{:})\n", agc_params.ana_thresh_l, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != agc_params.ana_thresh_h {
            error!("ERROR: wrong ana_thresh_h (w:{:} r:{:})\n", agc_params.ana_thresh_h, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of analog threshold done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure decimator attenuation min/max */
        self.sx1302_agc_mailbox_write(0, agc_params.dec_attn_min)?;
        self.sx1302_agc_mailbox_write(1, agc_params.dec_attn_max)?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x05)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x06)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != agc_params.dec_attn_min {
            error!("ERROR: wrong dec_attn_min (w:{:} r:{:})\n", agc_params.dec_attn_min, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != agc_params.dec_attn_max {
            error!("ERROR: wrong dec_attn_max (w:{:} r:{:})\n", agc_params.dec_attn_max, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of decimator atten min/max done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure decimator attenuation thresholds */
        self.sx1302_agc_mailbox_write(0, agc_params.dec_thresh_l)?;
        self.sx1302_agc_mailbox_write(1, agc_params.dec_thresh_h1)?;
        self.sx1302_agc_mailbox_write(2, agc_params.dec_thresh_h2)?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x06)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x07)?;

            /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != agc_params.dec_thresh_l {
            error!("ERROR: wrong dec_thresh_l (w:{:} r:{:})\n", agc_params.dec_thresh_l, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != agc_params.dec_thresh_h1 {
            error!("ERROR: wrong dec_thresh_h1 (w:{:} r:{:})\n", agc_params.dec_thresh_h1, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(2)?;
        if val != agc_params.dec_thresh_h2 {
            error!("ERROR: wrong dec_thresh_h2 (w:{:} r:{:})\n", agc_params.dec_thresh_h2, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of decimator threshold done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure channel attenuation min/max */
        self.sx1302_agc_mailbox_write(0, agc_params.chan_attn_min)?;
        self.sx1302_agc_mailbox_write(1, agc_params.chan_attn_max)?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x07)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x08)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != agc_params.chan_attn_min {
            error!("ERROR: wrong chan_attn_min (w:{:} r:{:})\n", agc_params.chan_attn_min, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != agc_params.chan_attn_max {
            error!("ERROR: wrong chan_attn_max (w:{:} r:{:})\n", agc_params.chan_attn_max, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of channel atten min/max done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure channel attenuation threshold */
        self.sx1302_agc_mailbox_write(0, agc_params.chan_thresh_l)?;
        self.sx1302_agc_mailbox_write(1, agc_params.chan_thresh_h)?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x08)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x09)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != agc_params.chan_thresh_l {
            error!("ERROR: wrong chan_thresh_l (w:{:} r:{:})\n", agc_params.chan_thresh_l, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }
        let val = self.sx1302_agc_mailbox_read(1)?;
        if val != agc_params.chan_thresh_h {
            error!("ERROR: wrong chan_thresh_h (w:{:} r:{:})\n", agc_params.chan_thresh_h, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of channel atten threshold done\n");

        /* -----------------------------------------------------------------------*/

        /* Configure sx1250 SetPAConfig */
        if radio_type == LgwRadioType::LGW_RADIO_TYPE_SX1250 {
            self.sx1302_agc_mailbox_write(0, agc_params.device_sel)?;
            self.sx1302_agc_mailbox_write(1, agc_params.hp_max)?;
            self.sx1302_agc_mailbox_write(2, agc_params.pa_duty_cycle)?;

            /* notify AGC that params have been set to mailbox */
            self.sx1302_agc_mailbox_write(3, 0x09)?;

            /* Wait for AGC to acknoledge it has received params */
            self.sx1302_agc_wait_status(0x0A)?;

            /* Check params */
            let val = self.sx1302_agc_mailbox_read(0)?;
            if val != agc_params.device_sel {
                error!("ERROR: wrong device_sel (w:{:} r:{:})\n", agc_params.device_sel, val);
                return Err(anyhow!("LGW_REG_ERR"));
            }
            let val = self.sx1302_agc_mailbox_read(1)?;
            if val != agc_params.hp_max {
                error!("ERROR: wrong hp_max (w:{:} r:{:})\n", agc_params.hp_max, val);
                return Err(anyhow!("LGW_REG_ERR"));
            }
            let val = self.sx1302_agc_mailbox_read(2)?;
            if val != agc_params.pa_duty_cycle {
                error!("ERROR: wrong pa_duty_cycle (w:{:} r:{:})\n", agc_params.pa_duty_cycle, val);
                return Err(anyhow!("LGW_REG_ERR"));
            }

            info!("AGC: config of sx1250 PA optimal settings done\n");
        }

        /* -----------------------------------------------------------------------*/

        /* Set PA start delay */
        let pa_start_delay = 8;
        self.sx1302_agc_mailbox_write(0, pa_start_delay)?; /* 1 LSB = 100 µs*/

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x0A)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x0B)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if val != pa_start_delay {
            error!("ERROR: wrong PA start delay (w:{:} r:{:})\n", pa_start_delay, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: config of PA start delay done\n");

        /* -----------------------------------------------------------------------*/

        /* Enable LBT if required */
        self.sx1302_agc_mailbox_write(0, if lbt_enable == true  { 1 } else { 0 } )?;

        /* notify AGC that params have been set to mailbox */
        self.sx1302_agc_mailbox_write(3, 0x0B)?;

        /* Wait for AGC to acknoledge it has received params */
        self.sx1302_agc_wait_status(0x0F)?;

        /* Check params */
        let val = self.sx1302_agc_mailbox_read(0)?;
        if (val != 0) != lbt_enable {
            error!("ERROR: wrong LBT configuration (w:{:} r:{:})\n", lbt_enable, val);
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC: LBT is {:}",  if lbt_enable == true { "enabled" } else { "disabled"} );

        /* -----------------------------------------------------------------------*/

        /* notify AGC that configuration is finished */
        self.sx1302_agc_mailbox_write(3, 0x0F)?;

        info!("AGC: started");

        Ok(())
    }

    fn sx1302_modem_enable(&mut self)->Result<()> {
       
        /* Enable LoRa multi-SF modems */
        self.lgw_reg_w(SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE, 0x01)?;
    
        /* Enable LoRa service modem */
        self.lgw_reg_w(SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE, 0x01)?;
    
        /* Enable FSK modem */
        self.lgw_reg_w(SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01)?;
    
        /* Enable RX */
        self.lgw_reg_w(SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01)?;
    
        Ok(())
    }

    fn sx1302_lora_syncword(&mut self,  public: bool,  lora_service_sf: u8) -> Result<()> {
       
        /* Multi-SF modem configuration */
        debug!("INFO: configuring LoRa (Multi-SF) SF5->SF6 with syncword PRIVATE (0x12)\n");
        self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF5_PEAK1_POS_SF5, 2)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF5_PEAK2_POS_SF5, 4)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF6_PEAK1_POS_SF6, 2)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF6_PEAK2_POS_SF6, 4)?;
        if public == true {
            debug!("INFO: configuring LoRa (Multi-SF) SF7->SF12 with syncword PUBLIC (0x34)\n");
            self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 6)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 8)?;
        } else {
            debug!("INFO: configuring LoRa (Multi-SF) SF7->SF12 with syncword PRIVATE (0x12)\n");
            self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 2)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 4)?;
        }
    
        /* LoRa Service modem configuration */
        if (public == false) || (lora_service_sf == DR_LORA_SF5 as u8) || (lora_service_sf == DR_LORA_SF6 as u8) {
            info!("INFO: configuring LoRa (Service) SF{:} with syncword PRIVATE (0x12)\n", lora_service_sf);
            self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 2)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 4)?;
        } else {
            info!("INFO: configuring LoRa (Service) SF{:} with syncword PUBLIC (0x34)\n", lora_service_sf);
            self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 6)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 8)?;
        }
    
        Ok(())
    }

    fn sx1302_fsk_configure(&mut self, cfg: &LgwConfRxIf) -> Result<()> {


        debug!("FSK: syncword:0x{:02X?}, syncword_size:{:}", cfg.sync_word, cfg.sync_word_size);

        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_PSIZE, cfg.sync_word_size as i32 - 1)?;
        let fsk_sync_word_reg = cfg.sync_word << (8 * (8 - cfg.sync_word_size));
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN, (fsk_sync_word_reg >> 0) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN, (fsk_sync_word_reg >> 8) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN, (fsk_sync_word_reg >> 16) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN, (fsk_sync_word_reg >> 24) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN, (fsk_sync_word_reg >> 32) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN, (fsk_sync_word_reg >> 40) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN, (fsk_sync_word_reg >> 48) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN, (fsk_sync_word_reg >> 56) as i32)?;

        let fsk_br_reg = 32000000 / cfg.datarate;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_MSB_BIT_RATE, (fsk_br_reg >> 8) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_LSB_BIT_RATE, (fsk_br_reg >> 0) as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_CH_BW_EXPO, 0x03)?; /* 125KHz */

        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RX_INVERT, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_MODEM_INVERT_IQ, 0)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_RSSI_LENGTH, 4)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_PKT_MODE, 1)?; /* variable length */
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_EN, 1)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_DCFREE_ENC, 2)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_IBM, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_ERROR_OSR_TOL, 10)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_PKT_LENGTH_PKT_LENGTH, 255)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_NODE_ADRS_NODE_ADRS, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_BROADCAST_BROADCAST, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_AUTO_AFC, 1)?; /* ?? */
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_MSB_TIMEOUT, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_LSB_TIMEOUT, 128)?;

        Ok(())
    }


    fn sx1302_lora_service_modem_configure(&mut self, cfg: &LgwConfRxIf, radio_freq_hz: u32) -> Result<()> {

        let preamble_nb_symb: u8;

    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP, 0x01)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL, 0x01)?;
    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_AUTO, 0x03)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PAYLOAD, 0x03)?;
    
        match cfg.datarate {
            DR_LORA_SF5 | DR_LORA_SF6 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x04)?; // Default value
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x00)?; // Default value
            },
            DR_LORA_SF7 | DR_LORA_SF8 | DR_LORA_SF9 | DR_LORA_SF10 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x06)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x00)?;
            },
            DR_LORA_SF11 | DR_LORA_SF12 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB, 0x07)?;
                match cfg.bandwidth {
                    BW_125KHZ => {
                        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x01)?;
                    },
                    BW_250KHZ => {
                        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x02)?;
                    },
                    BW_500KHZ => {
                        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN, 0x03)?;
                    },
                    _ => {
                        println!("ERROR: unsupported bandwidth {} for LoRa Service modem", cfg.bandwidth);
                    }
                }
            },
            _ => {
                println!("ERROR: unsupported datarate {} for LoRa Service modem", cfg.datarate);
            }
        }
    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_IMPLICIT_HEADER, if cfg.implicit_hdr == true { 1 } else { 0 })?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CRC_EN, if cfg.implicit_crc_en == true { 1 } else { 0 })?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE, cfg.implicit_coderate as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG3_PAYLOAD_LENGTH, cfg.implicit_payload_length as i32)?;
    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF, cfg.datarate as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW, cfg.bandwidth as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET, if SET_PPM_ON(cfg.bandwidth, cfg.datarate) { 1 } else { 0 })?;
    
        /* Set preamble size to 8 for SF7->SF12 and to 12 for SF5->SF6 (aligned with end-device drivers) */
        if (cfg.datarate == DR_LORA_SF5) || (cfg.datarate == DR_LORA_SF6) {
            preamble_nb_symb = 12;
        } else {
            preamble_nb_symb = 8;
        }
        info!("INFO: LoRa Service modem: configuring preamble size to {} symbols\n", preamble_nb_symb);
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG7_PREAMBLE_SYMB_NB, (preamble_nb_symb as i32 >> 8) & 0xFF)?; /* MSB */
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG6_PREAMBLE_SYMB_NB, (preamble_nb_symb as i32 >> 0) & 0xFF)?; /* LSB */
    
        /* Freq2TimeDrift computation */
        let draft = calculate_freq_to_time_drift(radio_freq_hz, cfg.bandwidth);
        if draft.is_err() {
            error!("ERROR: failed to calculate frequency to time drift for LoRa service modem\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        let (mantissa, exponent ) = draft.unwrap();
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT, (mantissa as i32 >> 8) & 0x00FF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT, (mantissa as i32) & 0x00FF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent as i32)?;
        debug!("Freq2TimeDrift SingleSF: Mantissa = {} (0x{:02X}), 0x{:02X}), Exponent = {} (0x{:02X})\n", mantissa, (mantissa >> 8) & 0x00FF, (mantissa) & 0x00FF, exponent, exponent);
    
        /* Time drift compensation */
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1)?;
    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_IN_COMP, 1)?;
    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN, 1)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX, 1)?;
    
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START, 1)?;
    
        /* DFT peak mode : set to AUTO, check timestamp_counter_correction() if changed */
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_DFT_PEAK_EN, RX_DFT_PEAK_MODE_AUTO as i32)?;
    
        Ok(())
    }

    fn sx1302_lora_service_correlator_configure(&mut self,  cfg: &LgwConfRxIf) -> Result<()> {
    
        /* Common config for all SF */
       self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7)?;
       self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5)?;
       self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_USE_GAIN_SYMB, 1)?;
    
        match cfg.datarate {
            5 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            6 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 1)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            7 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            8 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            9 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            10 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            11 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            12 => {
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN, 0)?;
                self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR, 52)?;
            },
            _ => {
                error!("ERROR: Failed to configure LoRa service modem correlators");
                return Err(anyhow!("LGW_REG_ERR"));
            }
        }
    
        Ok(())
    }

    fn sx1302_lora_modem_configure(&mut self,  radio_freq_hz:u32)-> Result<()> {

    
        self.lgw_reg_w(SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR, 0x01)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_GAIN_DROP_COMP, 0x01)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_DAGC_CFG_TARGET_LVL, 0x01)?;
    
        /* Enable full modems */
        debug!("Configuring 8 full-SF modems\n");
        self.lgw_reg_w(SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0xFF)?;
    
        /* Enable limited modems */
        debug!("Configuring 8 limited-SF modems\n");
        self.lgw_reg_w(SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0xFF)?;
    
        /* Configure coarse sync between correlators and modems */
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA, 126)?;
    
        /* Configure fine sync offset for each channel */
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_0_OFFSET, 1)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_1_OFFSET, 5)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_2_OFFSET, 9)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_3_OFFSET, 13)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_4_OFFSET, 1)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_5_OFFSET, 5)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_6_OFFSET, 9)?;
        self.lgw_reg_w(SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_7_OFFSET, 13)?;
    
        /* Configure PPM offset */
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10, 0x00)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11, 0x01)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12, 0x01)?;
    
        /* Improve SF5 and SF6 performances */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_AUTO, 3)?; // Default is 1
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_PAYLOAD, 3)?; // Default is 2
    
        /* Improve SF11/SF12 performances */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF11, 1)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF12, 1)?;
    
        /* Set threshold for 1bin correction (CAN-314) */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK4_FREQ_SYNCH_THR, 15)?;
    
        /* Configure modems for best tracking (best demodulation) */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF5, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF6, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF7, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF8, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF9, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF10, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF11, RX_FREQ_TRACK_AUTO)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF12, RX_FREQ_TRACK_AUTO)?;
    
        /* Configure modems for best timestamping (only valid when double demodulation is enabled) */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF5, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF6, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF7, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF8, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF9, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF10, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF11, RX_FREQ_TRACK_OFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF12, RX_FREQ_TRACK_OFF)?;
        /* -- */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF11, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF12, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_0_ROUNDING, 1)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_0_MODE, RX_FINE_TIMING_MODE_LINEAR)?;
        /* -- */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_AUTO, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_PREAMB, 6)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_PAYLOAD, 2)?;
        /* -- */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_AUTO, 0)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_PREAMB, 1)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_PAYLOAD, 0)?;
    
        /* Set preamble size to 10 (to handle 12 for SF5/SF6 and 8 for SF7->SF12) */
        self.lgw_reg_w(SX1302_REG_RX_TOP_TXRX_CFG7_PREAMBLE_SYMB_NB,  0)?; /* MSB */
        self.lgw_reg_w(SX1302_REG_RX_TOP_TXRX_CFG6_PREAMBLE_SYMB_NB, 10)?; /* LSB */
    
        /* Freq2TimeDrift computation */
        let draft = calculate_freq_to_time_drift(radio_freq_hz, BW_125KHZ);
        if draft.is_err() {
            error!("ERROR: failed to calculate frequency to time drift for LoRa modem\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        let ( mantissa, exponent) = draft.unwrap();

        debug!("Freq2TimeDrift MultiSF: Mantissa = {} (0x{:02X}, 0x{:02X}), Exponent = {} (0x{:02X})\n", mantissa, (mantissa >> 8) & 0x00FF, (mantissa) & 0x00FF, exponent, exponent);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT, (mantissa as i32 >> 8) & 0x00FF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT, (mantissa as i32) & 0x00FF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent as i32)?;
    
        /* Time drift compensation */
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1)?;
    
        /* DFT peak mode : set to AUTO, check timestamp_counter_correction() if changed */
        self.lgw_reg_w(SX1302_REG_RX_TOP_RX_CFG0_DFT_PEAK_EN, RX_DFT_PEAK_MODE_AUTO as i32)?;
    
        Ok(())
    }

    fn sx1302_lora_correlator_configure(&mut self,  if_cfg: &[LgwConfRxIf],  demod_cfg:&LgwConfDemod) -> Result<()> {
        let mut channels_mask:u8 = 0x00;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF5_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 52)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 24)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 7)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 5)?;

        self.lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE, 0xFF)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR, 0xFF)?;

        /* Enabled selected spreading factors */
        self.lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN, demod_cfg.multisf_datarate as i32)?;
        debug!("INFO: LoRa multi-SF correlator SF enable mask: 0x{:02X}", demod_cfg.multisf_datarate);

        /* Enable correlator if channel is enabled (1 correlator per channel) */
        for i in 0 .. LGW_MULTI_NB as usize {
            channels_mask |= if if_cfg[i].enable { 0x011 << i as u8 } else { 0x00 };
        }
        debug!("INFO: LoRa multi-SF channel enable mask: 0x{:02X}\n", channels_mask);
        self.lgw_reg_w(SX1302_REG_RX_TOP_CORR_CLOCK_ENABLE_CLK_EN, channels_mask as i32)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN, channels_mask as i32)?;

        /* For debug: get packets with sync_error and header_error in FIFO */
    
        //self.lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_STORE_SYNC_FAIL_META, 0x01);
        //self.lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_STORE_HEADER_ERR_META, 0x01);
    

        Ok(())
    }

    fn sx1302_channelizer_configure(&mut self, if_cfg: &[LgwConfRxIf],  fix_gain: bool)-> Result<()> {
        let mut if_freq:i32;
        let mut channels_mask: u8 = 0x00;


        /* Select which radio is connected to each multi-SF channel */
        for i in 0 .. LGW_MULTI_NB as usize {
            channels_mask |= if_cfg[i].rf_chain << i;
        }

        debug!("LoRa multi-SF radio select: 0x{:02X}\n", channels_mask);
        self.lgw_reg_w(SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT, channels_mask as i32)?;

        /* Select which radio is connected to the LoRa service channel */
        debug!("LoRa service radio select: 0x{:}\n", if_cfg[8].rf_chain);
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT, if_cfg[8].rf_chain as i32)?;

        /* Select which radio is connected to the FSK channel */
        debug!("FSK radio select {:}\n", if_cfg[9].rf_chain);
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RADIO_SELECT, if_cfg[9].rf_chain as i32)?;

        /* Configure multi-SF channels IF frequencies */
        if_freq = IF_HZ_TO_REG(if_cfg[0].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[1].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[2].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[3].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[4].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[5].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[6].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6, (if_freq >> 0) & 0x000000FF)?;

        if_freq = IF_HZ_TO_REG(if_cfg[7].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7, (if_freq >> 0) & 0x000000FF)?;

        /* Configure LoRa service channel IF frequency */
        if_freq = IF_HZ_TO_REG(if_cfg[8].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF)?;

        /* Configure FSK channel IF frequency */
        if_freq = IF_HZ_TO_REG(if_cfg[9].freq_hz);
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_MSB_IF_FREQ_0, (if_freq >> 8) & 0x0000001F)?;
        self.lgw_reg_w(SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_LSB_IF_FREQ_0, (if_freq >> 0) & 0x000000FF)?;

        /* Set the low pass filtering corner frequency for RSSI indicator */
        self.lgw_reg_w(SX1302_REG_RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA, 0x05)?;

        /* Set the channelizer RSSI reset value */
        self.lgw_reg_w(SX1302_REG_RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE, 85)?;

        /* Force channelizer in fix gain, or let it be controlled by AGC */
        if fix_gain == true {
            self.lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x00)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN, 5)?;
        } else {
            /* Allow the AGC to control gains */
            self.lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE, 0x01)?;
            /* Disable the internal DAGC */
            self.lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG1_CHAN_DAGC_THRESHOLD_HIGH, 255 )?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG2_CHAN_DAGC_THRESHOLD_LOW, 0 )?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MAX_ATTEN, 15 )?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MIN_ATTEN, 0 )?;
        }

        Ok(())
    }

    fn sx1302_radio_fe_configure(&mut self) -> Result<()> {
        
    
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA, 0x03)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA, 0x07)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_B_RSSI_BB_FILTER_ALPHA, 0x03)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_B_RSSI_DEC_FILTER_ALPHA, 0x07)?;
    
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE, 23)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE, 66)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_B_RSSI_DB_DEFAULT_VALUE, 23)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_B_RSSI_DEC_DEFAULT_VALUE, 66)?;
    
        self.lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN, 1)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN, 0x0b)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN, 1)?;
        self.lgw_reg_w(SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN, 0x0b)?;
    
        Ok(())
    }

    fn sx1302_pa_lna_lut_configure(&mut self, context_board: &LgwConfigBoard) -> Result<()> {
        
        /* Configure LUT Table A */
        if context_board.full_duplex == true {
            self.lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT, 0x0C)?;     /* Enable PA: RADIO_CTRL[2] is high when PA_EN=1 */
            self.lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT, 0x0F)?;    /* Enable LNA: RADIO_CTRL[1] is always high */
        } else {
            self.lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT, 0x04)?;     /* Enable PA: RADIO_CTRL[2] is high when PA_EN=1 */
            self.lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT, 0x02)?;    /* Enable LNA: RADIO_CTRL[1] is high when PA_EN=0 & LNA_EN=1 */
        }

        /* Configure LUT Table B */
        self.lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_B_PA_LUT, 0x04)?;         /* Enable PA: RADIO_CTRL[8] is high when PA_EN=1 & LNA_EN=0 */
        self.lgw_reg_w(SX1302_REG_AGC_MCU_LUT_TABLE_B_LNA_LUT, 0x02)?;        /* Enable LNA: RADIO_CTRL[7] is high when PA_EN=0 & LNA_EN=1 */

        Ok(())
    }

    fn sx1302_get_model_id(&mut self) -> Result<u8> {
 
    
        /* Select ChipModelID */
        self.lgw_reg_w(SX1302_REG_OTP_BYTE_ADDR_ADDR, 0xD0)?;
        
    
        /* Read Modem ID */
        let val = self.lgw_reg_r(SX1302_REG_OTP_RD_DATA_RD_DATA)?;

        Ok(val as u8)
    }

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

    fn sx1302_config_gpio(&mut self) -> Result<()> {

        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, GPIO_CFG_REGISTER as i32)?; /* GPIO_0 => CONFIG_DONE */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, GPIO_CFG_REGISTER as i32)?; /* GPIO_1 => UNUSED */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, GPIO_CFG_STATUS as i32)?;   /* GPIO_2 => Tx ON */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, GPIO_CFG_REGISTER as i32)?; /* GPIO_3 => UNUSED */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, GPIO_CFG_STATUS as i32)?;   /* GPIO_4 => RX ON (PKT_RECEIVE_TOGGLE_OUT) */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, GPIO_CFG_REGISTER as i32)?; /* GPIO_5 => UNUSED */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, GPIO_CFG_REGISTER as i32)?; /* GPIO_6 => UNUSED */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, GPIO_CFG_AGC as i32)?;      /* GPIO_7 => USED FOR LBT (MUST BE INPUT) */
       
        self.lgw_reg_w(SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION, 0x7F)?;              /* GPIO output direction (0 for input and 1 for output) */
       

        Ok(())
    }

    fn sx1302_init(&mut self, ftime_context: &LgwConfigFtime) -> Result<()> {
        
        /* Initialize internal counter */
        //let  count_us = TimestampCounter::new();

        /* Initialize RX buffer */
        //rx_buffer_new(&rx_buffer);

        /* Configure timestamping mode */
        if ftime_context.enable == true {
            if let Ok(model_id) = self.sx1302_get_model_id(){
                if model_id != CHIP_MODEL_ID_SX1303 {
                    error!("ERROR: Fine Timestamping is not supported on this Chip Model ID 0x{:02X}\n", model_id);
                    return Err(anyhow!("LGW_REG_ERR"));
                }
                
            }
            else{
                error!("ERROR: failed to get Chip Model ID\n");
                return Err(anyhow!("LGW_REG_ERR"));
            }

            
        }
        if let Err(_) = self.timestamp_counter_mode(ftime_context.enable){

            error!("ERROR: failed to configure timestamp counter mode\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        if let Err(_) = self.sx1302_config_gpio() {
            error!("ERROR: failed to configure sx1302 GPIOs\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }
        
        Ok(())
    }


    fn sx1302_radio_host_ctrl(&mut self, host_ctrl:bool) -> Result<()> {
         self.lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, if host_ctrl == false  { 0x00 } else { 0x01 })
    }

    fn sx1302_set_gpio(&mut self, gpio_reg_val:u8) -> Result<()> {
        trace!("sx1302_set_gpio {:}", gpio_reg_val);
        if let Err(e) = self.lgw_reg_w(SX1302_REG_GPIO_GPIO_OUT_L_OUT_VALUE as u16, gpio_reg_val as i32) 
        {
            error!("Failed to set gpio: {:?}", e);
            return Err(e)
        }

        Ok(())
    }
    
    fn sx1302_radio_calibrate(&mut self, context_rf_chain: &[LgwConfRxrf],  clksrc:u8, txgain_lut: &[LgwTxGainLut])->Result<()> 
    {
        /* -- Reset radios */
        for i in 0 .. LGW_RF_CHAIN_NB {
            if context_rf_chain[i as usize].enable == true {
                if let Err(e)= self.sx1302_radio_reset(i as u8, context_rf_chain[i as usize]._type.clone()) {
                    error!("Failed to reset radio: {:?}", e);
                    return Err(e)
                }
                if let Err(e) = self.sx1302_radio_set_mode(i as u8, context_rf_chain[i as usize]._type.clone()) 
                {
                    error!("Failed to set radio mode: {:?}", e);
                    return Err(e)
                }
            }
        }
        /* -- Select the radio which provides the clock to the sx1302 */
        if let Err(e) = self.sx1302_radio_clock_select(clksrc) {
            error!("Failed to select clock: {:?}", e);
            return Err(e)
        }

        /* -- Ensure PA/LNA are disabled */
        self.lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 1)?;
        self.lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_PA_EN, 0)?;
        self.lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN, 0)?;

        /* -- Start calibration */
        if (context_rf_chain[clksrc as usize]._type == LgwRadioType::LGW_RADIO_TYPE_SX1257) ||
            (context_rf_chain[clksrc as usize]._type == LgwRadioType::LGW_RADIO_TYPE_SX1255) {
            debug!("Loading CAL fw for sx125x");
            if let Err(e) = self.sx1302_agc_load_firmware(&CAL_FIRMWARE_SX125X) {
                error!("Failed to load calibration fw: {:?}", e);
                return Err(e)
            }
          
            if let Err(e) = self.sx1302_cal_start(FW_VERSION_CAL, context_rf_chain, txgain_lut) {
                error!("Failed to start calibration: {:?}", e);
                self.sx1302_radio_reset(0, context_rf_chain[0]._type.clone())?;
                self.sx1302_radio_reset(1, context_rf_chain[1]._type.clone())?;
                return Err(e)
            }
            
        } else {
            debug!("Calibrating sx1250 radios\n");
            for i in 0 .. LGW_RF_CHAIN_NB  {
                if context_rf_chain[i as usize].enable {
                    if let Err(err) = self.sx1250_calibrate(i as u8, context_rf_chain[i as usize].freq_hz) {
                        error!("Failed to calibrate radio: {:?}", err);
                        return Err(err)
                    }
                }
            }
        }
        /* -- Release control over FE */
        self.lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 0)?;

        debug!("sx1302_radio_calibrate done");
        Ok(())
    }
    
    fn sx1302_radio_reset(&mut self, rf_chain:u8,  radio_type:LgwRadioType) -> Result<()> {
        let reg_radio_en:u16;
        let reg_radio_rst:u16;
    
        debug!(rf_chain=%rf_chain, radio_type=%radio_type, "sx1302_radio_reset:");
        /* Check input parameters */
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("Invalid RF chain");
            return Err(anyhow!("Invalid RF chain"));
        }
        
        if (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1255) && (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1257) && (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1250) {
            error!("ERROR: invalid radio type\n");
            return Err(anyhow!("Invalid radio type"));
        }

        /* Switch to SPI clock before reseting the radio */
        if let Err(e) = self.lgw_reg_w(SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x00){
            error!("ERROR: failed to switch to SPI clock");
            return Err(e);
        }

        /* Enable the radio */
        reg_radio_en = reg_select(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN, SX1302_REG_AGC_MCU_RF_EN_B_RADIO_EN);
        self.lgw_reg_w(reg_radio_en, 0x01)?;

        /* Select the proper reset sequence depending on the radio type */
        reg_radio_rst = reg_select(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST, SX1302_REG_AGC_MCU_RF_EN_B_RADIO_RST);
        self.lgw_reg_w(reg_radio_rst, 0x01)?;

        wait_ms(50); /* wait for the reset to be effective */
        self.lgw_reg_w(reg_radio_rst, 0x00)?;

        wait_ms(10); /* wait for the radio to boot */
        
        match radio_type {
            LgwRadioType::LGW_RADIO_TYPE_SX1255 | LgwRadioType::LGW_RADIO_TYPE_SX1257 => {
                /* Do nothing */
                debug!("INFO: reset sx125x (RADIO_{}) done", reg_select(rf_chain, 0, 1));
            },
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => {
                self.lgw_reg_w(reg_radio_rst, 0x01)?;
                wait_ms(10); /* wait for auto calibration to complete */
                debug!("INFO: reset sx1250 (RADIO_{}) done", reg_select(rf_chain, 0, 1));
            },
            _ => {
                error!("Invalid radio type");
                return Err(anyhow!("Invalid radio type"));
            }
        }
        info!("INFO: radio {} reset done", rf_chain);
        Ok(())
    }

    
    fn sx1302_radio_set_mode(&mut self, rf_chain:u8,  radio_type:LgwRadioType) -> Result<()> {
        let reg:u16;
        

        /* Check input parameters */
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: invalid RF chain\n");
            return Err(anyhow!("invalid RF chain"))
        }
        if (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1255) && (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1257) && (radio_type != LgwRadioType::LGW_RADIO_TYPE_SX1250) {
            error!("ERROR: invalid radio type\n");
            return Err(anyhow!("invalid radio type"));
        }

        /* Set the radio mode */
        reg = reg_select(rf_chain,  SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A,
                                    SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B);
        match radio_type {
            LgwRadioType::LGW_RADIO_TYPE_SX1250 => {
                debug!("Setting rf_chain_{:} in sx1250 mode\n", rf_chain);
                if let Err(e) = self.lgw_reg_w(reg, 0x01) {
                    error!("Error: Failed to set mode for radio: {:}", rf_chain);
                    return Err(e);
                }
            }
                
            _ => {
                debug!("Setting rf_chain_{:} in sx125x mode\n", rf_chain);
                if let Err(e) = self.lgw_reg_w(reg, 0x00) {
                    error!("Error: Failed to set mode for radio: {:}", rf_chain);
                    return Err(e);
                }
            }
                
        }
        Ok(())
    }
    
    fn sx1302_radio_clock_select(&mut self, rf_chain:u8) -> Result<()> {
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: invalid RF chain\n");
            return Err(anyhow!("invalid RF chain"));
        }
        

        /* Switch SX1302 clock from SPI clock to radio clock of the selected RF chain */
        match rf_chain {
            0 => {
                debug!("Select Radio A clock");
                self.lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x01)?;
                self.lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x00)?;
            },
            1 => {
                debug!("Select Radio B clock");
                self.lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL, 0x00)?;
                self.lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL, 0x01)?;
            },
            _ => {
                return Err(anyhow!("Invalid RF chain"));
            }
        }

        /* Enable clock dividers */
        let r1 = self.lgw_reg_w(SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01);

        /* Set the RIF clock to the 32MHz clock of the radio */
        let r2 = self.lgw_reg_w(SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x01);

        /* Check if something went wrong */
        if r1.is_err() || r2.is_err() {
            error!("ERROR: failed to select radio clock for radio_{:}", rf_chain);
            return Err(anyhow!("failed to select radio clock for radio_{:}", rf_chain));
        }

        Ok(())
    }
    
    fn sx1302_agc_load_firmware(&mut self, firmware: &[u8]) -> Result<()> {
    
        debug!("sx1302_agc_load_firmware start");

        let mut fw_check = [0u8;MCU_FW_SIZE];
        

        /* Take control over AGC MCU */
        self.lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x01)?;
        self.lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x01)?;
        self.lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0x00)?;

        /* Write AGC fw in AGC MEM */
        self.lgw_mem_wb(AGC_MEM_ADDR, firmware, MCU_FW_SIZE)?;

        wait_ms(1);
        /* Read back and check */
        self.lgw_mem_rb(AGC_MEM_ADDR, &mut fw_check, MCU_FW_SIZE, false)?;
        
        if firmware != fw_check {
            error!("ERROR: AGC fw read/write check failed\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* Release control over AGC MCU */
        self.lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x00)?;
        self.lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x00)?;

        let val = self.lgw_reg_r(SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR)?;
        if val != 0 {
            error!("ERROR: Failed to load AGC fw: parity error check failed\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        info!("AGC fw loaded\n");

        Ok(())
    }
    
    fn sx1302_cal_start(&mut self, _fw_version:u8, _context_rf_chain: &[LgwConfRxrf], _txgain_lut: &[LgwTxGainLut])->Result<()> {
        todo!()
    }
    
    fn sx1302_get_ifmod_config(if_chain:u8)-> u8 {
        debug!(if_chain=%if_chain, "{:02X}", IFMOD_CONFIG[if_chain as usize]);
        IFMOD_CONFIG[if_chain as usize]
    }




    fn sx1302_parse(&mut self) -> Result<LgwPktRx> {
        
        let context = &self.ctx;
        let rx_buffer = &mut self.sx1302.rx_buffer;
        let timestamp_correction = 0;
        
        /* get packet from RX buffer */
        let pkt = rx_buffer.pop(); 
        
        if pkt.is_err() {
            rx_buffer.del();
            return Err(anyhow!("LGW_HAL_ERR"))
        }

        let pkt = pkt.unwrap();
        let mut p = LgwPktRx::new();
        
        /* copy payload to result struct */
        
        p.payload[..pkt.rxbytenb_modem as usize].copy_from_slice(&pkt.payload[..pkt.rxbytenb_modem as usize]);
        p.size = pkt.rxbytenb_modem as u16;
    
        /* process metadata */
        p.modem_id = pkt.modem_id;
        p.if_chain = pkt.rx_channel_in;
        if p.if_chain >= LGW_IF_CHAIN_NB {
            warn!("WARNING: {:} NOT A VALID IF_CHAIN NUMBER, ABORTING\n", p.if_chain);
            return Err(anyhow!("LGW_REG_ERROR"));
        }
        
        let ifmod = IFMOD_CONFIG[p.if_chain as usize];
        debug!("[{:} 0x{:02X}]\n", p.if_chain, ifmod);
    
        p.rf_chain = context.if_chain_cfg[p.if_chain as usize].rf_chain;
    
        /* Get the frequency for the channel configuration */
        p.freq_hz = (context.rf_chain_cfg[p.rf_chain as usize].freq_hz as i32 + context.if_chain_cfg[p.if_chain as usize].freq_hz) as u32;
    
        /* Get signal strength : offset and temperature compensation will be applied later */
        p.rssic = (pkt.rssi_chan_avg) as f32;
        p.rssis = (pkt.rssi_signal_avg) as f32;
    
        /* Get modulation metadata */
        if (ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD) {
            debug!("Note: LoRa packet (modem {:} chan {:})\n", p.modem_id, p.if_chain);
            p.modulation = Modulation::LORA;
    
            /* Get CRC status */
            if pkt.crc_en || ((ifmod == IF_LORA_STD) && (context.lora_service_cfg.implicit_crc_en == true)) {
                /* CRC enabled */
                if pkt.payload_crc_error {
                    p.status = STAT_CRC_BAD;
                } else {
                    p.status = STAT_CRC_OK;
    
                    /* Sanity check of the payload CRC */
                    if p.size > 0 {
                        let payload_crc16_calc = sx1302_lora_payload_crc(&p.payload, p.size as usize);
                        if payload_crc16_calc != pkt.rx_crc16_value {
                            error!("ERROR: Payload CRC16 check failed (got:0x{:04X} calc:0x{:04X})\n", pkt.rx_crc16_value, payload_crc16_calc);
                            
                            return Err(anyhow!("LGW_REG_ERROR"));
                        } else {
                            trace!("Payload CRC check OK (0x{:04X})\n", pkt.rx_crc16_value);
                        }
                    }
                }
            } else {
                /* CRC disabled */
                p.status = STAT_NO_CRC;
            }
    
    
            /* Get SNR - converted from 0.25dB step to dB */
            p.snr = (pkt.snr_average) as f32 / 4f32;
    
            /* Get bandwidth */
            if ifmod == IF_LORA_MULTI {
                p.bandwidth = BW_125KHZ as u8; /* fixed in hardware */
            } else {
                p.bandwidth = context.lora_service_cfg.bandwidth as u8; /* get the parameter from the config variable */
            }
    
            /* Get datarate */
            match pkt.rx_rate_sf {
                5 => p.datarate = DR_LORA_SF5,
                6 => p.datarate = DR_LORA_SF6,
                7 => p.datarate = DR_LORA_SF7,
                8 => p.datarate = DR_LORA_SF8,
                9 => p.datarate = DR_LORA_SF9,
                10 => p.datarate = DR_LORA_SF10,
                11 => p.datarate = DR_LORA_SF11,
                12 => p.datarate = DR_LORA_SF12,
                _ => p.datarate = DR_UNDEFINED,
            }
    
            let cr ;
            /* Get coding rate */
            if (ifmod == IF_LORA_MULTI) || (context.lora_service_cfg.implicit_hdr == false) {
                cr = pkt.coding_rate;
            } else {
                cr = context.lora_service_cfg.implicit_coderate;
            }
            match cr {
                1 => p.coderate = CR_LORA_4_5,
                2 => p.coderate = CR_LORA_4_6,
                3 => p.coderate = CR_LORA_4_7,
                4 => p.coderate = CR_LORA_4_8,
                _ => p.coderate = CR_UNDEFINED,
            }
    
            /* Get frequency offset in Hz depending on bandwidth */
            match p.bandwidth {
                BW_125KHZ => {
                    p.freq_offset = (pkt.frequency_offset_error as f32 * FREQ_OFFSET_LSB_125KHZ) as i32;
                },
                BW_250KHZ => {
                    p.freq_offset = (pkt.frequency_offset_error as f32 * FREQ_OFFSET_LSB_250KHZ) as i32;
                },
                BW_500KHZ => {
                    p.freq_offset = (pkt.frequency_offset_error as f32 * FREQ_OFFSET_LSB_500KHZ) as i32;
                },
                _ => {
                    p.freq_offset = 0;
                    error!("Invalid frequency offset");
                },
            }
    
            /* Adjust the frequency offset with channel IF frequency error:
            When the channel IF frequency has been configured, a precision error may have been introduced
            due to the register precision. We calculate this error here, and adjust the returned frequency error
            accordingly. */
            let if_freq_hz = context.if_chain_cfg[p.if_chain as usize].freq_hz; /* The IF frequency set in the registers, is the offset from the zero IF. */
            let if_freq_error = if_freq_hz - (IF_HZ_TO_REG(if_freq_hz) * 15625 / 32); /* The error corresponds to how many Hz are missing to get to actual 0 IF. */
            /* Example to better understand what we get here:
                - For a channel set to IF 400000Hz
                - The IF frequency register will actually be set to 399902Hz due to its resolution
                - This means that the modem, to shift to 0 IF, will apply -399902, instead of -400000.
                - This means that the modem will be centered +98hz above the real 0 IF
                - As the freq_offset given is supposed to be relative to the 0 IF, we add this resolution error to it */
            p.freq_offset += if_freq_error;
    
            /* Get timestamp correction to be applied to count_us */
            let _timestamp_correction = timestamp_counter_correction(context, p.bandwidth, p.datarate as u8, p.coderate, pkt.crc_en, pkt.rxbytenb_modem, RX_DFT_PEAK_MODE_AUTO);
    
            /* Compute fine timestamp for packets coming from the modem optimized for fine timestamping, if CRC is OK */
            p.ftime_received = false;
            p.ftime = 0;
            if (pkt.num_ts_metrics_stored > 0) && (pkt.timing_set == true) && (p.status == STAT_CRC_OK) {
                /* The actual packet frequency error compared to the channel frequency, need to compute the ftime */
                let pkt_freq_error = ((p.freq_hz as f64 + p.freq_offset as f64)  / (p.freq_hz) as f64) - 1.0;
    
                /* Compute the fine timestamp */
                if let Ok(ftime) = self.precise_timestamp_calculate(pkt.num_ts_metrics_stored, &pkt.timestamp_avg, pkt.timestamp_cnt, pkt.rx_rate_sf, context.if_chain_cfg[p.if_chain as usize].freq_hz, pkt_freq_error){
                    p.ftime = ftime;
                    p.ftime_received = true;
                }
            }
        } else if ifmod == IF_FSK_STD {
            trace!("Note: FSK packet (modem {:} chan {:})\n", pkt.modem_id, p.if_chain);
            p.modulation = Modulation::FSK;
    
            /* Get CRC status */
            if pkt.crc_en {
                /* CRC enabled */
                if pkt.payload_crc_error {
                    error!("FSK: CRC ERR\n");
                    p.status = STAT_CRC_BAD;
                } else {
                    error!("FSK: CRC OK\n");
                    p.status = STAT_CRC_OK;
                }
            } else {
                /* CRC disabled */
                p.status = STAT_NO_CRC;
            }
    
            /* Get modulation params */
            p.bandwidth = context.fsk_cfg.bandwidth as u8;
            p.datarate = context.fsk_cfg.datarate;
    
            /* Compute timestamp correction to be applied */
            let _timestamp_correction = (680000u32 / context.fsk_cfg.datarate) - 20;
    
            /* RSSI correction */
            p.rssic = RSSI_FSK_POLY_0 + RSSI_FSK_POLY_1 * p.rssic + RSSI_FSK_POLY_2 * p.rssic.powi(2) + RSSI_FSK_POLY_3 * p.rssic.powi(3);
    
            /* Undefined for FSK */
            p.coderate = CR_UNDEFINED;
            p.snr = -128.0;
            p.rssis = -128.0;
        } else {
            error!("ERROR: UNEXPECTED PACKET ORIGIN\n");
            p.status = STAT_UNDEFINED;
            p.modulation = Modulation::UNDEFINED;
            p.rssic = -128.0;
            p.rssis = -128.0;
            p.snr = -128.0;
            p.snr_min = -128.0;
            p.snr_max = -128.0;
            p.bandwidth = BW_UNDEFINED;
            p.datarate = DR_UNDEFINED;
            p.coderate = CR_UNDEFINED;
        }
    
        /* Scale 32 MHz packet timestamp to 1 MHz (microseconds) */
        p.count_us = pkt.timestamp_cnt / 32;
    
        /* Expand 27-bits counter to 32-bits counter, based on current wrapping status (updated after fetch) */
        p.count_us = self.sx1302.counter_us.pkt_expand( p.count_us);//timestamp_pkt_expand(&counter_us, p.count_us);
    
        /* Packet timestamp corrected */
        p.count_us = p.count_us + timestamp_correction;
    
        /* Packet CRC status */
        p.crc = pkt.rx_crc16_value;
    

    
       Ok(p)
    }



    fn precise_timestamp_calculate(&mut self, ts_metrics_nb: u8,  ts_metrics: &[i8],  timestamp_cnt:u32,  sf:u8,  if_freq_hz:i32,  pkt_freq_error:f64) -> Result<u32> {
        let mut ftime_sum:i32;
        let mut ftime=[0i32;256];
        let ftime_mean:f32;
        let timestamp_cnt_end_of_preamble;
        let mut timestamp_pps = 0;
        let mut timestamp_pps_reg ;
        let mut offset_preamble_hdr: u32;
        let mut buff = [0u8;4];
        let mut diff_pps:u32;
        let mut pkt_ftime:f64;
        let ts_metrics_nb_clipped:u8;
        let xtal_correct:f64;
        let mut timestamp_pps_idx  = 0;

        /* Check if we can calculate a ftime */
        if self.sx1302.timestamp_pps_history.size < MAX_TIMESTAMP_PPS_HISTORY {
            error!("INFO: Cannot compute ftime yet, PPS history is too short\n");
            return Err(anyhow!("LGW_HAL_ERROR"));
        }

        /* Coarse timestamp correction to match with GW v2 (end of header -> end of preamble) */
        offset_preamble_hdr =   256 * (1 << sf) * (8 + 4 + ( if (sf == 5) || (sf == 6) { 2 } else { 0 })) +
                                256 * ((1 << sf) / 4 - 1); /* 32e6 / 125e3 = 256 */

        /* Take the packet frequency error in account in the offset */
        offset_preamble_hdr += (offset_preamble_hdr as f64 * pkt_freq_error + 0.5) as u32;

        timestamp_cnt_end_of_preamble = timestamp_cnt - offset_preamble_hdr + 2138; /* 2138 is the number of 32MHz clock cycle offset b/w GW_V2 and SX1303 decimation/filtering group delay */

        /* Shift the packet coarse timestamp which is used to get ref PPS counter */
        let timestamp_cnt = timestamp_cnt_end_of_preamble;

        /* Clip the number of metrics depending on Spreading Factor, reduce fine timestamp variation versus packet duration */
        ts_metrics_nb_clipped = match sf {
            12 => ts_metrics_nb.min(4),
            11 => ts_metrics_nb.min(8),
            10 => ts_metrics_nb.min(16),
            _ => ts_metrics_nb.min(32),
        };


        /* Compute the ftime cumulative sum */
        ftime[0] = ts_metrics[0] as i32;
        ftime_sum = ftime[0];
        for i in 1 ..  2 * ts_metrics_nb_clipped as usize {
            ftime[i] = ftime[i-1] + ts_metrics[i] as i32;
            ftime_sum += ftime[i];
        }

        /* Compute the mean of the cumulative sum */
        ftime_mean = ftime_sum as f32 / (2 * ts_metrics_nb_clipped) as f32;

        /* Find the last timestamp_pps before packet to use as reference for ftime */
        let x = self.lgw_reg_rb(SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS , &mut buff, 4);
        if x.is_err() {
            error!("ERROR: Failed to get timestamp counter value\n");
            return Err(anyhow!("LGW_REG_ERROR"));
        }
        timestamp_pps_reg  = ((buff[0] as u32) << 24) & 0xFF000000;
        timestamp_pps_reg |= ((buff[1] as u32) << 16) & 0x00FF0000;
        timestamp_pps_reg |= ((buff[2] as u32) << 8)  & 0x0000FF00;
        timestamp_pps_reg |= ((buff[3] as u32) << 0)  & 0x000000FF;

        /* Ensure that the timestamp PPS history is up-to-date */
        self.sx1302.timestamp_pps_history.save(timestamp_pps_reg);
        

        /* Check if timestamp_pps_reg we just read is the reference to be used to compute ftime or not */
        if (timestamp_cnt - timestamp_pps_reg) > 32000000u32 {
            /* The timestamp_pps_reg we just read is after the packet timestamp, we need to rewind */
            for timestamp_pps_idx in 0 .. self.sx1302.timestamp_pps_history.size {
                /* search the pps counter in history */
                if (timestamp_cnt - self.sx1302.timestamp_pps_history.history[timestamp_pps_idx]) < 32000000u32 {
                    timestamp_pps = self.sx1302.timestamp_pps_history.history[timestamp_pps_idx];
                    trace!("==> timestamp_pps found at history[{:}] => {:}\n", timestamp_pps_idx, timestamp_pps);
                    break;
                }
            }
            if timestamp_pps_idx == self.sx1302.timestamp_pps_history.size {
                error!("ERROR: failed to find the reference timestamp_pps, cannot compute ftime\n");
                return Err(anyhow!("LGW_REG_ERROR"));
            }

            /* Calculate the Xtal error between the reference PPS we just found and the next one */
            let timestamp_pps_idx_next = if timestamp_pps_idx == (MAX_TIMESTAMP_PPS_HISTORY - 1) { 0 } else { timestamp_pps_idx + 1 };
            diff_pps = self.sx1302.timestamp_pps_history.history[timestamp_pps_idx_next] - self.sx1302.timestamp_pps_history.history[timestamp_pps_idx];
            xtal_correct = 32e6 / (diff_pps as f64);
        } else {
            /* The timestamp_pps_reg we just read is the reference we use to calculate the fine timestamp */
            timestamp_pps = timestamp_pps_reg;
            trace!("==> timestamp_pps => {:}\n", timestamp_pps);

            /* Calculate the Xtal error between the reference PPS we just found and the previous one */
            timestamp_pps_idx = self.sx1302.timestamp_pps_history.idx;
            let timestamp_pps_idx_prev = if timestamp_pps_idx == 0 { MAX_TIMESTAMP_PPS_HISTORY - 1 } else { timestamp_pps_idx - 1 };
            diff_pps = self.sx1302.timestamp_pps_history.history[timestamp_pps_idx] - self.sx1302.timestamp_pps_history.history[timestamp_pps_idx_prev];
            xtal_correct = 32e6 / (diff_pps as f64);
        }

        /* Sanity Check on xtal_correct */
        if (xtal_correct > 1.2) || (xtal_correct < 0.8) {
            error!("ERROR: xtal_error is invalid ({:})\n", xtal_correct);
            return Err(anyhow!("LGW_REG_ERROR"));
        }

        /* Coarse timestamp based on PPS reference */
        diff_pps = timestamp_cnt - timestamp_pps;

        trace!("timestamp_cnt : {:}\n", timestamp_cnt);
        trace!("timestamp_pps : {:}\n", timestamp_pps);
        trace!("diff_pps : {:}\n", diff_pps);

        /* Compute the fine timestamp */
        pkt_ftime = diff_pps as f64 + ftime_mean as f64;
        trace!("pkt_ftime = {:}\n", pkt_ftime);

        /* Add the DC notch filtering delay if necessary */
        pkt_ftime += sx1302_dc_notch_delay(if_freq_hz as f64 / 1E3);

        /* Convert fine timestamp from 32 Mhz clock to nanoseconds */
        pkt_ftime *= 31.25;

        /* Apply current XTAL error correction */
        pkt_ftime *= xtal_correct;

        let result_ftime = pkt_ftime as u32;
        if result_ftime > 1000000000 {
            error!("ERROR: fine timestamp is out of range ({:})\n", result_ftime);
            return Err(anyhow!("LGW_REG_ERROR"));
        }

        trace!("==> ftime = {:} ns since last PPS ({:})\n", result_ftime, pkt_ftime);

        return Ok(result_ftime);
    }

    
}

fn lora_crc16(data: u8, crc: &mut u16) {
    let mut next = 0u16;

    next |= (((data >> 0) & 1) as u16 ^ ((*crc >> 12) & 1) ^ ((*crc >> 8) & 1)) << 0;
    next |= (((data >> 1) & 1) as u16 ^ ((*crc >> 13) & 1) ^ ((*crc >> 9) & 1)) << 1;
    next |= (((data >> 2) & 1) as u16 ^ ((*crc >> 14) & 1) ^ ((*crc >> 10) & 1)) << 2;
    next |= (((data >> 3) & 1) as u16 ^ ((*crc >> 15) & 1) ^ ((*crc >> 11) & 1)) << 3;
    next |= (((data >> 4) & 1) as u16 ^ ((*crc >> 12) & 1)) << 4;
    next |= (((data >> 5) & 1) as u16 ^ ((*crc >> 13) & 1) ^ ((*crc >> 12) & 1) ^ ((*crc >> 8) & 1)) << 5;
    next |= (((data >> 6) & 1) as u16 ^ ((*crc >> 14) & 1) ^ ((*crc >> 13) & 1) ^ ((*crc >> 9) & 1)) << 6;
    next |= (((data >> 7) & 1) as u16 ^ ((*crc >> 15) & 1) ^ ((*crc >> 14) & 1) ^ ((*crc >> 10) & 1)) << 7;
    next |= ((((*crc >> 0) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 11) & 1))) << 8;
    next |= ((((*crc >> 1) & 1) ^ ((*crc >> 12) & 1))) << 9;
    next |= ((((*crc >> 2) & 1) ^ ((*crc >> 13) & 1))) << 10;
    next |= ((((*crc >> 3) & 1) ^ ((*crc >> 14) & 1))) << 11;
    next |= ((((*crc >> 4) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 12) & 1) ^ ((*crc >> 8) & 1))) << 12;
    next |= ((((*crc >> 5) & 1) ^ ((*crc >> 13) & 1) ^ ((*crc >> 9) & 1))) << 13;
    next |= ((((*crc >> 6) & 1) ^ ((*crc >> 14) & 1) ^ ((*crc >> 10) & 1))) << 14;
    next |= ((((*crc >> 7) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 11) & 1))) << 15;

    *crc = next;
}



fn sx1302_lora_payload_crc(data: &[u8], size:usize) -> u16 {
    let mut crc = 0;

    for i in 0 .. size {
        lora_crc16(data[i], &mut crc);
    }

    //println!("CRC16: 0x{:02X} 0x{:02X} ({:X})", (crc >> 8) as u8, crc as u8, crc);
    crc as u16
}

fn sx1302_dc_notch_delay(if_freq_khz: f64) -> f64 {
    if if_freq_khz < -75.0 || if_freq_khz > 75.0 {
        0.0
    } else {
        1.7e-6 * if_freq_khz.powi(4)
            + 2.4e-6 * if_freq_khz.powi(3)
            - 0.0101 * if_freq_khz.powi(2)
            - 0.01275 * if_freq_khz
            + 10.2922
    }
}
