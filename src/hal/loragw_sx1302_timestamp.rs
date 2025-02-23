use super::{LoragwRegTrait, BW_125KHZ, BW_250KHZ, BW_500KHZ, SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, SX1302_REG_RX_TOP_RXBUFFER_TIMESTAMP_CFG_MAX_TS_METRICS, SX1302_REG_RX_TOP_TIMESTAMP_ENABLE, SX1302_REG_RX_TOP_TIMESTAMP_NB_SYMB};

use super::{LgwContext, loragw_sx1302::{RX_DFT_PEAK_MODE_DISABLED, SET_PPM_ON}, Hal};
use anyhow::{anyhow,Result};
use tracing::{debug, error, trace};


pub const PRECISION_TIMESTAMP_TS_METRICS_MAX:u8 =  32; /* reduce number of metrics to better match GW v2 fine timestamp (max is 255) */
pub const PRECISION_TIMESTAMP_NB_SYMBOLS: u8 =      0;

#[derive(Debug)]
pub struct TimestampInfo {
    pub counter_us_27bits_ref: u32,     /* reference value (last read) */
    pub counter_us_27bits_wrap: u8    /* rollover/wrap status */
}

#[derive(Debug)]
pub struct TimestampCounter {
    pub  inst: TimestampInfo, /* holds current reference of the instantaneous counter */
    pub  pps: TimestampInfo  /* holds current reference of the pps-trigged counter */
}

impl TimestampCounter {
    pub fn new() -> Self {
        Self {
            inst: TimestampInfo {
                counter_us_27bits_ref: 0, 
                counter_us_27bits_wrap: 0
            },
            pps: TimestampInfo {
                counter_us_27bits_ref: 0, 
                counter_us_27bits_wrap: 0
            }
        }
    }

    pub fn update(&mut self,  pps:u32,  inst:u32 ) {
        //struct TimestampInfo* tinfo = (pps == true) ? &self->pps : &self->inst;

        /* Check if counter has wrapped, and update wrap status if necessary */
        if pps < self.pps.counter_us_27bits_ref {
            self.pps.counter_us_27bits_wrap += 1;
            self.pps.counter_us_27bits_wrap %= 32;
        }
        if inst < self.inst.counter_us_27bits_ref {
            self.inst.counter_us_27bits_wrap += 1;
            self.inst.counter_us_27bits_wrap %= 32;
        }

        /* Update counter reference */
        self.pps.counter_us_27bits_ref = pps;
        self.inst.counter_us_27bits_ref = inst;
    }

    pub fn expand(&mut self,  pps:bool,  cnt_us: u32) -> u32 {
        let tinfo = if pps == true { &self.pps } else{ &self.inst };
    
        let counter_us_32bits = ((tinfo.counter_us_27bits_wrap as u32) << 27) | cnt_us;

    
        return counter_us_32bits;
    }


    pub fn pkt_expand(&mut self,  pkt_cnt_us: u32) -> u32 {
        let tinfo = &self.inst;
        
    
        /* Check if counter has wrapped since the packet has been received in the sx1302 internal FIFO */
        /* If the sx1302 counter was greater than the pkt timestamp, it means that the internal counter
            hasn't rolled over since the packet has been received by the sx1302
            case 1: --|-P--|----|--R-|----|--||-|----|-- : use current wrap status counter
            case 2: --|-P-||-|-R--|-- : use previous wrap status counter
            P : packet received in sx1302 internal FIFO
            R : read packet from sx1302 internal FIFO
            | : last update internal counter ref value.
            ||: sx1302 internal counter rollover (wrap)
        */
    
        /* Use current wrap counter or previous ? */
        let mut wrap_status = tinfo.counter_us_27bits_wrap - (if tinfo.counter_us_27bits_ref >= pkt_cnt_us { 0 } else{ 1 });
        wrap_status &= 0x1F; /* [0..31] */
    
        /* Expand packet counter */
        let counter_us_32bits = ((wrap_status as u32) << 27) | pkt_cnt_us;
    
        counter_us_32bits
    }
    
}


pub const MAX_TIMESTAMP_PPS_HISTORY:usize = 16;

#[derive(Debug)]
pub struct TimestampPpsHistory {
    pub history: [u32;MAX_TIMESTAMP_PPS_HISTORY],
    pub idx:usize, /* next slot to be written */
    pub size:usize /* current size */
}

impl TimestampPpsHistory {
    pub fn new()->Self {
        Self { 
            history: [0u32;MAX_TIMESTAMP_PPS_HISTORY],
            idx: 0,
            size: 0
        }
    }

    pub fn save(&mut self, timestamp_pps_reg:u32) {
        /* Store it only if different from the previous one */
        if timestamp_pps_reg != self.history[self.idx] || (self.size == 0) {
            /* Select next index */
            if self.size > 0 {
                self.idx += 1;
            }
            if self.idx == MAX_TIMESTAMP_PPS_HISTORY {
                self.idx = 0;
            }

            /* Set PPS counter value */
            self.history[self.idx] = timestamp_pps_reg;

            /* Add one entry to the history */
            if self.size < MAX_TIMESTAMP_PPS_HISTORY {
                self.size += 1;
            }
        }
    }

    
}


pub trait SX1302TimestampTrait {
    fn timestamp_counter_mode(&mut self,  ftime_enable:bool)->Result<()>;
}


impl SX1302TimestampTrait for Hal {


    

    
    fn timestamp_counter_mode(&mut self,  ftime_enable:bool)->Result<()>{
        
        if ftime_enable == false {
            debug!("INFO: using legacy timestamp\n");
            /* Latch end-of-packet timestamp (sx1301 compatibility) */
            self.lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x01)?;
        } else {
            debug!("INFO: using precision timestamp (max_ts_metrics:{:} nb_symbols:{:})\n", PRECISION_TIMESTAMP_TS_METRICS_MAX, PRECISION_TIMESTAMP_NB_SYMBOLS);

            /* Latch end-of-preamble timestamp */
            self.lgw_reg_w(SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP, 0x00)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_RXBUFFER_TIMESTAMP_CFG_MAX_TS_METRICS, PRECISION_TIMESTAMP_TS_METRICS_MAX as i32)?;

            /* LoRa multi-SF modems */
            self.lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_ENABLE, 0x01)?;
            self.lgw_reg_w(SX1302_REG_RX_TOP_TIMESTAMP_NB_SYMB, PRECISION_TIMESTAMP_NB_SYMBOLS as i32)?;
        }

        Ok(())
    }
}


pub fn timestamp_counter_correction( context: &LgwContext,  bandwidth: u8,  datarate: u8,  coderate:u8,  crc_en:bool,  payload_length: u8,  dft_peak_mode: u8) -> i32 {
    /* Check input parameters */

    if Hal::is_lora_dr(datarate as u32) == false {
        error!("ERROR: wrong datarate ({:})", datarate);
        return 0;
    }
    if Hal::is_lora_bw(bandwidth) == false {
        error!("ERROR: wrong bandwidth ({:})\n", bandwidth);
        return 0;
    }
    if Hal::is_lora_cr(coderate) == false {
        error!("ERROR: wrong coding rate ({:})", coderate);
        return 0;
    }

    /* Calculate the correction to be applied */
    if context.ftime_cfg.enable == false {
        return legacy_timestamp_correction(bandwidth, datarate, coderate, crc_en, payload_length, dft_peak_mode);
    } else {
        return precision_timestamp_correction(bandwidth, datarate, coderate, crc_en, payload_length);
    }
}



pub fn legacy_timestamp_correction( bandwidth:u8,  sf:u8,  cr:u8,  crc_en:bool,  payload_length:u8,  dft_peak_mode:u8) -> i32 {
    
    let demap_delay:u64;
    let  fft_delay_state3:u64;
    let  fft_delay:u64;
    let decode_delay:u64;
    let  total_delay:u64;

    let mut nb_iter =  if dft_peak_mode == RX_DFT_PEAK_MODE_DISABLED  { 0 } else { 1 };
    let bw_pow ;
    let mut dft_peak_en= nb_iter;

    let nb_nibble_in_hdr:u32;
    let mut nb_nibble_in_last_block:u32;

    let ppm =  if SET_PPM_ON(bandwidth, sf as u32)  { 1 } else { 0 };
    let timestamp_correction:i32;
    let mut payload_fits_in_header = false;
    let mut cr_local = cr;


    match bandwidth {
        BW_125KHZ => bw_pow = 1,
        BW_250KHZ => bw_pow = 2,
        BW_500KHZ => bw_pow = 4,
        _ => {
            println!("ERROR: UNEXPECTED VALUE {} IN MATCH STATEMENT", bandwidth);
            return 0;
        }
    }

    /* Prepare variables for delay computing */
    let clk_period = 250000u64 / bw_pow;

    let nb_nibble = (payload_length as u32 + 2 * (crc_en as u32)) * 2 + 5;

    if (sf == 5) || (sf == 6) {
        nb_nibble_in_hdr = sf as u32;
    } else {
        nb_nibble_in_hdr = sf as u32 - 2;
    }

    nb_nibble_in_last_block = nb_nibble - nb_nibble_in_hdr - (sf as u32 - 2 * ppm) * ((nb_nibble - nb_nibble_in_hdr) / (sf as u32 - 2 * ppm));
    if nb_nibble_in_last_block == 0 {
        nb_nibble_in_last_block = sf as u32 - 2 * ppm;
    }

    nb_iter = (sf as u64 + 1) / 2; /* intended to be truncated */

    /* Update some variables if payload fits entirely in the header */
    if ((2 * (payload_length + 2 * (crc_en as u8)) as i32 - (sf as i32 - 7)) <= 0) || ((payload_length == 0) && (crc_en == false)) {
        /* Payload fits entirely in first 8 symbols (header):
            - not possible for SF5/SF6, unless payload length is 0 and no CRC
        */
        payload_fits_in_header = true;

        /* overwrite some variables accordingly */
        dft_peak_en = 0;

        cr_local = 4; /* header coding rate is 4 */

        if sf > 6 {
            nb_nibble_in_last_block = sf as u32 - 2;
        } else {
            nb_nibble_in_last_block = sf as u32;
        }
    }

    /* Filtering delay : I/Q 32Mhz -> 4Mhz */
    let filtering_delay = 16000000u64 / bw_pow + 2031250;

    /* demap delay */
    if payload_fits_in_header == true {
        demap_delay = clk_period + (1 << sf) * clk_period * 3 / 4 + 3 * clk_period + (sf as u64 - 2) * clk_period;
    } else {
        demap_delay = clk_period + (1 << sf) * clk_period * (1 - ppm as u64 / 4) + 3 * clk_period + (sf as u64 - 2 * ppm as u64) * clk_period;
    }

    /* FFT delays */
    fft_delay_state3 = clk_period * (((1 << sf) - 6) + 2 * ((1 << sf) * (nb_iter - 1) + 6)) + 4 * clk_period;

    if dft_peak_en != 0 {
        fft_delay = (5 - 2 * ppm) as u64 * (((1 as u64)  << sf) * clk_period + 7 * clk_period) + 2 * clk_period;
    } else {
        fft_delay = (1 << sf) * 2 * clk_period + 3 * clk_period;
    }

    /* Decode delay */
    decode_delay = 5 * clk_period + (9 * clk_period + clk_period * cr_local as u64) * nb_nibble_in_last_block as u64 + 3 * clk_period;

    /* Cumulated delays */
    total_delay = (filtering_delay + fft_delay_state3 + fft_delay + demap_delay + decode_delay + 500000u64) / 1000000u64;

    if total_delay > i32::MAX as u64 {
        error!("ERROR: overflow error for timestamp correction (SHOULD NOT HAPPEN)\n");
        error!("=> filtering_delay {:}\n", filtering_delay);
        error!("=> fft_delay_state3 {:}\n", fft_delay_state3);
        error!("=> fft_delay {:}\n", fft_delay);
        error!("=> demap_delay {:}\n", demap_delay);
        error!("=> decode_delay {:}\n", decode_delay);
        error!("=> total_delay {:}\n", total_delay);
    }

    timestamp_correction = -(total_delay as i32); /* compensate all decoding processing delays */

    trace!("FTIME OFF : filtering_delay {:} \n", filtering_delay);
    trace!("FTIME OFF : fft_delay_state3 {:} \n", fft_delay_state3);
    trace!("FTIME OFF : fft_delay {:} \n", fft_delay);
    trace!("FTIME OFF : demap_delay {:} \n", demap_delay);
    trace!("FTIME OFF : decode_delay {:} \n", decode_delay);
    trace!("FTIME OFF : timestamp correction {:}\n", timestamp_correction);

    return timestamp_correction;
}



pub fn precision_timestamp_correction(bandwidth: u8, datarate: u8, coderate: u8, crc_en: bool, payload_length: u8) -> i32 {

    let mut timestamp_correction: i32;
    let bw_pow: u8;
    let filtering_delay: u32;

    match bandwidth {
        BW_125KHZ => bw_pow = 1,
        BW_250KHZ => bw_pow = 2,
        BW_500KHZ => bw_pow = 4,
        _ => {
            println!("ERROR: UNEXPECTED VALUE {} IN MATCH STATEMENT", bandwidth);
            return 0;
        }
    }

    filtering_delay = 16000000 / bw_pow as u32 + 2031250;

    /* NOTE: no need of the preamble size, only the payload duration is needed */
    /* WARNING: implicit header not supported */
    let toa = lora_packet_time_on_air(bandwidth, datarate, coderate, 0, false, !crc_en, payload_length);
    if toa.is_err()
    {
        error!("ERROR: failed to compute packet time on air");
        return 0;
    }

    let (_, _, nb_symbols_payload, t_symbol_us) = toa.unwrap();

    timestamp_correction = 0;
    timestamp_correction += (nb_symbols_payload * t_symbol_us as u32) as i32; /* shift from end of header to end of packet */
    timestamp_correction -= ((filtering_delay + 500000) / 1000000) as i32; /* compensate the filtering delay */

    println!("FTIME ON : timestamp correction {}", timestamp_correction);

    timestamp_correction
}



/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

pub fn lora_packet_time_on_air(
    bw: u8,
    sf: u8,
    cr: u8,
    n_symbol_preamble: u16,
    no_header: bool,
    no_crc: bool,
    size: u8,
) -> Result<(u32, f64, u32, u16)> {
    let h: u8;
    let de: u8;
    let n_bit_crc: u8;
    let bw_pow: u8;
    let t_symbol_us: u16;
    let n_symbol: f64;
    let toa_us: u32;
    let n_symbol_payload: u32;

    /* Check input parameters */
    if !Hal::is_lora_dr(sf as u32) {
        error!("ERROR: wrong datarate");
        return Err(anyhow!("LGW_HAL_ERR"));
    }
    if !Hal::is_lora_bw(bw) {
        error!("ERROR: wrong bandwidth");
        return Err(anyhow!("LGW_HAL_ERR"));
    }
    if !Hal::is_lora_cr(cr) {
        error!("ERROR: wrong coding rate");
        return Err(anyhow!("LGW_HAL_ERR"));
    }

    /* Get bandwidth 125KHz divider*/
    bw_pow = match bw {
        BW_125KHZ => 1,
        BW_250KHZ => 2,
        BW_500KHZ => 4,
        _ => {
            error!("ERROR: unsupported bandwidth 0x{:02X}", bw);
            return Err(anyhow!("LGW_HAL_ERR"));
        }
    };

    /* Duration of 1 symbol */
    t_symbol_us = (1u16 << sf) * 8 / bw_pow as u16; /* 2^SF / BW , in microseconds */

    /* Packet parameters */
    h = if !no_header { 1 } else { 0 }; /* header is always enabled, except for beacons */
    de = if sf >= 11 { 1 } else { 0 }; /* Low datarate optimization enabled for SF11 and SF12 */
    n_bit_crc = if !no_crc { 16 } else { 0 };

    /* Number of symbols in the payload */
    n_symbol_payload = (((8 * size as u32 + n_bit_crc as u32 - 4 * sf as u32 + if sf >= 7 { 8 } else { 0 } + 20 * h as u32) as f64).max(0.0)
        / (4 * (sf as u32 - 2 * de as u32)) as f64)
        .ceil() as u32
        * (cr as u32 + 4);

    /* number of symbols in packet */
    n_symbol = n_symbol_preamble as f64 + if sf >= 7 { 4.25 } else { 6.25 } + 8.0 + n_symbol_payload as f64;

    /* Duration of packet in microseconds */
    toa_us = (n_symbol * t_symbol_us as f64) as u32;

    trace!("INFO: LoRa packet ToA: {} us (n_symbol:{}, t_symbol_us:{})", toa_us, n_symbol, t_symbol_us);

    let out_nb_symbols = n_symbol;
    let out_nb_symbols_payload = n_symbol_payload;
    let out_t_symbol_us = t_symbol_us;
   

    Ok((toa_us, out_nb_symbols, out_nb_symbols_payload, out_t_symbol_us))
}


