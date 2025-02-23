use anyhow::{anyhow,Result};
use tracing::{debug, error, info};
use super::helper::wait_ms;
use super::{loragw_com::LgwSpiMuxTarget, mcu::command::{ECmdSpiTarget, MCU_SPI_REQ_TYPE_READ_WRITE}, Hal, LGW_RF_CHAIN_NB};


const    STDBY_RC :u8               = 0x00;
const    STDBY_XOSC:u8              = 0x01;

#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum Sx1250OpCode {
    CALIBRATE               = 0x89,
    CALIBRATE_IMAGE         = 0x98,
    CLR_IRQ_STATUS          = 0x02,
    STOP_TIMER_ON_PREAMBLE  = 0x9F,
    SET_RFSWITCHMODE        = 0x9D,
    GET_IRQ_STATUS          = 0x12,
    GET_RX_BUFFER_STATUS    = 0x13,
    GET_PACKET_STATUS       = 0x14,
    READ_BUFFER             = 0x1E,
    READ_REGISTER           = 0x1D,
    SET_DIO_IRQ_PARAMS      = 0x08,
    SET_MODULATION_PARAMS   = 0x8B,
    SET_PA_CONFIG           = 0x95,
    SET_PACKET_PARAMS       = 0x8C,
    SET_PACKET_TYPE         = 0x8A,
    SET_RF_FREQUENCY        = 0x86,
    SET_BUFFER_BASE_ADDRESS = 0x8F,
    SET_SLEEP               = 0x84,
    SET_STANDBY             = 0x80,
    SET_RX                  = 0x82,
    SET_TX                  = 0x83,
    SET_TX_PARAMS           = 0x8E,
    WRITE_BUFFER            = 0x0E,
    WRITE_REGISTER          = 0x0D,
    SET_TXCONTINUOUSWAVE    = 0xD1,
    SET_TXCONTINUOUSPREAMBLE= 0xD2,
    GET_STATUS              = 0xC0,
    SET_REGULATORMODE       = 0x96,
    SET_FS                  = 0xC1,
    GET_DEVICE_ERRORS       = 0x17
} 


const WAIT_BUSY_SX1250_MS:u32 =  1;

macro_rules! take_n_bits_from {
    ($b:expr, $p:expr, $n:expr) => {
        (($b >> $p) & ((1 << $n) - 1))
    };
}

fn sx1250_freq_to_reg(f: u32) -> u32 {
    ((f as u64 * (1 << 25)) / 32000000) as u32
}

pub trait LoragwSx1250Trait {
    fn sx1250_calibrate(&mut self, radio:u8, freq_hz: u32) -> Result<()>;
    fn sx1250_setup(&mut self,  rf_chain:u8,  freq_hz:u32,  single_input_mode:bool)->Result<()>;
    fn sx1250_w(&mut self,  spi_mux_target:LgwSpiMuxTarget,  op_code:Sx1250OpCode, data:&[u8], size:usize) -> Result<()>;
    fn sx1250_reg_w(&mut self,  op_code:Sx1250OpCode, data:&[u8], size: usize, rf_chain:u8)->Result<()> ;
    fn sx1250_r(&mut self,  spi_mux_target: LgwSpiMuxTarget,  op_code:Sx1250OpCode,  data:&mut [u8], size: usize) ->Result<()>;
    fn sx1250_reg_r(&mut self, op_code: Sx1250OpCode,  data:&mut [u8], size: usize, rf_chain:u8)->Result<()>;
}

impl LoragwSx1250Trait for Hal {
    fn sx1250_calibrate(&mut self, _radio:u8, _freq_hz: u32) -> Result<()> {
        Ok(())
    }
    
    fn sx1250_r(&mut self,  spi_mux_target: LgwSpiMuxTarget,  op_code:Sx1250OpCode,  data:&mut [u8], size:usize) ->Result<()>
    {
        let command_size = size + 7; /* 5 bytes: REQ metadata, 2 bytes: RAW SPI frame */
        let mut in_out_buf = vec![0u8;command_size];
       

        if data.is_empty() {
            return Err(anyhow!("LGW_REG_ERR"))
        }
    
        /* wait BUSY */
        wait_ms(WAIT_BUSY_SX1250_MS);
    
        /* prepare command */
        /* Request metadata */
        in_out_buf[0] = 0; /* Req ID */
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1302 as u8; /* MCU -> SX1302 */
        in_out_buf[3] = ((size + 2) >> 8) as u8; /* payload size + spi_mux_target + op_code */
        in_out_buf[4] = ((size + 2) >> 0) as u8; /* payload size + spi_mux_target + op_code */
        /* RAW SPI frame */
        in_out_buf[5] = spi_mux_target as u8; /* SX1302 -> RADIO_A or RADIO_B */
        in_out_buf[6] = op_code as u8;

        for i in 0 .. size {
            in_out_buf[i + 7] = data[i];
        }
        if let Err(_) = self.mcu.mcu_spi_write( &mut in_out_buf ) {
            error!("ERROR: USB SX1250 READ FAILURE\n");
            return Err(anyhow!("LGW_COM_ERR"))
        }
        else{
            debug!("Note: USB SX1250 read success\n");
            for i in 0 .. size {
                data[i] = in_out_buf[7 + i];
            }
        }
            
        Ok(())
    }


    fn sx1250_reg_r(&mut self, op_code: Sx1250OpCode,  data:&mut [u8], size: usize,  rf_chain:u8)->Result<()> {
       
        /* checking input parameters */
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: INVALID RF_CHAIN\n");
            return Err(anyhow!("LGW_REG_ERROR"))
        }

        if let Err(_) = self.sx1250_r( if rf_chain == 0 { LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_RADIOA } else{ LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_RADIOB}, op_code, data, size) {
            error!("ERROR: COM ERROR DURING RADIO REGISTER READ\n");
            return Err(anyhow!("LGW_REG_ERROR"))
        }

        Ok(())
    }
    fn sx1250_w(&mut self,  spi_mux_target:LgwSpiMuxTarget,  op_code:Sx1250OpCode, data:&[u8], size: usize) -> Result<()> {
     
        let command_size = size + 7; /* 5 bytes: REQ metadata, 2 bytes: RAW SPI frame */
        let mut in_out_buf = vec![0u8;command_size];
        

        if data.is_empty() {
            return Err(anyhow!("LGW_COM_ERR"))
        }

        /* wait BUSY */
        wait_ms(WAIT_BUSY_SX1250_MS);

        /* prepare command */
        /* Request metadata */
        in_out_buf[0] = 0; /* Req ID */
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1302 as u8; /* MCU -> SX1302 */
        in_out_buf[3] = ((size + 2) >> 8) as u8; /* payload size + spi_mux_target + op_code */
        in_out_buf[4] = ((size + 2) >> 0) as u8; /* payload size + spi_mux_target + op_code */
        /* RAW SPI frame */
        in_out_buf[5] = spi_mux_target as u8; /* SX1302 -> RADIO_A or RADIO_B */
        in_out_buf[6] = op_code as u8;
        for i in 0 .. size {
            in_out_buf[i + 7] = data[i];
        }

        if let Err(e) = self.mcu.mcu_spi_write(&mut in_out_buf) {
            error!("ERROR: USB SX1250 WRITE FAILURE\n");
            return Err(e)
        }


        Ok(())
    }

    fn sx1250_reg_w(&mut self,  op_code:Sx1250OpCode, data:&[u8], size: usize, rf_chain:u8)->Result<()> {
        
        /* checking input parameters */
        if rf_chain >= LGW_RF_CHAIN_NB {
            error!("ERROR: INVALID RF_CHAIN\n");
            return Err(anyhow!("LGW_REG_ERROR"));
        }
    
        if let Err(_) = self.sx1250_w( if rf_chain == 0 {  LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_RADIOA } else { LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_RADIOB}, op_code, data, size) {
            error!("ERROR: COM ERROR DURING RADIO REGISTER WRITE\n");
            return Err(anyhow!("LGW_REG_ERROR"));
        }
        Ok(())
    }

    fn sx1250_setup(&mut self,  rf_chain:u8,  freq_hz:u32,  single_input_mode:bool)->Result<()> {
        let freq_reg: u32;
        let mut buff = vec![0u8;16];
        /* Set Radio in Standby for calibrations */
        buff[0] = STDBY_RC as u8;
        let e1 = self.sx1250_reg_w(Sx1250OpCode::SET_STANDBY, &buff,1, rf_chain);
        wait_ms(10);

        /* Get status to check Standby mode has been properly set */
        buff[0] = 0x00;
        let e2 = self.sx1250_reg_r(Sx1250OpCode::GET_STATUS, &mut buff, 1, rf_chain);

        if e1.is_err() || e2.is_err() {
            return Err(anyhow!("LGW_COM_ERR"))
        }

        if (take_n_bits_from!(buff[0], 4, 3)) as u8 != 0x02 {
            error!("ERROR: Failed to set SX1250_{:} in STANDBY_RC mode\n", rf_chain);
            return Err(anyhow!("LGW_REG_ERROR"));
        }

        /* Run all calibrations (TCXO) */
        buff[0] = 0x7F;
        self.sx1250_reg_w(Sx1250OpCode::CALIBRATE, &buff, 1, rf_chain)?;
        wait_ms(10);

        /* Set Radio in Standby with XOSC ON */
        buff[0] = STDBY_XOSC as u8;
        self.sx1250_reg_w(Sx1250OpCode::SET_STANDBY, &buff, 1, rf_chain)?;
        wait_ms(10);

        /* Get status to check Standby mode has been properly set */
        buff[0] = 0x00;
        self.sx1250_reg_r(Sx1250OpCode::GET_STATUS, &mut buff, 1, rf_chain)?;
        if take_n_bits_from!(buff[0], 4, 3) as u8 != 0x03 {
            error!("ERROR: Failed to set SX1250_{:} in STANDBY_XOSC mode\n", rf_chain);
            return Err(anyhow!("LGW_REG_ERROR"))
        }

        /* Set Bitrate to maximum (to lower TX to FS switch time) */
        buff[0] = 0x06;
        buff[1] = 0xA1;
        buff[2] = 0x01;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?;
        buff[0] = 0x06;
        buff[1] = 0xA2;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?;
        buff[0] = 0x06;
        buff[1] = 0xA3;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?;

        /* Configure DIO for Rx */
        buff[0] = 0x05;
        buff[1] = 0x82;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?; /* Drive strength to min */
        buff[0] = 0x05;
        buff[1] = 0x83;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?; /* Input enable, all disabled */
        buff[0] = 0x05;
        buff[1] = 0x84;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?; /* No pull up */
        buff[0] = 0x05;
        buff[1] = 0x85;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?; /* No pull down */
        buff[0] = 0x05;
        buff[1] = 0x80;
        buff[2] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?; /* Output enable, all enabled */

        /* Set fix gain (??) */
        buff[0] = 0x08;
        buff[1] = 0xB6;
        buff[2] = 0x2A;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?;

        /* Set frequency */
        freq_reg = sx1250_freq_to_reg(freq_hz);
        buff[0] = (freq_reg >> 24) as u8;
        buff[1] = (freq_reg >> 16) as u8;
        buff[2] = (freq_reg >> 8) as u8;
        buff[3] = (freq_reg >> 0) as u8;
        self.sx1250_reg_w(Sx1250OpCode::SET_RF_FREQUENCY, &buff, 4, rf_chain)?;

        /* Set frequency offset to 0 */
        buff[0] = 0x08;
        buff[1] = 0x8F;
        buff[2] = 0x00;
        buff[3] = 0x00;
        buff[4] = 0x00;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 5, rf_chain)?;

        /* Set Radio in Rx mode, necessary to give a clock to SX1302 */
        buff[0] = 0xFF;
        buff[1] = 0xFF;
        buff[2] = 0xFF;
        self.sx1250_reg_w(Sx1250OpCode::SET_RX, &buff, 3, rf_chain)?; /* Rx Continuous */

        /* Select single input or differential input mode */
        if single_input_mode == true {
            info!("INFO: Configuring SX1250_{:} in single input mode\n", rf_chain);
            buff[0] = 0x08;
            buff[1] = 0xE2;
            buff[2] = 0x0D;
            self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?;
        }

        buff[0] = 0x05;
        buff[1] = 0x87;
        buff[2] = 0x0B;
        self.sx1250_reg_w(Sx1250OpCode::WRITE_REGISTER, &buff, 3, rf_chain)?; /* FPGA_MODE_RX */

        Ok(())
    }
}