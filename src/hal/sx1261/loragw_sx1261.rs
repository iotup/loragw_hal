use std::{sync::{Arc, RwLock}, thread::sleep, time::Duration};

use crate::hal::{ error::Error, mcu::{command::{ECmdSpiTarget, EComWriteMode, MCU_SPI_REQ_TYPE_READ_WRITE}, Mcu}, sx1261::{sx1261_def::{sx1261_freq_to_reg, SX1261StatusCommandStatus, SX1261StatusMode}, sx1261_pram::{PRAM, PRAM_COUNT}}, BW_125KHZ, BW_250KHZ};
use super::{sx1261_def::{SX1261OpCode, SX1261StandbyModes}, LgwSpectralScanStatus};
use anyhow::{Result,anyhow};
use tracing::{debug, error, info, trace};

#[derive(Debug)]
pub struct SX126x {

    spi_req_nb: u8,
    write_mode: EComWriteMode,
    mcu: Arc<RwLock<Mcu>>
}

const SX1261_PRAM_VERSION_FULL_SIZE:usize = 16; /* 15 bytes + terminating char */

impl SX126x {
    pub fn new( mcu: Arc<RwLock<Mcu>> ) -> Self {
        Self {
            spi_req_nb: 0,
            write_mode: EComWriteMode::LGW_COM_WRITE_MODE_SINGLE,
            mcu: mcu
        }
    }

    pub fn write(&mut self, op_code: SX1261OpCode, data: &[u8], size: usize ) -> Result<()> {
        let command_size = size + 6;

        let mut in_out_buf = vec![0u8;command_size];

        in_out_buf[0] = self.spi_req_nb;
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE;
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1261 as u8;

        in_out_buf[3] = ((size + 1) >> 8) as u8;
        in_out_buf[4] = ((size + 1) >> 0) as u8;

        in_out_buf[5] = op_code as u8;

        for i in 0 .. size {
            in_out_buf[i + 6] = data[i]
        }

        let mut mcu = self.mcu.write().unwrap();

        if self.write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            mcu.mcu_spi_store(&in_out_buf)?;
            self.spi_req_nb += 1;
        }
        else{
            mcu.mcu_spi_write(&mut in_out_buf)?;
        }

        Ok(())
    }

    pub fn read(&mut self, op_code: SX1261OpCode, data: &mut [u8], size: usize ) -> Result<()> {
        let command_size = size + 6;
        let mut in_out_buf = vec![0u8; command_size];

        in_out_buf[0] = self.spi_req_nb;
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE;
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1261 as u8;

        in_out_buf[3] = ((size  + 1) >> 8) as u8; /* payload size + op_code */
        in_out_buf[4] = ((size  + 1) >> 0) as u8; /* payload size + op_code */

        in_out_buf[5] = op_code as u8;

        for i in 0 .. size {
            in_out_buf[i + 6] = data[i];
        }

        if self.write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            return Err(anyhow!("ERROR: USB READ BURST FAILURE - bulk mode is enabled"))
        }
        else {
            let mut mcu = self.mcu.write().unwrap();
            mcu.mcu_spi_write(&mut in_out_buf)?;
        }

        for i in 0..size {
            data[i] = in_out_buf[6+i]
        }

        Ok(())
    }

    pub fn set_write_mode(&mut self, write_mode:EComWriteMode){
        self.write_mode = write_mode;
    }

    pub fn flush(&mut self) -> Result<()> {

        self.write_mode = EComWriteMode::LGW_COM_WRITE_MODE_SINGLE;

        if self.spi_req_nb == 0 {
            return Ok(())
        }

        let mut mcu = self.mcu.write().unwrap();
        mcu.mcu_spi_flush()?;

        self.spi_req_nb = 0;

        Ok(())
    }



    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    fn reg_w(&mut self,  op_code: SX1261OpCode,  data: &[u8],  size: usize) -> Result<()> {
        self.write(op_code, data, size)
    }

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

    fn reg_r(&mut self,  op_code: SX1261OpCode, data: &mut [u8],  size: usize) -> Result<()> {
        self.read(op_code, data, size)
    }


    fn get_status(&mut self) -> Result<u8> {
        let mut buff = [0u8;1];

        buff[0] = 0x00;

        self.reg_r(SX1261OpCode::SX1261_GET_STATUS, &mut buff, 1)?;

        let status = buff[0] & 0x7E; /* ignore bit 0 & 7 */

        debug!("SX1261: get_status: 0x{:02X} (0x{:02X})\n", status, buff[0]);

        Ok(status)
    }


    fn check_status( &mut self, expected_status: u8) -> Result<()> {
        let status   = self.get_status()?;

        if status != expected_status {
            error!("ERROR: SX1261 status is not as expected: got:0x{:02X} expected:0x{:02X}\n", status, expected_status);
            return Err(Error::LGW_HAL_ERROR.into())
        }

        Ok(())
    }

    pub fn load_pram(&mut self) -> Result<()> {
        let mut buff = [0u8;32];
    
        /* Set Radio in Standby mode */
        buff[0] = SX1261StandbyModes::SX1261_STDBY_RC as u8;
        self.reg_w(SX1261OpCode::SX1261_SET_STANDBY, &buff, 1)?;
    
        /* Check status */
        self.check_status(SX1261StatusMode::SX1261_STATUS_MODE_STBY_RC as u8 | SX1261StatusCommandStatus::SX1261_STATUS_READY as u8)?;


        let ver = self.pram_get_version()?;

        debug!("SX1261: PRAM version: {}", ver);
    
        /* Enable patch update */
        buff[0] = 0x06;
        buff[1] = 0x10;
        buff[2] = 0x10;
        self.reg_w( SX1261OpCode::SX1261_WRITE_REGISTER, &buff, 3)?;
    
        /* Load patch */
        for i in 0 .. PRAM_COUNT {
            let val = PRAM[i];
            let addr = 0x8000 + 4*i;
    
            buff[0] = ((addr >> 8) & 0xFF) as u8;
            buff[1] = ((addr >> 0) & 0xFF) as u8;
            buff[2] = ((val >> 24) & 0xFF) as u8;
            buff[3] = ((val >> 16) & 0xFF) as u8;
            buff[4] = ((val >> 8)  & 0xFF) as u8;
            buff[5] = ((val >> 0)  & 0xFF) as u8;

            self.reg_w(SX1261OpCode::SX1261_WRITE_REGISTER, &buff, 6)?;
        }
    
        /* Disable patch update */
        buff[0] = 0x06;
        buff[1] = 0x10;
        buff[2] = 0x00;
        self.reg_w( SX1261OpCode::SX1261_WRITE_REGISTER, &buff, 3)?;
    
        /* Update pram */
        buff[0] = 0;
        self.reg_w(SX1261OpCode::SX1261_0XD9 , &buff, 0)?;
    
        let ver = self.pram_get_version()?;
        
        debug!("SX1261: PRAM version: {}", ver);
    
        /* Check PRAM version (only last 4 bytes) 
        if (strncmp(pram_version + 11, sx1261_pram_version_string, 4) != 0) {
            debug!("ERROR: SX1261 PRAM version mismatch (got:%s expected:%s)\n", pram_version + 11, sx1261_pram_version_string);
            return -1;
        }
        */
       Ok(())
    }


    fn pram_get_version(&mut self) -> Result<String> {
        let mut buff = [0u8;3 + SX1261_PRAM_VERSION_FULL_SIZE];

    
        /* Get version string (15 bytes) at address 0x320 */
        buff[0] = 0x03;
        buff[1] = 0x20;
        buff[2] = 0x00; /* status */
        
        self.reg_r(SX1261OpCode::SX1261_READ_REGISTER, &mut buff, 18)?;

    
        /* Return full PRAM version string */
        buff[18] = 0;

        let version_str = String::from_utf8_lossy(&buff[3..]).to_string();
        Ok(version_str)
    }



    pub fn connect(&self) -> Result<()> {
        Ok(())
    }

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

    pub fn disconnect(&self) -> Result<()> {
        Ok(())    
    }


    pub fn calibrate(&mut self, freq_hz: u32) -> Result<()> {
        let mut buff = [0u8;16];
    
        buff[0] = 0x00;

        self.reg_r(SX1261OpCode::SX1261_GET_STATUS, &mut buff, 1)?;
    
        /* Run calibration */
        if (freq_hz > 430000000) && (freq_hz < 440000000) {
            buff[0] = 0x6B;
            buff[1] = 0x6F;
        } else if (freq_hz > 470000000) && (freq_hz < 510000000) {
            buff[0] = 0x75;
            buff[1] = 0x81;
        } else if (freq_hz > 779000000) && (freq_hz < 787000000) {
            buff[0] = 0xC1;
            buff[1] = 0xC5;
        } else if (freq_hz > 863000000) && (freq_hz < 870000000) {
            buff[0] = 0xD7;
            buff[1] = 0xDB;
        } else if (freq_hz > 902000000) && (freq_hz < 928000000) {
            buff[0] = 0xE1;
            buff[1] = 0xE9;
        } else {
            error!("ERROR: failed to calibrate sx1261 radio, frequency range not supported ({})", freq_hz);
            return Err(Error::LGW_REG_ERROR.into())
        }

        self.reg_w(SX1261OpCode::SX1261_CALIBRATE_IMAGE, &buff, 2)?;
    
        /* Wait for calibration to complete */
        sleep(Duration::from_millis(10));
    
        buff[0] = 0x00;
        buff[1] = 0x00;
        buff[2] = 0x00;

        self.reg_r(SX1261OpCode::SX1261_GET_DEVICE_ERRORS, &mut buff, 3)?;

        //#define TAKE_N_BITS_FROM(b, p, n) (((b) >> (p)) & ((1 << (n)) - 1))

        if ( buff[2] >> 4 ) & ( (1 << 1 ) - 1) != 0 {
            error!("ERROR: sx1261 Image Calibration Error\n");
            return Err(Error::LGW_HAL_ERROR.into())
        }
    
        Ok(())
    }
    
    
    pub fn setup(&mut self) -> Result<()> {
        let mut buff = [0u8;32];
    
        /* Set Radio in Standby mode */
        buff[0] = SX1261StandbyModes::SX1261_STDBY_RC as u8;
        self.reg_w(SX1261OpCode::SX1261_SET_STANDBY, &buff, 1)?;

        
        sleep(Duration::from_millis(10));

        /* Check radio status */
        self.check_status(SX1261StatusMode::SX1261_STATUS_MODE_STBY_RC as u8 | SX1261StatusCommandStatus::SX1261_STATUS_READY as u8)?;
    
        /* Set Buffer Base address */
        buff[0] = 0x80;
        buff[1] = 0x80;
        self.reg_w(SX1261OpCode::SX1261_SET_BUFFER_BASE_ADDRESS, &buff, 2)?;
    
        /* sensi adjust */
        buff[0] = 0x08;
        buff[1] = 0xAC;
        buff[2] = 0xCB;

        self.reg_w(SX1261OpCode::SX1261_WRITE_REGISTER, &buff, 3)?;
    
        debug!("SX1261: setup for LBT / Spectral Scan done\n");
    
        Ok(())
    }

    pub fn spectral_scan_start(&mut self, nb_scan: u16) -> Result<()> {
        
        let mut buff = [0u8;4]; /* 66 bytes for spectral scan results + 2 bytes register address + 1 dummy byte for reading */

        /* Start spectral scan */
        buff[0] = ((nb_scan >> 8) & 0xFF) as u8; /* nb_scan MSB */
        buff[1] = ((nb_scan >> 0) & 0xFF) as u8; /* nb_scan LSB */
        buff[2] = 11; /* interval between scans - 8.2 us */

        if let Err(e) = self.reg_w(SX1261OpCode::SX1261_0X9B, &buff, 4) {
            error!(Error=%e, "Unable to start spectral scan");
            return Err(Error::LGW_REG_ERROR.into());
        }

        trace!("INFO: Spectral Scan started...\n");

        Ok(())
    }



    pub fn spectral_scan_get_results(&mut self,  rssi_offset: i8) -> Result<([i16;33], [u16;33])>{

        let mut buff = [0u8;69]; /* 66 bytes for spectral scan results + 2 bytes register address + 1 dummy byte for reading */

        /* Get the results (66 bytes) */
        buff[0] = 0x04;
        buff[1] = 0x01;
        buff[2] = 0x00; /* dummy */

        for i in  3 .. 69 {
            buff[i] = 0x00;
        }

        self.reg_r(SX1261OpCode::SX1261_READ_REGISTER, &mut buff, 66 + 3)?;

        let mut levels_dbm = [0i16;33];
        let mut results = [0u16;33];

        /* Copy the results in the given buffers */
        /* The number of points measured ABOVE each threshold */
        for i in 0 .. 32 as usize {
            levels_dbm[i] = -(i as i16*4 ) + rssi_offset as i16;
            results[i] = ((buff[3 + i*2] as u16) << 8) | (buff[3 + i*2 + 1] as u16);
        }

        /* The number of points measured BELOW the lower threshold */
        levels_dbm[32] = -31i16*4 + rssi_offset as i16;
        results[32] = ((buff[3 + 32*2] as u16) << 8) + buff[3 + 32*2 + 1] as u16;


        Ok((levels_dbm, results))
    }

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

    pub fn spectral_scan_status(&mut self) -> Result<LgwSpectralScanStatus> {

        let mut buff = [0u8;16];

        /* Get status */
        buff[0] = 0x07;
        buff[1] = 0xCD;
        buff[2] = 0x00; /* dummy */
        buff[3] = 0x00; /* read value holder */
        
        self.reg_r(SX1261OpCode::SX1261_READ_REGISTER, &mut buff, 4)?;

        let status = match buff[3] {
            0x00 => LgwSpectralScanStatus::LGW_SPECTRAL_SCAN_STATUS_NONE,
            0x0F => LgwSpectralScanStatus::LGW_SPECTRAL_SCAN_STATUS_ON_GOING,
            0xF0 => LgwSpectralScanStatus::LGW_SPECTRAL_SCAN_STATUS_ABORTED,
            0xFF => LgwSpectralScanStatus::LGW_SPECTRAL_SCAN_STATUS_COMPLETED,
            _ => LgwSpectralScanStatus::LGW_SPECTRAL_SCAN_STATUS_UNKNOWN
        };

        Ok(status)
    }

    pub fn spectral_scan_abort(&mut self) -> Result<()>{

        let mut buff = [0u8;3];
        /* Disable LBT */
        buff[0] = 0x08;
        buff[1] = 0x9B;
        buff[2] = 0x00;
        self.reg_w(SX1261OpCode::SX1261_WRITE_REGISTER, &buff, 3)?;

        trace!("SX1261: spectral scan aborted\n");

        Ok(())
    }

    
    pub fn set_rx_params(&mut self,  freq_hz: u32,  bandwidth: u8) -> Result<()> {

        let mut buff= [0u8;16];
    
        /* Set SPI write bulk mode to optimize speed on USB */
        self.set_write_mode(EComWriteMode::LGW_COM_WRITE_MODE_BULK);

    
        /* Disable any on-going spectral scan to free the sx1261 radio for LBT */
        self.spectral_scan_abort()?;
    
        /* Set FS */
        self.reg_w(SX1261OpCode::SX1261_SET_FS, &buff, 0)?;

    
        /* Set frequency */
        let freq_reg = sx1261_freq_to_reg(freq_hz);
        buff[0] = (freq_reg >> 24) as u8;
        buff[1] = (freq_reg >> 16) as u8;
        buff[2] = (freq_reg >> 8) as u8;
        buff[3] = (freq_reg >> 0) as u8;

        self.reg_w(SX1261OpCode::SX1261_SET_RF_FREQUENCY, &buff, 4)?;
    
        /* Configure RSSI averaging window */
        buff[0] = 0x08;
        buff[1] = 0x9B;
        buff[2] = 0x05 << 2;

        self.reg_w(SX1261OpCode::SX1261_WRITE_REGISTER, &buff, 3)?;
    
        /* Set PacketType */
        buff[0] = 0x00; /* FSK */
        self.reg_w(SX1261OpCode::SX1261_SET_PACKET_TYPE, &buff, 1)?;
    
        /* Set GFSK bandwidth */
        let fsk_bw_reg = match bandwidth {
            BW_125KHZ => 0x0a,
            BW_250KHZ => 0x09,
            _ => {
                error!("ERROR: %s: Cannot configure sx1261 for bandwidth {}", bandwidth);
                return Err(Error::LGW_REG_ERROR.into())
            }
        };

        /* Set modulation params for FSK */
        buff[0] = 0;    // BR
        buff[1] = 0x14; // BR
        buff[2] = 0x00; // BR
        buff[3] = 0x00; // Gaussian BT disabled
        buff[4] = fsk_bw_reg;
        buff[5] = 0x02; // FDEV
        buff[6] = 0xE9; // FDEV
        buff[7] = 0x0F; // FDEV

        self.reg_w(SX1261OpCode::SX1261_SET_MODULATION_PARAMS, &buff, 8)?;
    
        /* Set packet params for FSK */
        buff[0] = 0x00; /* Preamble length MSB */
        buff[1] = 0x20; /* Preamble length LSB 32 bits*/
        buff[2] = 0x05; /* Preamble detector lenght 16 bits */
        buff[3] = 0x20; /* SyncWordLength 32 bits*/
        buff[4] = 0x00; /* AddrComp disabled */
        buff[5] = 0x01; /* PacketType variable size */
        buff[6] = 0xff; /* PayloadLength 255 bytes */
        buff[7] = 0x00; /* CRCType 1 Byte */
        buff[8] = 0x00; /* Whitening disabled*/
        self.reg_w(SX1261OpCode::SX1261_SET_PACKET_PARAMS, &buff, 9)?;
    
        /* Set Radio in Rx continuous mode */
        buff[0] = 0xFF;
        buff[1] = 0xFF;
        buff[2] = 0xFF;
        self.reg_w(SX1261OpCode::SX1261_SET_RX, &buff, 3)?;
    
        /* Flush write (USB BULK mode) */
        self.flush()?;
       
    
        /* Setting back to SINGLE BULK write mode */
        self.set_write_mode(EComWriteMode::LGW_COM_WRITE_MODE_SINGLE);

    
        trace!("SX1261: RX params set to {:} Hz (bw:0x%{:02X})\n", freq_hz, bandwidth);
    
        Ok(())
    }
    
}


