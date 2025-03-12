use std::sync::{Arc, RwLock};

use crate::hal::{loragw_sx1250::Sx1250OpCode, mcu::{command::{ECmdSpiTarget, EComWriteMode, MCU_SPI_REQ_TYPE_READ_WRITE}, Mcu}};
use super::sx1261_def::SX1261OpCode;
use anyhow::{Result,anyhow};

#[derive(Debug)]
pub struct SX126x {
    spi_req_nb: u8,
    write_mode: EComWriteMode,
    mcu: Arc<RwLock<Mcu>>
}

impl SX126x {
    pub fn new( mcu: Arc<RwLock<Mcu>> ) -> Self {
        Self {
            spi_req_nb: 0,
            write_mode: EComWriteMode::LGW_COM_WRITE_MODE_SINGLE,
            mcu: mcu
        }
    }

    pub fn usb_w(&mut self, op_code: SX1261OpCode, data: &[u8], size: usize ) -> Result<()> {
        let command_size = size + 6;

        let mut in_out_buf = vec![0u8;command_size];

        in_out_buf[0] = self.spi_req_nb;
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE;
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1261 as u8;

        in_out_buf[3] = (size as u8 + 1) >> 8;
        in_out_buf[4] = (size as u8 + 1) >> 0;

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

    pub fn usb_r(&mut self, op_code: Sx1250OpCode, data: &[u8], size: usize ) -> Result<()> {
        let command_size = size + 6;
        let mut in_out_buf = vec![0u8; command_size];

        in_out_buf[0] = self.spi_req_nb;
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE;
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1261 as u8;

        in_out_buf[3] = (size as u8 + 1) >> 8; /* payload size + op_code */
        in_out_buf[4] = (size as u8 + 1) >> 0; /* payload size + op_code */

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

        Ok(())
    }

    pub fn set_write_mode(&mut self, write_mode:EComWriteMode){
        self.write_mode = write_mode;
    }

    pub fn usb_flush(&mut self) -> Result<()> {

        self.write_mode = EComWriteMode::LGW_COM_WRITE_MODE_SINGLE;

        if self.spi_req_nb == 0 {
            return Ok(())
        }

        let mut mcu = self.mcu.write().unwrap();
        mcu.mcu_spi_flush()?;

        self.spi_req_nb = 0;

        Ok(())

    }
}
