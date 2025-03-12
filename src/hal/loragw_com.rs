

use serde::{Deserialize, Serialize};
use tracing::{debug, error, info, trace};

use std::fmt;

use crate::hal::{error::Error, loragw_reg::{LOREGS, SX1302_REG_COMMON_VERSION_VERSION}, mcu::command::{ECmdSpiTarget, EComWriteMode, MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE, MCU_SPI_REQ_TYPE_READ_WRITE}};

use super::loragw_sx1302::SX1302;
use anyhow::{Result,anyhow};


const CHUNK_SIZE_MAX:usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum LgwComType {
    LGW_COM_SPI,
    LGW_COM_USB,
    LGW_COM_UNKNOWN
}

#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum  LgwSpiMuxTarget{
    LGW_SPI_MUX_TARGET_SX1302 = 0x00,
    LGW_SPI_MUX_TARGET_RADIOA = 0x01,
    LGW_SPI_MUX_TARGET_RADIOB = 0x02,
}
impl fmt::Display for LgwComType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            LgwComType::LGW_COM_SPI => "LGW_COM_SPI",
            LgwComType::LGW_COM_USB => "LGW_COM_USB",
            LgwComType::LGW_COM_UNKNOWN => "LGW_COM_UNKNOWN",
        };
        write!(f, "{}", s)
    }
}

impl fmt::Display for LgwSpiMuxTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_SX1302 => "LGW_SPI_MUX_TARGET_SX1302",
            LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_RADIOA => "LGW_SPI_MUX_TARGET_RADIOA",
            LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_RADIOB => "LGW_SPI_MUX_TARGET_RADIOB",
        };
        write!(f, "{}", s)
    }
}


pub trait LgwComTrait {
    fn lgw_connect(&mut self) -> Result<()>;
    fn lgw_disconnect(&mut self) -> Result<()>;
    fn lgw_rb(&mut self, spi_mux_target: LgwSpiMuxTarget, address:u16, data: &mut [u8], size: usize) -> Result<()>;
    fn lgw_wb(&mut self, spi_mux_target: LgwSpiMuxTarget, address:u16, data: &[u8], size: usize) -> Result<()>;
    fn lgw_rmw(&mut self, spi_mux_target:LgwSpiMuxTarget, address:u16,  offs:u8,  leng:u8,  data:u8) -> Result<()>;
    fn lgw_flush(&mut self)->Result<()>;
    fn lgw_mem_rb(&mut self,  mem_addr: u16, data:&mut [u8],  size: usize,  fifo_mode: bool)-> Result<()>;
    fn lgw_mem_wb(&mut self,  mem_addr:u16,  data:&[u8],  size:usize)->Result<()>;
}

impl LgwComTrait for SX1302 {

    fn lgw_connect(&mut self) -> Result<()> {
        let mut u= [0u8; 1];

        let mcu = self.mcu.clone();
        let ctx = self.ctx.clone();

        let mut mcu = mcu.write().unwrap();
        

        mcu.open(ctx.read().unwrap().board_cfg.com_path.clone())?;

        /* check SX1302 version */
        self.lgw_rb(LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_SX1302, LOREGS[SX1302_REG_COMMON_VERSION_VERSION as usize].addr, &mut u, 1)?;

        info!("Note: chip version is {:02X} (v{:}.{:})", u[0], (u[0] >> 4) & 0x0F, u[0] & 0x0F) ;

        info!("Note: success connecting the concentrator");
        Ok(())
    }


    fn lgw_disconnect(&mut self) -> Result<()> {

        let mut mcu = self.mcu.write().unwrap();

        /* Reset SX1302 before closing */
        let e1 = mcu.mcu_gpio_write(0, 1, 1); /*   set PA1 : POWER_EN */
        let e2 = mcu.mcu_gpio_write( 0, 2, 1); /*   set PA2 : SX1302_RESET active */
        let e3 =mcu.mcu_gpio_write( 0, 2, 0); /* unset PA2 : SX1302_RESET inactive */
        /* Reset SX1261 (LBT / Spectral Scan) */
        let e4 = mcu.mcu_gpio_write( 0, 8, 0); /*   set PA8 : SX1261_NRESET active */
        let e5 = mcu.mcu_gpio_write( 0, 8, 1); /* unset PA8 : SX1261_NRESET inactive */
        if e1.is_err() || e2.is_err() || e3.is_err() || e4.is_err() || e5.is_err() {
            error!("ERROR: failed to reset SX1302\n");
            return Err(Error::LGW_USB_ERROR.into());
        }

        mcu.close();
        
        Ok(())
    }

    fn lgw_flush(&mut self)->Result<()> {


        if self.write_mode != EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            error!("ERROR: cannot flush in single write mode\n");
            return Err(Error::LGW_COM_ERROR.into());
        }
    
        /* Restore single mode after flushing */
        self.write_mode = EComWriteMode::LGW_COM_WRITE_MODE_SINGLE;
    
        if self.spi_req_nb == 0 {
            debug!("INFO: no SPI request to flush\n");
            return Ok(())
        }
    
        let mut mcu = self.mcu.write().unwrap();

        debug!("INFO: flushing USB write buffer\n");
        if let Err(_) = mcu.mcu_spi_flush(){
            error!("ERROR: Failed to flush USB write buffer\n");
            return Err(Error::LGW_COM_ERROR.into());
        }
    
        /* reset the pending request number */
        self.spi_req_nb = 0;
    
        return Ok(())
    }


    fn lgw_mem_wb(&mut self,  mem_addr:u16,  data:&[u8],  size:usize)->Result<()> {

        let mut chunk_cnt = 0;
        let mut addr = mem_addr;
        let mut sz_todo = size;

        
        debug!("lgw_mem_wb");
        /* check input parameters */
        
        if data.is_empty() || size == 0 {
            error!("ERROR: BURST OF NULL LENGTH\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* write memory by chunks */
        while sz_todo > 0 {
            
            /* full or partial chunk ? */
            let chunk_size = if sz_todo  > CHUNK_SIZE_MAX { CHUNK_SIZE_MAX }   else { sz_todo };
            trace!(chunk_cnt=%chunk_cnt, chunk_size=%chunk_size);

            let bulk = &data[chunk_cnt * CHUNK_SIZE_MAX  .. (chunk_cnt * CHUNK_SIZE_MAX  + chunk_size)];
            /* do the burst write */
            self.lgw_wb(LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_SX1302, addr, &bulk, chunk_size)?;

            /* prepare for next write */
            addr += chunk_size as u16;
            sz_todo -= chunk_size;
            chunk_cnt += 1;  
        }

        Ok(())
    }

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

    fn lgw_mem_rb(&mut self,  mem_addr: u16, data:&mut [u8],  size: usize,  fifo_mode: bool)-> Result<()> {

        let mut chunk_cnt = 0;
        let mut addr = mem_addr;
        let mut sz_todo = size;
        


        if data.is_empty() || size == 0 {
            error!("ERROR: BURST OF NULL LENGTH\n");
            return Err(anyhow!("LGW_REG_ERR"));
        }

        /* read memory by chunks */
        while sz_todo > 0 {
            /* full or partial chunk ? */
            let chunk_size = if sz_todo  > CHUNK_SIZE_MAX as usize  { CHUNK_SIZE_MAX as usize} else { sz_todo  } ;

            let bulk = &mut data[chunk_cnt * CHUNK_SIZE_MAX  .. (chunk_cnt * CHUNK_SIZE_MAX  + chunk_size)];
            /* do the burst read */
            self.lgw_rb(LgwSpiMuxTarget::LGW_SPI_MUX_TARGET_SX1302, addr,  bulk, chunk_size)?;

            /* do not increment the address when the target memory is in FIFO mode (auto-increment) */
            if fifo_mode == false {
                addr += chunk_size as u16;
            }

            /* prepare for next read */
            sz_todo -= chunk_size;
            chunk_cnt += 1;
        }

        Ok(())
    }

    /* Burst (multiple-byte) read */
    fn lgw_rb(&mut self, spi_mux_target: LgwSpiMuxTarget, address:u16, data: &mut [u8], size: usize) -> Result<()> {
      
        let command_size = size + 9;  /* 5 bytes: REQ metadata (MCU), 3 bytes: SPI header (SX1302), 1 byte: dummy*/
        let mut in_out_buf = vec![0u8; command_size];

        /* prepare command */
        /* Request metadata */
        in_out_buf[0] = 0; /* Req ID */
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE ;// MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1302 as u8; /* MCU -> SX1302 */
        in_out_buf[3] = ((size + 4) >> 8) as u8; /* payload size + spi_mux_target + address + dummy byte */
        in_out_buf[4] = ((size + 4) >> 0) as u8; /* payload size + spi_mux_target + address + dummy byte */
        /* RAW SPI frame */
        in_out_buf[5] = spi_mux_target as u8; /* SX1302 -> RADIO_A or RADIO_B */
        in_out_buf[6] = ( 0x00 | ((address >> 8) & 0x7F)) as u8;
        in_out_buf[7] =        ((address >> 0) & 0xFF) as u8;
        in_out_buf[8] = 0x00; /* dummy byte */

        for i in 0..size {
            in_out_buf[i + 9] = data[i];
        }
     

        if self.write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK{
            /* makes no sense to read in bulk mode, as we can't get the result */
            return Err(anyhow!("ERROR: USB READ BURST FAILURE - bulk mode is enabled"))
        } else {
            let mut mcu = self.mcu.write().unwrap();
            if let Err(e) = mcu.mcu_spi_write( &mut in_out_buf) {
                error!("ERROR: USB READ BURST FAILURE");
                return Err(e);
            }
        }

    
        trace!("Note: USB read burst success");
        for i in 0..size {
            data[i] = in_out_buf[9+i]
        }
        
        Ok(())
        
    }
    
    fn lgw_rmw(&mut self, _spi_mux_target:LgwSpiMuxTarget, address:u16,  offs:u8,  leng:u8,  data:u8) -> Result<()> {
        let command_size: usize = 6;
        let mut in_out_buf = vec![0u8;command_size];

        trace!("==> RMW register @ 0x{:04X}, offs:{:} leng:{:} value:0x{:02X}", address, offs, leng, data);

        /* prepare frame to be sent */
        in_out_buf[0] = self.spi_req_nb; /* Req ID */
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE as u8; /* Req type */
        in_out_buf[2] = (address >> 8) as u8; /* Register address MSB */
        in_out_buf[3] = (address >> 0) as u8; /* Register address LSB */
        in_out_buf[4] = ((1 << leng) - 1) << offs; /* Register bitmask */
        in_out_buf[5] = data << offs;

        let mut mcu = self.mcu.write().unwrap();

        if self.write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            mcu.mcu_spi_store(&mut in_out_buf)?;
            self.spi_req_nb += 1;
        } else {
            mcu.mcu_spi_write(&mut in_out_buf)?;
        }

        Ok(())
    }
    
    fn lgw_wb(&mut self, spi_mux_target: LgwSpiMuxTarget, address:u16, data: &[u8], size: usize) -> Result<()> {
    
        let command_size = size + 8; /* 5 bytes: REQ metadata (MCU), 3 bytes: SPI header (SX1302) */
        let mut in_out_buf = vec![0u8;command_size];

        if data.is_empty() {
            return Err(anyhow!("ERROR: empty data array"))
        }

        /* prepare command */
        /* Request metadata */
        in_out_buf[0] = self.spi_req_nb; /* Req ID */
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE as u8; /* Req type */
        in_out_buf[2] = ECmdSpiTarget::MCU_SPI_TARGET_SX1302 as u8; /* MCU -> SX1302 */
        in_out_buf[3] = ((size + 3) >> 8) as u8; /* payload size + spi_mux_target + address */
        in_out_buf[4] = ((size + 3) >> 0) as u8; /* payload size + spi_mux_target + address */
        /* RAW SPI frame */
        in_out_buf[5] = spi_mux_target as u8; /* SX1302 -> RADIO_A or RADIO_B */
        in_out_buf[6] = 0x80 | ((address >> 8) & 0x7F) as u8;
        in_out_buf[7] =        ((address >> 0) & 0xFF) as u8;

        for i in 0..size {
            in_out_buf[i + 8] = data[i];
        }

        let mut mcu = self.mcu.write().unwrap();

        if self.write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            mcu.mcu_spi_store(&mut in_out_buf).map_err(|e| anyhow!("ERROR: USB WRITE BURST FAILURE: {:}", e))?;
            self.spi_req_nb += 1;
        } else {
            mcu.mcu_spi_write(&mut in_out_buf).map_err(|e| anyhow!("ERROR: USB WRITE BURST FAILURE: {:}", e))?;
        }

        trace!("Note: USB write burst success\n");

        Ok(())
    }
}