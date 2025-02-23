use anyhow::Result;

use serde::{Deserialize, Serialize};

use super::{mcu::McuTrait, Hal};
use std::fmt;


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

pub trait LoragwComTrait {
    fn lgw_com_open(&mut self)->Result<()>;
    fn lgw_com_w(&mut self, spi_mux_target:LgwSpiMuxTarget, address: u16, value: u8) -> Result<()>;
    fn lgw_com_rmw(&mut self, spi_mux_target:LgwSpiMuxTarget, address: u16, offs: u8, leng: u8, value: u8) -> Result<()>;
    fn lgw_com_r(&mut self, spi_mux_target: LgwSpiMuxTarget, address: u16, value: &mut [u8], size: usize) -> Result<()>;
}

impl LoragwComTrait for Hal {
    fn lgw_com_open(&mut self)->Result<()> {
        let port_name = &self.ctx.board_cfg.com_path;
        self.mcu.open(port_name.to_owned())
    }

    /* Simple read */
    fn lgw_com_r(&mut self, spi_mux_target:LgwSpiMuxTarget,  address: u16,  data: &mut [u8], size: usize) -> Result<()> {
       

        if data.is_empty() {
            return Err(anyhow::anyhow!("ERROR: empty data array"));
        }

        self.mcu.lgw_rb(spi_mux_target, address, data, size)?;

        Ok(())
    }
    
    fn lgw_com_w(&mut self, spi_mux_target:LgwSpiMuxTarget, address: u16, value: u8) -> Result<()> {

        self.mcu.lgw_wb(spi_mux_target, address, &[value;1], 1)?;
        Ok(())
    }
    
    fn lgw_com_rmw(&mut self, spi_mux_target:LgwSpiMuxTarget, address: u16, offs: u8, leng: u8, value: u8) -> Result<()> {

        self.mcu.lgw_rmw( spi_mux_target, address, offs, leng, value)?;
        
        Ok(())
    }

}