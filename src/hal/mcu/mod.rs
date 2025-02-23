pub mod command;
mod helper;


use command::{*};
use rand::Rng;
use tracing::{debug, error, trace};
use super::loragw_usb_com::UsbComPort;
use anyhow::{anyhow,Result};
use crate::hal::{error::Error, loragw_com::LgwSpiMuxTarget};

const MAX_SIZE_COMMAND:usize =  4200;
const MAX_SPI_COMMAND:usize  =   MAX_SIZE_COMMAND - CommandOrderOffset::Data as usize - 1 ;
const LGW_USB_BURST_CHUNK:usize =  4096 ;

const HEADER_CMD_SIZE:usize = 4;


#[derive(Debug)]
pub struct PingInfo {
    pub unique_id_high: u32,
    pub unique_id_mid: u32,
    pub unique_id_low: u32,
    pub version: String /* format is V00.00.00\0 */
}

#[derive(Debug)]
pub struct Status{
    pub system_time_ms :u32,
    pub temperature: f32
}


impl Status {
    pub fn from_get_status(hdr: &[u8], payload:&[u8]) -> Result<Status> {
        if hdr[CommandOrderOffset::Cmd as usize] != OrderId::AckGetStatus as u8 {
            return Err(anyhow!("ERROR: received wrong ACK type {:}\n", hdr[CommandOrderOffset::Cmd as usize]));
        }

        Ok(Status {
            system_time_ms: helper::bytes_be_to_uint32_le(&payload[ECmdOffsetAckGetStatus::AckGetStatusSystemTime31_24 as usize..])?,
            temperature: (((payload[ECmdOffsetAckGetStatus::AckGetStatusTemperature15_8 as usize] as u16) << 8) | (payload[ECmdOffsetAckGetStatus::AckGetStatusTemperature7_0 as usize] as u16)) as f32 / 100.0
        })
    }
}

impl PingInfo {
    pub fn from_ping(hdr: &[u8], payload:&[u8]) -> Result<PingInfo> {

        if hdr[CommandOrderOffset::Cmd as usize] != OrderId::AckPing as u8 {
            return Err(anyhow!("ERROR: received wrong ACK type {:}\n", hdr[CommandOrderOffset::Cmd as usize]));
        }

        Ok(PingInfo {
            unique_id_high: helper::bytes_be_to_uint32_le(&payload[ECmdOffsetAckPing::AckPingUniqueId0 as usize..])?,
            unique_id_mid: helper::bytes_be_to_uint32_le(&payload[ECmdOffsetAckPing::AckPingUniqueId4 as usize..])?,
            unique_id_low: helper::bytes_be_to_uint32_le(&payload[ECmdOffsetAckPing::AckPingUniqueId8 as usize..])?,
            version: String::from_utf8(payload[ECmdOffsetAckPing::AckPingVersion0 as usize..].to_vec())?
        })
    }
    
}

#[derive(Debug)]
pub struct SpiReqBulk {
    pub size: usize,
    pub nb_req: u8,
    pub buffer: [u8; LGW_USB_BURST_CHUNK],
    
}

#[derive(Debug)]
pub struct Mcu {
    pub com: UsbComPort,
    pub ping_info: Option<PingInfo>,
    pub status: Option<Status>,
    pub spi_req_bulk: SpiReqBulk,
    pub lgw_write_mode: EComWriteMode,
    pub lgw_spi_req_nb: u8
}

impl Mcu {
    pub fn new () -> Self {
        Self {
            com: UsbComPort::new(),
            ping_info: None,
            status: None,
            spi_req_bulk: SpiReqBulk {
                size: 0,
                nb_req: 0,
                buffer: [0u8; LGW_USB_BURST_CHUNK]
            },
            lgw_write_mode: EComWriteMode::LGW_COM_WRITE_MODE_SINGLE,
            lgw_spi_req_nb: 0
        }
    }
    
    pub fn open(&mut self, port_name: String) -> Result<()> {
        self.com.open(port_name)?;

        let ping_info = self.mcu_ping()?;

        self.ping_info = Some(ping_info);
        self.status = Some(self.get_mcu_status()?);

        self.mcu_reset()?;

        Ok(())
    }

    pub fn close(&mut self) {
        self.com.close();
    }
    
    fn write_req(&mut self, order:OrderId, payload: &[u8]) ->Result<()> {
        let buf_w = &mut [0u8; HEADER_CMD_SIZE];

        if payload.len() > MAX_SPI_COMMAND {
            return Err(anyhow!("Data too long"));
        }

        let mut rng = rand::thread_rng();
        buf_w[0] = rng.gen::<u8>();
        buf_w[1] = (payload.len() >> 8) as u8; // MSB
        buf_w[2] = (payload.len() & 0xFF) as u8; // LSB
        buf_w[3] = order as u8;

        let _n = self.com.write(buf_w)?;

        if _n < HEADER_CMD_SIZE {
            return Err(anyhow!("Failed to write to serial port"));
        }
        //wait_ms(10);
        if payload.len() > 0 {
            let _n = self.com.write(payload)?;
            if _n < payload.len() {
                return Err(anyhow!("Failed to write to serial port"));
            }
        }

        trace!("write_req done");
        Ok(())
    }

    fn read_ack(&mut self, hdr: &mut [u8], buf: &mut [u8]) -> Result<()> {
        
        self.com.read_exact(hdr).map_err(|e| anyhow!("Failed to read HDR: {}", e))?;
        
        trace!("READ HD: {:02X?}", hdr);

        if  hdr[CommandOrderOffset::Cmd as usize] < OrderId::AckPing as u8 ||
            hdr[CommandOrderOffset::Cmd as usize] > OrderId::AckMultipleSpi as u8 
        {
            return Err(anyhow!("ERROR: received wrong ACK type {:}\n", hdr[CommandOrderOffset::Cmd as usize]))
        }

        let size = (((hdr[1] as u16) << 8) | (hdr[2] as u16)) as usize;
        trace!("READ SIZE: {:}", size);

        let mut read_buf = vec![0u8; size];
        if size > 0 {
            self.com.read_exact(&mut read_buf).map_err(|e| anyhow!("Failed to read Payload: {}", e))?;
            buf[..size].copy_from_slice(&read_buf);
        }

        Ok(())
    }

    pub fn mcu_ping(&mut self) -> Result<PingInfo> {
        let buf_w = [0u8; 0];
        let mut buf_r = [0u8; ECmdOffsetAckPing::AckPingSize as usize];
        let mut buf_hdr = [0u8; HEADER_CMD_SIZE];

        self.write_req(OrderId::ReqPing, &buf_w)?;
        self.read_ack(&mut buf_hdr, &mut buf_r)?;

        Ok(PingInfo::from_ping(&buf_hdr, &buf_r)?)
    }

    pub fn get_mcu_status(&mut self) -> Result<Status> {
        let buf_w = [0u8; 0];
        let mut buf_r = [0u8; ECmdOffsetAckGetStatus::AckGetStatusSize as usize];
        let mut buf_hdr = [0u8; HEADER_CMD_SIZE];

        self.write_req(OrderId::ReqGetStatus, &buf_w)?;
        self.read_ack(&mut buf_hdr, &mut buf_r)?;


        Ok(Status::from_get_status(&buf_hdr, &buf_r)?)
    }

    fn decode_ack_gpio_access(hdr: &[u8], payload: &[u8]) -> Result<u8> {
        if hdr[CommandOrderOffset::Cmd as usize] != OrderId::AckWriteGpio as u8 {
            return Err(anyhow!("ERROR: received wrong ACK type {:}\n", hdr[CommandOrderOffset::Cmd as usize]));
        }

        Ok(payload[ECmdOffsetAckGpioWrite::AckGpioWriteStatus as usize])
    }

    fn cmd_get_size( bytes:& [u8]) -> usize {
        return (((bytes[1] as u16) << 8) | (bytes[2] as u16)) as usize;
    }

    fn decode_ack_spi_bulk(hdr: &[u8], payload: &[u8]) -> Result<()> {
        let mut i = 0;

        if hdr.is_empty() || payload.is_empty() {
            return Err(anyhow!("ERROR: invalid parameter"));
        }

        if hdr[CommandOrderOffset::Cmd as usize] != OrderId::AckMultipleSpi as u8 {
            return Err(anyhow!("ERROR: wrong ACK type for ACK_MULTIPLE_SPI (expected:0x{:02X}, got 0x{:02X})", OrderId::AckMultipleSpi as u8, hdr[CommandOrderOffset::Cmd as usize]));
        }

        while i < Self::cmd_get_size(hdr) {
            let req_id = payload[i];
            let req_type = payload[i + 1];
            if req_type != MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE && req_type != MCU_SPI_REQ_TYPE_READ_WRITE {
                return Err(anyhow!("ERROR: wrong type for SPI request {} (0x{:02X})", req_id, req_type));
            }
            let req_status = payload[i + 2];
            if req_status != 0 {
                return Err(anyhow!("ERROR: SPI request {} failed with {:?}", req_id, req_status));
            }

            if req_type == MCU_SPI_REQ_TYPE_READ_WRITE {
                let frame_size = ((payload[i + 3] as u16) << 8) | (payload[i + 4] as u16);
                i += 5 + frame_size as usize;
            } else {
                i += 5;
            }
        }

        Ok(())
    }

    pub fn mcu_gpio_write(&mut self, gpio_port: u8, gpio_id: u8, gpio_value: u8) -> Result<()> {
        let mut buf_req = [0u8; ECmdOffsetReqWriteGpio::ReqWriteGpioSize as usize];
        let mut buf_ack = [0u8; ECmdOffsetAckGpioWrite::AckGpioWriteSize as usize];
        let mut buf_hdr = [0u8; HEADER_CMD_SIZE];

        buf_req[ECmdOffsetReqWriteGpio::ReqWriteGpioPort as usize] = gpio_port;
        buf_req[ECmdOffsetReqWriteGpio::ReqWriteGpioPin as usize] = gpio_id;
        buf_req[ECmdOffsetReqWriteGpio::ReqWriteGpioState as usize] = gpio_value;

        self.write_req(OrderId::ReqWriteGpio, &buf_req)?;

        self.read_ack(&mut buf_hdr, &mut buf_ack)?;

        let status = Mcu::decode_ack_gpio_access(&buf_hdr, &buf_ack)?;

        if status != 0 {
            return Err(anyhow!("ERROR: Failed to write GPIO (port:{} id:{} value:{})", gpio_port, gpio_id, gpio_value));
        }

        Ok(())
    }


    pub fn mcu_reset(&mut self) -> Result<()> {
        /* Reset SX1302 */
        self.mcu_gpio_write( 0, 1, 1)?; /*   set PA1 : POWER_EN */
        self.mcu_gpio_write( 0, 2, 1)?; /*   set PA2 : SX1302_RESET active */
        self.mcu_gpio_write( 0, 2, 0)?; /* unset PA2 : SX1302_RESET inactive */
        /* Reset SX1261 (LBT / Spectral Scan) */
        self.mcu_gpio_write( 0, 8, 0)?; /*   set PA8 : SX1261_NRESET active */
        self.mcu_gpio_write( 0, 8, 1)?; /* unset PA8 : SX1261_NRESET inactive */

        Ok(())
    }


    pub fn mcu_spi_write(&mut self, in_out_buf: &mut [u8]) -> Result<()> {

        trace!(in_out_buf=?in_out_buf, "mcu_spi_write");
        /* Check input parameters */
        if in_out_buf.is_empty() {
            return Err(anyhow!("Input buffer is null"));
        }

        self.write_req(OrderId::ReqMultipleSpi, in_out_buf)?;

        let mut buf_hdr = [0u8; HEADER_CMD_SIZE];

        self.read_ack(&mut buf_hdr, in_out_buf)?;

        Mcu::decode_ack_spi_bulk(&buf_hdr, in_out_buf)?;

        Ok(())
    }

    fn spi_req_bulk_insert(bulk_buffer: &mut SpiReqBulk, req: &[u8]) -> Result<()> {
        if bulk_buffer.nb_req == 255 {
            return Err(anyhow!("ERROR: cannot insert a new SPI request in bulk buffer - too many requests"));
        }
    
        if (bulk_buffer.size + req.len()) > LGW_USB_BURST_CHUNK {
            return Err(anyhow!("ERROR: cannot insert a new SPI request in bulk buffer - buffer full"));
        }
    
        bulk_buffer.buffer[bulk_buffer.size..bulk_buffer.size + req.len()].copy_from_slice(req);
    
        bulk_buffer.nb_req += 1;
        bulk_buffer.size += req.len();
    
        Ok(())
    }

    pub fn mcu_spi_store( &mut self, in_out_buf: &[u8]) -> Result<()> {
        if in_out_buf.is_empty() {
            return Err(anyhow!("Input buffer is null"));
        }
    
        Mcu::spi_req_bulk_insert(&mut self.spi_req_bulk, in_out_buf)?;

        Ok(())
    }
    
    pub fn mcu_spi_flush(&mut self) -> Result<()> {
        /* Write pending SPI requests to MCU */
        let buffer_size = self.spi_req_bulk.size;
        if buffer_size == 0 {
            return Ok(());
        }

        let mut buffer_slice = vec![0u8; buffer_size];
        
        buffer_slice.copy_from_slice(&self.spi_req_bulk.buffer[..self.spi_req_bulk.size]);
        
        self.mcu_spi_write(&mut buffer_slice)?;
        
    
        /* Reset bulk storage buffer */
        self.spi_req_bulk.nb_req = 0;
        self.spi_req_bulk.size = 0;
    
        Ok(())
    }

    pub fn mcu_set_write_mode(&mut self, mode:EComWriteMode) {
        self.lgw_write_mode = mode;
    }
}

pub trait McuTrait {
    fn lgw_rb(&mut self, spi_mux_target: LgwSpiMuxTarget, address:u16, data: &mut [u8], size: usize) -> Result<()>;
    fn lgw_wb(&mut self, spi_mux_target: LgwSpiMuxTarget, address:u16, data: &[u8], size: usize) -> Result<()>;
    fn lgw_rmw(&mut self, spi_mux_target:LgwSpiMuxTarget, address:u16,  offs:u8,  leng:u8,  data:u8) -> Result<()>;
    fn lgw_flush(&mut self)->Result<()>;
    fn lgw_disconnect(&mut self) -> Result<()>;
}

impl McuTrait for Mcu {
    fn lgw_disconnect(&mut self) -> Result<()> {


        /* Reset SX1302 before closing */
        let e1 = self.mcu_gpio_write(0, 1, 1); /*   set PA1 : POWER_EN */
        let e2 = self.mcu_gpio_write( 0, 2, 1); /*   set PA2 : SX1302_RESET active */
        let e3 =self.mcu_gpio_write( 0, 2, 0); /* unset PA2 : SX1302_RESET inactive */
        /* Reset SX1261 (LBT / Spectral Scan) */
        let e4 = self.mcu_gpio_write( 0, 8, 0); /*   set PA8 : SX1261_NRESET active */
        let e5 = self.mcu_gpio_write( 0, 8, 1); /* unset PA8 : SX1261_NRESET inactive */
        if e1.is_err() || e2.is_err() || e3.is_err() || e4.is_err() || e5.is_err() {
            error!("ERROR: failed to reset SX1302\n");
            return Err(Error::LGW_USB_ERROR.into());
        }

        /* close file & deallocate file descriptor */

        self.close();


        Ok(())

    }
    fn lgw_flush(&mut self)->Result<()> {


        if self.lgw_write_mode != EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            error!("ERROR: cannot flush in single write mode\n");
            return Err(Error::LGW_COM_ERROR.into());
        }
    
        /* Restore single mode after flushing */
        self.lgw_write_mode = EComWriteMode::LGW_COM_WRITE_MODE_SINGLE;
    
        if self.lgw_spi_req_nb == 0 {
            debug!("INFO: no SPI request to flush\n");
            return Ok(())
        }
    

    
        debug!("INFO: flushing USB write buffer\n");
        if let Err(_) = self.mcu_spi_flush(){
            error!("ERROR: Failed to flush USB write buffer\n");
            return Err(Error::LGW_COM_ERROR.into());
        }
    
        /* reset the pending request number */
        self.lgw_spi_req_nb = 0;
    
        return Ok(())
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
     

        if self.lgw_write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK{
            /* makes no sense to read in bulk mode, as we can't get the result */
            return Err(anyhow!("ERROR: USB READ BURST FAILURE - bulk mode is enabled"))
        } else {
            if let Err(e) = self.mcu_spi_write( &mut in_out_buf) {
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
        in_out_buf[0] = self.lgw_spi_req_nb; /* Req ID */
        in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE as u8; /* Req type */
        in_out_buf[2] = (address >> 8) as u8; /* Register address MSB */
        in_out_buf[3] = (address >> 0) as u8; /* Register address LSB */
        in_out_buf[4] = ((1 << leng) - 1) << offs; /* Register bitmask */
        in_out_buf[5] = data << offs;

        if self.lgw_write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            self.mcu_spi_store(&mut in_out_buf)?;
            self.lgw_spi_req_nb += 1;
        } else {
            self.mcu_spi_write(&mut in_out_buf)?;
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
        in_out_buf[0] = self.lgw_spi_req_nb; /* Req ID */
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

        if self.lgw_write_mode == EComWriteMode::LGW_COM_WRITE_MODE_BULK {
            self.mcu_spi_store(&mut in_out_buf).map_err(|e| anyhow!("ERROR: USB WRITE BURST FAILURE: {:}", e))?;
            self.lgw_spi_req_nb += 1;
        } else {
            self.mcu_spi_write(&mut in_out_buf).map_err(|e| anyhow!("ERROR: USB WRITE BURST FAILURE: {:}", e))?;
        }

        trace!("Note: USB write burst success\n");

        Ok(())
    }
}