use std::{sync::Mutex, time::Duration};

use serialport::{FlowControl, Parity, SerialPort, StopBits};

use tracing::trace;
use anyhow::Result;

#[derive(Debug)]
pub struct UsbComPort {
    pub port: Mutex<Option<Box<dyn SerialPort>>>,
}

impl UsbComPort {
    pub fn new () -> Self {
        Self {
            port: Mutex::new(None)
        }
    }

    pub fn open(&mut self, port_name: String) -> Result<()> {
        let port = serialport::new(port_name.as_str(), 115200)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_millis(5000))
            .open()?;
           
        let mut p = self.port.lock().unwrap();
        *p = Some(port);
        Ok(())
    }

    pub fn close(&mut self) {
        let port = self.port.lock().unwrap();

        drop(port);
        *self.port.lock().unwrap() = None;
    }
}

impl UsbComPort {
    pub fn write(&mut self, data: &[u8]) -> Result<usize, std::io::Error> {
        trace!(size=%data.len(), "WR: {:02X?}", data);
        
        let mut p = self.port.lock().unwrap();
        if let Some(ref mut port) = *p {
            return port.write(data);
        }
        Err(std::io::Error::new(std::io::ErrorKind::Other, "Port not open"))
        
    }

    pub fn read(&mut self, data: &mut [u8]) -> Result<usize, std::io::Error> {
        let mut p = self.port.lock().unwrap();

        if let Some(ref mut port) = *p {
            return port.read(data);
        }
        Err(std::io::Error::new(std::io::ErrorKind::Other, "Port not open"))
    }

    pub fn read_exact(&mut self, data: &mut [u8]) -> Result<(), std::io::Error> {
        
        let mut p = self.port.lock().unwrap();
        if let Some(ref mut port) = *p {
            if let Err(e) = port.read_exact(data) {
                return Err(e);
            }
        } else {
            return Err(std::io::Error::new(std::io::ErrorKind::Other, "Port not open"));
        }
           
        trace!("RD: {:02X?}", data);
        Ok(())
    }
    
}