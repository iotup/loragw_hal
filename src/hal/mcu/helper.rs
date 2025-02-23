use anyhow::{anyhow,Result};

pub fn bytes_be_to_uint32_le(bytes: &[u8]) -> Result<u32> {
    if bytes.len() >= 4 {
        /* Big endian to Little Endian */
        Ok(((bytes[0] as u32) << 24) |
        ((bytes[1] as u32) << 16) |
        ((bytes[2] as u32) << 8) |
        (bytes[3] as u32))
    } else {
        return Err(anyhow!("ERROR: bytes_be_to_uint32_le: bytes.len() < 4"));
    }
}