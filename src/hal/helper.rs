

pub fn wait_ms(ms:u32) {
    std::thread::sleep(std::time::Duration::from_millis(ms as u64));
}