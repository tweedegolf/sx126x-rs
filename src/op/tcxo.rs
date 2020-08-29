#[repr(u8)]
#[derive(Copy, Clone)]
pub enum TcxoVoltage {
    Volt1_6 = 0x00,
    Volt1_7 = 0x01,
    Volt1_8 = 0x02,
    Volt2_2 = 0x03,
    Volt2_4 = 0x04,
    Volt2_7 = 0x05,
    Volt3_0 = 0x06,
    Volt3_3 = 0x07,
}

#[derive(Copy, Clone)]
pub struct TcxoDelay {
    inner: [u8; 3],
}

impl Into<[u8; 3]> for TcxoDelay {
    fn into(self) -> [u8; 3] {
        self.inner
    }
}

impl TcxoDelay {
    pub const fn from_us(us: u32) -> Self {
        let inner = divide!(us, 15.625);
        let inner = inner.to_le_bytes();
        let inner = [inner[2], inner[1], inner[0]];
        Self { inner }
    }
}
