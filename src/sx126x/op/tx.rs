#[derive(Copy, Clone)]
pub struct TxTimeout {
    inner: [u8; 3],
}

impl Into<[u8; 3]> for TxTimeout {
    fn into(self) -> [u8; 3] {
        self.inner
    }
}

impl TxTimeout {
    pub const fn from_us(us: u32) -> Self {
        let inner = divide!(us, 15.625);
        let inner = inner.to_le_bytes();
        let inner = [inner[2], inner[1], inner[0]];
        Self { inner }
    }
}