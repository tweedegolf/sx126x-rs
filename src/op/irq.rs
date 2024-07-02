#[repr(u16)]
#[derive(Copy, Clone)]
pub enum IrqMaskBit {
    None = 0x0000,
    TxDone = 1 << 0,
    RxDone = 1 << 1,
    PreambleDetected = 1 << 2,
    SyncwordValid = 1 << 3,
    HeaderValid = 1 << 4,
    HeaderError = 1 << 5,
    CrcErr = 1 << 6,
    CadDone = 1 << 7,
    CadDetected = 1 << 8,
    Timeout = 1 << 9,
    All = 0xFFFF,
}

#[derive(Copy, Clone)]
pub struct IrqMask {
    inner: u16,
}

impl IrqMask {
    pub const fn none() -> Self {
        Self {
            inner: IrqMaskBit::None as u16,
        }
    }

    pub const fn all() -> Self {
        Self {
            inner: IrqMaskBit::All as u16,
        }
    }

    pub const fn combine(self, bit: IrqMaskBit) -> Self {
        let inner = self.inner | bit as u16;
        Self { inner }
    }
}

impl From<IrqMask> for u16 {
    fn from(val: IrqMask) -> Self {
        val.inner
    }
}

impl From<u16> for IrqMask {
    fn from(mask: u16) -> Self {
        Self { inner: mask }
    }
}

impl Default for IrqMask {
    fn default() -> Self {
        Self::none()
    }
}

#[derive(Copy, Clone)]
pub struct IrqStatus {
    inner: u16,
}

impl From<u16> for IrqStatus {
    fn from(status: u16) -> Self {
        Self { inner: status }
    }
}

impl core::fmt::Debug for IrqStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "IrqStatus {{inner: {:#016b}, tx_done: {}, rx_done: {}, preamble_detected: {}, syncword_valid: {}, header_valid: {}, header_error: {}, crc_err: {}, cad_done: {}, cad_detected: {}, timeout : {}}}",
            self.inner,
            self.tx_done(),
            self.rx_done(),
            self.preamble_detected(),
            self.syncword_valid(),
            self.header_valid(),
            self.header_error(),
            self.crc_err(),
            self.cad_done(),
            self.cad_detected(),
            self.timeout(),
        )
    }
}

impl IrqStatus {
    pub fn tx_done(self) -> bool {
        (self.inner & IrqMaskBit::TxDone as u16) > 0
    }

    pub fn rx_done(self) -> bool {
        (self.inner & IrqMaskBit::RxDone as u16) > 0
    }

    pub fn preamble_detected(self) -> bool {
        (self.inner & IrqMaskBit::PreambleDetected as u16) > 0
    }

    pub fn syncword_valid(self) -> bool {
        (self.inner & IrqMaskBit::SyncwordValid as u16) > 0
    }

    pub fn header_valid(self) -> bool {
        (self.inner & IrqMaskBit::HeaderValid as u16) > 0
    }

    pub fn header_error(self) -> bool {
        (self.inner & IrqMaskBit::HeaderError as u16) > 0
    }

    pub fn crc_err(self) -> bool {
        (self.inner & IrqMaskBit::CrcErr as u16) > 0
    }

    pub fn cad_done(self) -> bool {
        (self.inner & IrqMaskBit::CadDone as u16) > 0
    }

    pub fn cad_detected(self) -> bool {
        (self.inner & IrqMaskBit::CadDetected as u16) > 0
    }

    pub fn timeout(self) -> bool {
        (self.inner & IrqMaskBit::Timeout as u16) > 0
    }
}
