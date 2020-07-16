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

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum RampTime {
    /// 10us
    Ramp10u = 0x00,
    /// 20us
    Ramp20u = 0x01,
    /// 40u
    Ramp40u = 0x02,
    /// 80us
    Ramp80u = 0x03,
    /// 200us
    Ramp200u = 0x04,
    /// 800us
    Ramp800u = 0x05,
    /// 1700us
    Ramp1700u = 0x06,
    /// 3400us
    Ramp3400u = 0x07,
}

pub struct TxParams {
    power_dbm: i8,
    ramp_time: RampTime,
}

impl Default for TxParams {
    fn default() -> Self {
        Self {
            power_dbm: 0,
            ramp_time: RampTime::Ramp10u,
        }
    }    
}

impl Into<[u8; 2]> for TxParams {
    fn into(self) -> [u8; 2] {
        [self.power_dbm as u8, self.ramp_time as u8]
    }
}

impl TxParams {
    /// The output power is defined as power in dBm in a range of
    /// - -17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
    /// - -9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
    /// Selection between high power PA and low power PA is done with the command SetPaConfig and the parameter deviceSel.
    /// By default low power PA and +14 dBm are set.
    pub fn set_power_dbm(mut self, power_dbm: i8) -> Self {
        debug_assert!(power_dbm >= -17);
        debug_assert!(power_dbm < 22);
        self.power_dbm = power_dbm;
        self
    }

    /// Set power ramp time
    pub fn set_ramp_time(mut self, ramp_time: RampTime) -> Self {
        self.ramp_time = ramp_time;
        self
    }
}
