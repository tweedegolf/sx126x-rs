#[derive(Copy, Clone)]
pub struct RxTxTimeout {
    inner: [u8; 3],
}

impl Into<[u8; 3]> for RxTxTimeout {
    fn into(self) -> [u8; 3] {
        self.inner
    }
}

impl RxTxTimeout {
    pub const fn from_ms(ms: u32) -> Self {
        let inner = ms << 6;
        let inner = inner.to_le_bytes();
        let inner = [inner[2], inner[1], inner[0]];
        Self { inner }
    }
}

impl From<u32> for RxTxTimeout {
    fn from(val: u32) -> Self {
        let bytes = val.to_be_bytes();
        Self {
            inner: [bytes[0], bytes[1], bytes[2]],
        }
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

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum DeviceSel {
    SX1262 = 0x00,
    SX1261 = 0x01,
}

pub struct PaConfig {
    pa_duty_cycle: u8,
    hp_max: u8,
    device_sel: DeviceSel,
}

impl Into<[u8; 4]> for PaConfig {
    fn into(self) -> [u8; 4] {
        [self.pa_duty_cycle, self.hp_max, self.device_sel as u8, 0x01]
    }
}

impl Default for PaConfig {
    fn default() -> Self {
        Self {
            pa_duty_cycle: 0x00,
            hp_max: 0x00,
            device_sel: DeviceSel::SX1262,
        }
    }
}

impl PaConfig {
    pub fn set_pa_duty_cycle(mut self, pa_duty_cycle: u8) -> Self {
        self.pa_duty_cycle = pa_duty_cycle;
        self
    }

    pub fn set_hp_max(mut self, hp_max: u8) -> Self {
        self.hp_max = hp_max;
        self
    }

    pub fn set_device_sel(mut self, device_sel: DeviceSel) -> Self {
        self.device_sel = device_sel;
        self
    }
}

#[derive(Debug)]
pub struct RxBufferStatus {
    payload_length_rx: u8,
    rx_start_buffer_pointer: u8,
}

impl From<[u8; 2]> for RxBufferStatus {
    fn from(raw: [u8; 2]) -> Self {
        Self {
            payload_length_rx: raw[0],
            rx_start_buffer_pointer: raw[1],
        }
    }
}

impl RxBufferStatus {
    pub fn payload_length_rx(&self) -> u8 {
        self.payload_length_rx
    }

    pub fn rx_start_buffer_pointer(&self) -> u8 {
        self.rx_start_buffer_pointer
    }
}
