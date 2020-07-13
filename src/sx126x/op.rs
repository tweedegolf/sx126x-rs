#[repr(u8)]
#[derive(Copy, Clone)]
pub enum PacketType {
    GFSK = 0x00,
    LORA = 0x01,
}
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum StandbyConfig {
    StbyRc = 0x00,
    StbyXOSC = 0x01,
}

#[derive(Copy, Clone)]
pub struct Status {
    inner: u8,
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum ChipMode {
    StbyRC = 0x02,
    StbyXOSC = 0x03,
    FS = 0x04,
    RX = 0x05,
    TX = 0x06,
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum CommandStatus {
    DataAvaiable = 0x02,
    CommandTimeout = 0x03,
    CommandProcessingError = 0x04,
    FailureToExecute = 0x05,
    CommandTxDone = 0x06,
}

impl From<u8> for Status {
    fn from(b: u8) -> Self {
        Self { inner: b }
    }
}

impl Status {
    pub fn chip_mode(&self) -> ChipMode {
        use ChipMode::*;
        match (self.inner & 0x70) >> 4 {
            0x02 => StbyRC,
            0x03 => StbyXOSC,
            0x04 => FS,
            0x05 => RX,
            0x06 => TX,
            _ => unreachable!(),
        }
    }

    pub fn command_status(self) -> CommandStatus {
        use CommandStatus::*;
        match (self.inner & 0x0E) >> 1 {
            0x02 => DataAvaiable,
            0x03 => CommandTimeout,
            0x04 => CommandProcessingError,
            0x05 => FailureToExecute,
            0x06 => CommandTxDone,
            _ => unreachable!(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct CalibParam {
    inner: u8,
}

impl Into<u8> for CalibParam {
    fn into(self) -> u8 {
        self.inner
    }
}

impl From<u8> for CalibParam {
    fn from(val: u8) -> Self {
        Self { inner: val & 0x7F }
    }
}

impl CalibParam {
    pub const fn new(
        rc64k_en: bool,
        rc13_en: bool,
        pll_en: bool,
        adc_pulse_en: bool,
        adc_bulk_n_en: bool,
        adc_bulk_p_en: bool,
        image_en: bool,
    ) -> Self {
        let inner = (rc64k_en as u8) << 0
            | (rc13_en as u8) << 1
            | (pll_en as u8) << 2
            | (adc_pulse_en as u8) << 3
            | (adc_bulk_n_en as u8) << 4
            | (adc_bulk_p_en as u8) << 5
            | (image_en as u8) << 6;
        Self { inner }
    }
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum TcxoVoltage {
    Volt_1_6 = 0x00,
    Volt_1_7 = 0x01,
    Volt_1_8 = 0x02,
    Volt_2_2 = 0x03,
    Volt_2_4 = 0x04,
    Volt_2_7 = 0x05,
    Volt_3_0 = 0x06,
    Volt_3_3 = 0x07,
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
    pub fn from_millis(millis: u32) -> Self {
        let inner = (millis as f32 / 15.625) as u32;
        let inner = inner.to_be_bytes();
        let inner = [inner[2], inner[1], inner[0]];
        Self { inner }
    }
}

#[derive(Copy, Clone)]
pub struct DeviceErrors {
    inner: u16,
}

impl From<u16> for DeviceErrors {
    fn from(val: u16) -> Self {
        Self { inner: val }
    }
}

impl DeviceErrors {
    pub fn rc64k_calib_err(self) -> bool {
        (self.inner & 1 << 0) > 0
    }

    pub fn rc13m_calib_err(self) -> bool {
        (self.inner & 1 << 1) > 0
    }

    pub fn pll_calib_err(self) -> bool {
        (self.inner & 1 << 2) > 0
    }

    pub fn adc_calib_err(self) -> bool {
        (self.inner & 1 << 3) > 0
    }

    pub fn img_calib_err(self) -> bool {
        (self.inner & 1 << 4) > 0
    }

    pub fn xosc_start_err(self) -> bool {
        (self.inner & 1 << 5) > 0
    }

    pub fn pll_lock_err(self) -> bool {
        (self.inner & 1 << 6) > 0
    }

    pub fn pa_ramp_err(self) -> bool {
        (self.inner & 1 << 8) > 0
    }
}
