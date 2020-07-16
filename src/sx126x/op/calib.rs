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

#[derive(Copy, Clone)]
#[repr(u16)]
pub enum CalibImageFreq {
    MHz430_440 = 0x6B6F,
    MHz470_510 = 0x7581,
    MHz779_787 = 0xC1C5,
    MHz863_870 = 0xD7DB,
    MHz902_928 = 0xE1E9,
}

impl Into<[u8; 2]> for CalibImageFreq {
    fn into(self) -> [u8; 2] {
        (self as u16).to_be_bytes()
    }
}

impl CalibImageFreq {
    pub const fn from_rf_freq(rf_freq: u32) -> Self {
        match rf_freq / 1000000 {
            902..=928 => Self::MHz902_928,
            863..=870 => Self::MHz863_870,
            779..=787 => Self::MHz779_787,
            470..=510 => Self::MHz470_510,
            430..=440 => Self::MHz430_440,
            _ => Self::MHz902_928 // Default
        }
    }
}
