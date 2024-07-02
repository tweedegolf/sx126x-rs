#[derive(Copy, Clone)]
pub struct CalibParam {
    inner: u8,
}

impl From<CalibParam> for u8 {
    fn from(val: CalibParam) -> Self {
        val.inner
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
        let inner = (rc64k_en as u8)
            | (rc13_en as u8) << 1
            | (pll_en as u8) << 2
            | (adc_pulse_en as u8) << 3
            | (adc_bulk_n_en as u8) << 4
            | (adc_bulk_p_en as u8) << 5
            | (image_en as u8) << 6;
        Self { inner }
    }

    pub const fn all() -> Self {
        Self::new(true, true, true, true, true, true, true)
    }
}

#[derive(Copy, Clone)]
#[repr(u16)]
pub enum CalibImageFreq {
    MHz430_440 = 0x6B_6F,
    MHz470_510 = 0x75_81,
    MHz779_787 = 0xC1_C5,
    MHz863_870 = 0xD7_DB,
    MHz902_928 = 0xE1_E9,
}

impl From<CalibImageFreq> for [u8; 2] {
    fn from(val: CalibImageFreq) -> Self {
        (val as u16).to_be_bytes()
    }
}

impl CalibImageFreq {
    pub const fn from_rf_frequency(rf_frequency: u32) -> Self {
        match rf_frequency / 1000000 {
            902..=928 => Self::MHz902_928,
            863..=870 => Self::MHz863_870,
            779..=787 => Self::MHz779_787,
            470..=510 => Self::MHz470_510,
            430..=440 => Self::MHz430_440,
            _ => Self::MHz902_928, // Default
        }
    }
}
