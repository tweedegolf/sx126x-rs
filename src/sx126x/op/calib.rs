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