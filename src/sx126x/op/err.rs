#[derive(Copy, Clone)]
pub struct DeviceErrors {
    inner: u16,
}

impl core::fmt::Debug for DeviceErrors {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let rc64k_calib_err = self.rc64k_calib_err();
        let rc13m_calib_err = self.rc13m_calib_err();
        let pll_calib_err = self.pll_calib_err();
        let adc_calib_err = self.adc_calib_err();
        let img_calib_err = self.img_calib_err();
        let xosc_start_err = self.xosc_start_err();
        let pll_lock_err = self.pll_lock_err();
        let pa_ramp_err = self.pa_ramp_err();
        write!(
            f,
            "{{inner: {:#016b},rc64k_calib_err: {}, rc13m_calib_err: {}, pll_calib_err: {}, adc_calib_err: {}, img_calib_err: {}, xosc_start_err: {}, pll_lock_err: {}, pa_ramp_err: {}}}",
            self.inner,
            rc64k_calib_err,
            rc13m_calib_err,
            pll_calib_err,
            adc_calib_err,
            img_calib_err,
            xosc_start_err,
            pll_lock_err,
            pa_ramp_err,
        )
    }
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
