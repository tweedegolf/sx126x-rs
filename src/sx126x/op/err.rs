
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