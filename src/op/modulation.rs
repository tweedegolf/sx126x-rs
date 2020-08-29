pub struct ModParams {
    inner: [u8; 8],
}

impl Into<[u8; 8]> for ModParams {
    fn into(self) -> [u8; 8] {
        self.inner
    }
}

pub mod lora {
    use super::ModParams;
    #[derive(Copy, Clone)]
    #[repr(u8)]
    pub enum LoRaSpreadFactor {
        SF5 = 0x05,
        SF6 = 0x06,
        SF7 = 0x07,
        SF8 = 0x08,
        SF9 = 0x09,
        SF10 = 0x0A,
        SF11 = 0x0B,
        SF12 = 0x0C,
    }

    #[derive(Copy, Clone)]
    #[repr(u8)]
    pub enum LoraBandWidth {
        /// 7.81 kHz
        BW7 = 0x00,
        /// 10.42 kHz
        BW10 = 0x08,
        /// 15.63 kHz
        BW15 = 0x01,
        /// 20.83 kHz
        BW20 = 0x09,
        /// 31.25 kHz
        BW31 = 0x02,
        /// 41.67 kHz
        BW41 = 0x0A,
        /// 62.50 kHz
        BW62 = 0x03,
        /// 125 kHz
        BW125 = 0x04,
        /// 250 kHz
        BW250 = 0x05,
        /// 500 kHz
        BW500 = 0x06,
    }

    #[derive(Copy, Clone)]
    #[repr(u8)]
    pub enum LoraCodingRate {
        CR4_5 = 0x01,
        CR4_6 = 0x02,
        CR4_7 = 0x03,
        CR4_8 = 0x04,
    }

    pub struct LoraModParams {
        spread_factor: LoRaSpreadFactor,
        bandwith: LoraBandWidth,
        coding_rate: LoraCodingRate,
        /// LowDataRateOptimize
        low_dr_opt: bool,
    }

    impl Default for LoraModParams {
        fn default() -> Self {
            Self {
                spread_factor: LoRaSpreadFactor::SF7,
                bandwith: LoraBandWidth::BW125,
                coding_rate: LoraCodingRate::CR4_5,
                low_dr_opt: false,
            }
        }
    }

    impl Into<ModParams> for LoraModParams {
        fn into(self) -> ModParams {
            ModParams {
                inner: [
                    self.spread_factor as u8,
                    self.bandwith as u8,
                    self.coding_rate as u8,
                    self.low_dr_opt as u8,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                ],
            }
        }
    }
}
