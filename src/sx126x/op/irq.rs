
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

    impl Into<u16> for IrqMask {
        fn into(self) -> u16 {
            self.inner
        }
    }

    impl From<u16> for IrqMask {
        fn from(mask: u16) -> Self {
            Self { inner: mask }
        }
    }