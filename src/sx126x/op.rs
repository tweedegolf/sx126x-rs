#[allow(dead_code)]

pub trait Operation {
    type Raw: AsRef<[u8]>;
    fn op_code() -> u8;

    fn into_raw(&self) -> Self::Raw;

    fn from_raw(raw: &Self::Raw) -> Self;
}

macro_rules! op {
    ($op:ident, $op_raw_ty:ty, $op_code:literal, $comment:literal, $raw_len:literal, |$raw_ident:ident| $from_raw:tt) => {
        #[doc=$comment]
        pub struct $op {
            raw: $op_raw_ty
        }

        impl Operation for $op {
            type Raw = [u8; $raw_len];

            #[inline(always)]
            fn op_code() -> u8 {
                $op_code
            }

            fn into_raw(&self) -> Self::Raw {                
                // TODO: Use MaybeUninit
                let mut raw = [0; $raw_len];
                raw[0] = $op_code;
                // TODO: verify that this works correctly
                if $raw_len > 0 {
                    unsafe { core::ptr::replace(&mut raw[1] as *mut _ as *mut $op_raw_ty, self.raw); }
                }
                raw
            }

            fn from_raw($raw_ident: &Self::Raw) -> Self
                {$from_raw}
        }

    };
}

macro_rules! zb_op {
    ($op:ident, $op_code:literal, $comment:literal) => {
        op!($op, [u8; 0], $op_code, $comment, 2, |_raw| { Self { raw: [] } });
    };
}

/// Defines a single-byte payload operation
macro_rules! sb_op {
    ($op:ident, $op_code:literal, $comment:literal) => {
        op!($op, [u8; 1], $op_code, $comment, 2, |raw| { Self { raw: [raw[1]] } });
    };
}

/// Defines a dual-byte payload operation
macro_rules! db_op {
    ($op:ident, $op_code:literal, $comment:literal) => {
        op!($op, [u8;2], $op_code, $comment, 3, |raw| { Self { raw: [raw[1], raw[2]] } });
    };
}

/// Defines a triple-byte payload operation
macro_rules! tb_op {
    ($op:ident, $op_code:literal, $comment:literal) => {
        op!($op, [u8;3], $op_code, $comment, 4, |raw| { Self { raw: [raw[1], raw[2], raw[3]] } });
    };
}

/// Defines a quadruple-byte payload operation
macro_rules! qb_op {
    ($op:ident, $op_code:literal, $comment:literal) => {
        op!($op, [u8;4], $op_code, $comment, 5, |raw| { Self { raw: [raw[1], raw[2], raw[3], raw[4]] } });
    };
}

/// Defines a hexuple-byte payload operation
macro_rules! hb_op {
    ($op:ident, $op_code:literal, $comment:literal) => {
        op!($op, [u8;6], $op_code, $comment, 7, |raw| { Self { raw: [raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]] } });
    };
}

pub mod mode {
    use super::Operation;
    impl SetSleep {
        pub fn new(warm_start: bool, rtc_wake: bool) -> Self {
            let mut raw = 0x00;
            if warm_start {
                raw |= 1 << 2;
            }
            if rtc_wake {
                raw |= 1 << 0
            }
            Self { raw: [raw] }
        }
    }
    sb_op!(SetSleep, 0x84, "Set Chip in SLEEP mode");
    sb_op!(SetStandby, 0x80, "Set Chip in STDBY_RC or STDBY_XOSC mode");
    zb_op!(SetFs, 0xC1, "Set Chip in Frequency Synthesis mode");
    tb_op!(SetTx, 0x83, "Set Chip in Tx mode");
    tb_op!(SetRx, 0x82, "Set Chip in Rx mode");
    sb_op!(StopTimerOnPreamble, 0x9F, "Stop Rx timeout on Sync Word/Header or preamble detection");
    sext_op!(SetRxDutyCycle, 0x94, "Store values of RTC setup for listen mode and if period parameter is not 0, set chip into RX mode");
    zb_op!(SetCad, 0xC5, "Set chip into RX mode with passed CAD parameters");
    zb_op!(SetTxContinuousWave, 0xD1, "Set chip into TX mode with infinite carrier wave settings");
    zb_op!(SetTxInfinitePreamble, 0xD2, "Set chip into TX mode with infinite preamble settings");
    sb_op!(SetRegulatorMode, 0x96, "Select LDO or DC_DC for CFG_XOSC, FS, RX, or TX mode");
    sb_op!(Calibrate, 0x89, "Calibrate the RC13, RC64, ADC, PLL, Image according to parameter");
    tb_op!(CalibrateImage, 0x98, "Launches an image calibration at the given frequencies");
    qb_op!(SetPaConfig, 0x95, "Configure the Duty Cycle, Max output power, device for the PA for SX1261 or SX1262");
    sb_op!(SetRxTxFallbackMode, 0x93, "Defines into which mode the chip goes after a TX/RX done");
}

pub mod access {
    #[repr(u8)]
    pub enum OpCode {
        /// Write into one or several registers
        WriteRegister = 0x0D,
        /// Read one or several registers
        ReadRegister = 0x1D,
        /// Write data into the FIFO
        WriteBuffer = 0x0E,
        /// Read data from the FIFO
        ReadBuffer = 0x1E,
    }
}

pub mod irq_dio {
    pub enum OpCode {
        /// Configure the IRQ and the DIOs attached to each IRQ
        SetDioIrqParams = 0x08,
        /// Get the values of the triggered IRQs
        GetIrqStatus = 0x12,
        /// Clear one or several of the IRQs
        ClearIrqStatus = 0x02,
        /// Configure radio to control an RF switch from DIO2
        SetDIO2AsRfSwitchCtrl = 0x9D,
        /// Configure the radio to use a TCXO controlled by DIO3
        SetDIO3AsTcxoCtrl = 0x97,
    }
}

pub mod rf_pack {
    #[repr(u8)]
    pub enum OpCode {
        /// Set the RF frequecy of the radio
        SetRfFrequency = 0x86,
        /// Select the packet type corresponding to the modem
        SetPacketType = 0x8A,
        /// Get the current packet confiuration for the device
        GetPacketType = 0x11,
        /// Set output power and ramp time for the PA
        SetTxParams = 0x8E,
        /// Compute and set values in selected protocol modem
        /// for given modulation parameters
        SetModulationParams = 0x8B,
        /// Set values on selected protocol modem for given packet parameters
        SetPacketParams = 0x8C,
        /// Set the parameters which are used for performing a CAD (LoRa only)
        SetCadParams = 0x88,
        /// Store TX and RX base address in register of selected protocol modem
        SetBufferBaseAddress = 0x8F,
        /// Set the number of symbols the modem has to wait to validate a block
        SetLoRaSymbNumTimeout = 0xA0,
    }
}

pub mod status {
    #[repr(u8)]
    pub enum OpCode {
        /// Returns the current status of the device
        GetStatus = 0xC0,
        /// Returns the instantaneous measured RSSI while in Rx mode
        GetRssiInst = 0x15,
        /// Returns PayloadLengthRx(7:0), RxBufferPointer(7:0)
        GetRxBufferStatus = 0x13,
        /// Returns RssiAvg, RssiSync, PStatus2, PStatus3, PStatus4 in FSK protocol,
        /// returns RssiPkt, SnrPkt in LoRa protocol
        GetPacketStatus = 0x14,
        /// Returns the error which has occured in the device
        GetDeviceErrors = 0x17,
        /// Clear all the errors. The error(s) canot be cleared independently
        ClearDeviceErrors = 0x08,
        /// Returns statistics on the last few received packets
        GetStats = 0x10,
        /// Resets the value read by the command GetStats
        ResetStats = 0x00,
    }
}
