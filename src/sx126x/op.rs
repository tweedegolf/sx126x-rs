#[allow(dead_code)]

pub mod mode {
    #[repr(u8)]
    pub enum OpCode {
        /// Set Chip in SLEEP mode
        SetSleep = 0x84,
        /// Set Chip in STDBY_RC or STDBY_XOSC mode
        SetStandby = 0x80,
        /// Set Chip in Frequency Synthesis mode
        SetFs = 0xC1,
        /// Set Chip in Tx mode
        SetTx = 0x83,
        /// Set Chip in Rx mode
        SetRx = 0x82,
        /// Stop Rx timeout on Sync Word/Header or preamble detection
        StopTimerOnPreamble = 0x9F,
        /// Store values of RTC setup for listen mode and if period parameter
        /// is not 0, set chip into RX mode
        SetRxDutyCycle = 0x94,
        /// Set chip into RX mode with passed CAD parameters
        SetCad = 0xC5,
        /// Set chip into TX mode with infinite carrier wave settings
        SetTxContinuousWave = 0xD1,
        /// Set chip into TX mode with infinite preamble settings
        SetTxInfinitePreamble = 0xD2,
        /// Select LDO or DC_DC for CFG_XOSC, FS, RX, or TX mode
        SetRegulatorMode = 0x96,
        /// Calibrate the RC13, RC64, ADC, PLL, Image according to parameter
        Calibrate = 0x89,
        /// Launches an image calibration at the given frequencies
        CalibrateImage = 0x98,
        /// Configure the Duty Cycle, Max output power, device for the PA
        /// for SX1261 or SX1262
        SetPaConfig = 0x95,
        /// Defines into which mode the chip goes after a TX/RX done
        SetRxTxFallbackMode = 0x93,
    }
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
