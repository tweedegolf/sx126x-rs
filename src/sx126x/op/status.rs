#[derive(Copy, Clone)]
pub struct Status {
    inner: u8,
}

impl core::fmt::Debug for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let chip_mode = self.chip_mode();
        let command_status = self.command_status();
        write!(f, "{{inner: {:#08b}, chip_mode: {:?}, command_status: {:?}}}", self.inner, chip_mode, command_status)
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum ChipMode {
    StbyRC = 0x02,
    StbyXOSC = 0x03,
    FS = 0x04,
    RX = 0x05,
    TX = 0x06,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
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
