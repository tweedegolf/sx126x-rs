#[derive(Copy, Clone)]
pub struct Status {
    inner: u8,
}

impl core::fmt::Debug for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let chip_mode = self.chip_mode();
        let command_status = self.command_status();
        write!(
            f,
            "Status {{inner: {:#08b}, chip_mode: {:?}, command_status: {:?}}}",
            self.inner, chip_mode, command_status
        )
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
    DataAvailable = 0x02,
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
    pub fn chip_mode(&self) -> Option<ChipMode> {
        use ChipMode::*;
        match (self.inner & 0x70) >> 4 {
            0x02 => Some(StbyRC),
            0x03 => Some(StbyXOSC),
            0x04 => Some(FS),
            0x05 => Some(RX),
            0x06 => Some(TX),
            _ => None,
        }
    }

    pub fn command_status(self) -> Option<CommandStatus> {
        use CommandStatus::*;
        match (self.inner & 0x0E) >> 1 {
            0x02 => Some(DataAvailable),
            0x03 => Some(CommandTimeout),
            0x04 => Some(CommandProcessingError),
            0x05 => Some(FailureToExecute),
            0x06 => Some(CommandTxDone),
            _ => None,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Stats {
    pub status: Status,
    pub rx_pkt: u16,
    pub crc_error: u16,
    pub header_error: u16,
}

impl From<[u8; 7]> for Stats {
    fn from(b: [u8; 7]) -> Self {
        Self {
            status: b[0].into(),
            rx_pkt: u16::from_be_bytes([b[1], b[2]]),
            crc_error: u16::from_be_bytes([b[3], b[4]]),
            header_error: u16::from_be_bytes([b[5], b[6]]),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PacketStatus {
    rssi_pkt: u8,
    snr_pkt: i8,
    signal_rssi_pkt: u8,
}

impl From<[u8; 3]> for PacketStatus {
    fn from(b: [u8; 3]) -> Self {
        Self {
            rssi_pkt: b[0],
            snr_pkt: i8::from_be_bytes([b[1]]),
            signal_rssi_pkt: b[2],
        }
    }
}

impl PacketStatus {
    pub fn rssi_pkt(&self) -> f32 {
        self.rssi_pkt as f32 / -2.0
    }

    pub fn snr_pkt(&self) -> f32 {
        self.snr_pkt as f32 / 4.0
    }

    pub fn signal_rssi_pkt(&self) -> f32 {
        self.signal_rssi_pkt as f32 / -2.0
    }
}