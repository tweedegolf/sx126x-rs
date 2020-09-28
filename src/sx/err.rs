use core::fmt::{self, Debug};
use embedded_hal::digital::v2::OutputPin;

// pub type OutputPinError<TPIN> = <TPIN as OutputPin>::Error;

pub enum SpiError<TWERR, TTERR> {
    Write(TWERR),
    Transfer(TTERR),
}

impl<TWERR: Debug, TTERR: Debug> Debug for SpiError<TWERR, TTERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Write(err) => write!(f, "Write({:?})", err),
            Self::Transfer(err) => write!(f, "Transfer({:?})", err),
        }
    }
}

pub enum SxError<TSPIERR, TPINERR> {
    SpiWrite(TSPIERR),
    SpiTransfer(TSPIERR),
    OutputPin(TPINERR),
    InputPin(TPINERR),
}

impl<TSPIERR: Debug, TPINERR: Debug> Debug for SxError<TSPIERR, TPINERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SpiWrite(err) => write!(f, "Write({:?})", err),
            Self::SpiTransfer(err) => write!(f, "Transfer({:?})", err),
            Self::OutputPin(err) => write!(f, "OutputPin({:?})", err),
            Self::InputPin(err) => write!(f, "InputPin({:?})", err),
        }
    }
}
