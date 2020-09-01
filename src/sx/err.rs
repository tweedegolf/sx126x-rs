use embedded_hal::digital::v2::OutputPin;
use core::fmt::{self, Debug};

pub type OutputPinError<TPIN> = <TPIN as OutputPin>::Error;

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