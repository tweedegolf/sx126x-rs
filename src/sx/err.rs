use core::fmt::{self, Debug};

pub enum SpiError<TSPIERR> {
    Write(TSPIERR),
    Transfer(TSPIERR),
}

impl<TSPIERR: Debug> Debug for SpiError<TSPIERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Write(err) => write!(f, "Write({:?})", err),
            Self::Transfer(err) => write!(f, "Transfer({:?})", err),
        }
    }
}

pub enum PinError<TPINERR> {
    Output(TPINERR),
    Input(TPINERR),
}

impl<TPINERR: Debug> Debug for PinError<TPINERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Output(err) => write!(f, "Output({:?})", err),
            Self::Input(err) => write!(f, "Input({:?})", err),
        }
    }
}

pub enum SxError<TSPIERR, TPINERR> {
    Spi(SpiError<TSPIERR>),
    Pin(PinError<TPINERR>),
}

impl<TSPIERR: Debug, TPINERR: Debug> Debug for SxError<TSPIERR, TPINERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spi(err) => write!(f, "Spi({:?})", err),
            Self::Pin(err) => write!(f, "Pin({:?})", err),
        }
    }
}

impl<TSPIERR, TPINERR> From<SpiError<TSPIERR>> for SxError<TSPIERR, TPINERR> {
    fn from(spi_err: SpiError<TSPIERR>) -> Self {
        SxError::Spi(spi_err)
    }
}

impl<TSPIERR, TPINERR> From<PinError<TPINERR>> for SxError<TSPIERR, TPINERR> {
    fn from(spi_err: PinError<TPINERR>) -> Self {
        SxError::Pin(spi_err)
    }
}
