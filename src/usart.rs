use core::fmt;
use embedded_hal::serial;

pub struct UsartWrite<TX> {
    tx: TX,
}

impl<TX: serial::Write<u8>> UsartWrite<TX> {
    pub fn init(tx: TX) -> Self {
        Self { tx }
    }
}

impl<TX: serial::Write<u8>> fmt::Write for UsartWrite<TX> {
    fn write_str(&mut self, s: &str) -> core::result::Result<(), core::fmt::Error> {
        use nb::block;
        s.as_bytes()
            .iter()
            .try_for_each(|b| block!(self.tx.write(*b)).and_then(|_| block!(self.tx.flush())))
            .map_err(|_| core::fmt::Error)?;

        Ok(())
    }
}

#[macro_export]
macro_rules! uprint {
    ($serial:expr, $($arg:tt)*) => {
        core::fmt::Write::write_fmt($serial,format_args!($($arg)*)).ok()
    };
}

#[macro_export]
macro_rules! uprintln {
    ($serial:expr, $fmt:expr) => {
        uprint!($serial, concat!($fmt, "\n"))
    };
    ($serial:expr, $fmt:expr, $($arg:tt)*) => {
        uprint!($serial, concat!($fmt, "\n"), $($arg)*)
    };
}
