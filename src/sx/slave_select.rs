use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

use crate::err::{PinError, SpiError};

pub struct SlaveSelect<TNSS: OutputPin> {
    nss: TNSS,
}

impl<TNSS: OutputPin> SlaveSelect<TNSS> {
    pub fn new(nss: TNSS) -> Self {
        Self { nss }
    }
}

pub struct SlaveSelectGuard<'nss, 'spi, TNSS: OutputPin, TSPI: Write<u8> + Transfer<u8>> {
    nss: &'nss mut TNSS,
    spi: &'spi mut TSPI,
}

impl<TNSS: OutputPin> SlaveSelect<TNSS> {
    pub fn select<'spi, TSPI: Write<u8> + Transfer<u8>>(
        &'spi mut self,
        spi: &'spi mut TSPI,
    ) -> Result<SlaveSelectGuard<TNSS, TSPI>, PinError<<TNSS as OutputPin>::Error>> {
        self.nss.set_low().map_err(PinError::Output)?;
        Ok(SlaveSelectGuard {
            nss: &mut self.nss,
            spi,
        })
    }
}

impl<'nss, 'spi, TNSS: OutputPin, TSPI: Write<u8> + Transfer<u8>> Drop
    for SlaveSelectGuard<'nss, 'spi, TNSS, TSPI>
{
    fn drop(&mut self) {
        let _ = self.nss.set_high();
    }
}

impl<'nss, 'spi, TNSS, TSPI, TSPIERR> Write<u8> for SlaveSelectGuard<'nss, 'spi, TNSS, TSPI>
where
    TNSS: OutputPin,
    TSPI: Write<u8, Error = TSPIERR> + Transfer<u8, Error = TSPIERR>,
{
    type Error = SpiError<TSPIERR>;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(words).map_err(SpiError::Write)
    }
}

impl<'nss, 'spi, TNSS, TSPI, TSPIERR> Transfer<u8> for SlaveSelectGuard<'nss, 'spi, TNSS, TSPI>
where
    TNSS: OutputPin,
    TSPI: Write<u8, Error = TSPIERR> + Transfer<u8, Error = TSPIERR>,
{
    type Error = SpiError<TSPIERR>;
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer(words).map_err(SpiError::Transfer)
    }
}
