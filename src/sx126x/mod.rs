pub mod op;
pub mod reg;

use core::marker::PhantomData;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::InputPin;

use op::Operation;

pub struct SX126x<TSPI, TBUSY> {
    spi: PhantomData<TSPI>,
    busy: TBUSY,
}

impl<TSPI, TBUSY> SX126x<TSPI, TBUSY>
where
    TSPI: Transfer<u8> + Write<u8>,
    TBUSY: InputPin,
{

    pub fn init(spi: &mut TSPI, busy: TBUSY) -> Self {
        // TODO initialize SX126x device by sending
        // the correct commands
        Self {
            spi: PhantomData,
            busy,
        }
    }

    pub fn send_command(&mut self, spi: &mut TSPI, op: impl Operation) -> Result<(), <TSPI as Write<u8>>::Error> {
        let raw = op.into_raw();
        spi.write(raw.as_ref())
    }
}
