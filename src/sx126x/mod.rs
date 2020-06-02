pub mod op;
pub mod reg;

use core::marker::PhantomData;
use embedded_hal::blocking::spi::{Transfer, Write};

pub struct SX126x<TSPI, TBUSY> {
    spi: PhantomData<TSPI>,
    busy: TBUSY,
}

impl<TSPI, TBUSY> SX126x<TSPI, TBUSY>
where
    TSPI: Transfer<u8> + Write<u8>,
{
    pub fn init(spi: &mut TSPI, busy: TBUSY) -> Self {
        // TODO initialize SX126x device by sending
        //
        Self { spi: PhantomData, busy }
    }
}
