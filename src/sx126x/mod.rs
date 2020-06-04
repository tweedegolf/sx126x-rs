pub mod op;
pub mod reg;

use core::marker::PhantomData;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use op::Operation;

type InputPinError<TPIN: InputPin> = <TPIN as InputPin>::Error;
type OutputPinError<TPIN: OutputPin> = <TPIN as OutputPin>::Error;

pub struct SX126x<TSPI, TNSS, TNRST, TBUSY> {
    spi: PhantomData<TSPI>,
    nss: TNSS,
    nrst: TNRST,
    busy: TBUSY,
}

impl<TSPI, TNSS, TNRST, TBUSY> SX126x<TSPI, TNSS, TNRST, TBUSY>
where
    TSPI: Transfer<u8> + Write<u8>,
    TNSS: OutputPin,
    TNRST: OutputPin,
    TBUSY: InputPin,
{
    pub fn init(
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        pins: (TNSS, TNRST, TBUSY),
    ) -> Result<Self, <TSPI as Write<u8>>::Error> {
        let (mut nss, mut nrst, busy) = pins;
        nrst.set_high();
        nss.set_high();

        let mut sx = Self {
            spi: PhantomData,
            nss,
            nrst,
            busy,
        };

        sx.send_command(spi, delay, op::mode::SetStandby::with_payload([0]))?;

        // TODO wait for the device to boot

        // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby
        // 2. Define the protocol with the command SetPacketType
        // 3. Define the RF frequency with the command SetRfFrequency
        // 4. Define the Power Amplifier configuration with the command SetPaConfig
        // 5. Define output puwer and ramping time with the command SetTxParams
        // 6.

        Ok(sx)
    }

    pub fn send_command<TOP: Operation>(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        op: TOP,
    ) -> Result<(), <TSPI as Write<u8>>::Error> {
        while self.busy().unwrap_or_default() {}

        let raw = op.into_raw();
        let r = self.spi_write(spi, delay, raw.as_ref());
        delay.delay_us(TOP::wait_us());
        r
    }

    pub fn busy(&mut self) -> Result<bool, InputPinError<TBUSY>> {
        self.busy.is_high()
    }

    pub fn reset(&mut self, delay: &mut impl DelayUs<u32>,) -> Result<(), OutputPinError<TNRST>> {
        self.nrst.set_low()?;
        delay.delay_us(100);
        self.nrst.set_high()
    }

    fn spi_write(&mut self, spi: &mut TSPI, delay: &mut impl DelayUs<u32>, data: &[u8]) -> Result<(), <TSPI as Write<u8>>::Error> {
        self.nss.set_low();
        // Data sheet specifies a minumum delay of 32ns between falling edge of nss and sck setup,
        // though embedded_hal provides no trait for delaying in nanosecond resolution.
        delay.delay_us(1);
        let r = spi.write(data);
        self.nss.set_high();
        r
    }

    fn spi_transfer<'d>(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        data: &'d mut [u8],
    ) -> Result<&'d [u8], <TSPI as Transfer<u8>>::Error> {
        self.nss.set_low();
        // Data sheet specifies a minumum delay of 32ns between falling edge of nss and sck setup,
        // though embedded_hal provides no trait for delaying in nanosecond resolution.
        delay.delay_us(1);
        let r = spi.transfer(data);
        self.nss.set_high();
        r
    }
}
