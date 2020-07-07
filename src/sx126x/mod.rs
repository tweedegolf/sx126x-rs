pub mod op;
pub mod reg;

use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use op::*;
use reg::*;

type InputPinError<TPIN> = <TPIN as InputPin>::Error;
type OutputPinError<TPIN> = <TPIN as OutputPin>::Error;

type SpiWriteError<TSPI> = <TSPI as Write<u8>>::Error;
type SpiTransferError<TSPI> = <TSPI as Transfer<u8>>::Error;

const NOP: u8 = 0x00;

pub struct Config<TF1, TF2> {
    pub freq_1: TF1,
    pub freq_2: TF2,
    pub packet_type: PacketType,
    pub standby_config: StandbyConfig,
    pub sync_word: u16,
    #[cfg(feature = "tcxo")]
    pub tcxo_delay: TcxoDelay,
    #[cfg(feature = "tcxo")]
    pub tcxo_voltage: TcxoVoltage,
}

pub struct SX126x<TSPI, TNSS: OutputPin, TNRST, TBUSY, TANT> {
    spi: PhantomData<TSPI>,
    slave_select: SlaveSelect<TNSS>,
    nrst_pin: TNRST,
    busy_pin: TBUSY,
    ant_pin: TANT,
}

impl<TSPI, TNSS, TNRST, TBUSY, TANT> SX126x<TSPI, TNSS, TNRST, TBUSY, TANT>
where
    TSPI: Write<u8> + Transfer<u8>,
    TNSS: OutputPin<Error = Infallible>,
    TNRST: OutputPin<Error = Infallible>,
    TBUSY: InputPin<Error = Infallible>,
    TANT: OutputPin<Error = Infallible>,
{
    pub fn init(
        spi: &mut TSPI,
        delay: &mut (impl DelayUs<u32> + DelayMs<u32>),
        pins: (TNSS, TNRST, TBUSY, TANT),
        conf: Config<impl Into<u8>, impl Into<u8>>,
    ) -> Result<Self, SpiWriteError<TSPI>> {
        let (mut nss, mut nrst_pin, busy_pin, ant_pin) = pins;
        nrst_pin.set_high().unwrap();
        nss.set_high().unwrap();

        let mut sx = Self {
            spi: PhantomData,
            slave_select: SlaveSelect::new(nss),
            nrst_pin,
            busy_pin,
            ant_pin,
        };

        delay.delay_ms(1000);
        // Wait for modem to become available
        sx.wait_on_busy(delay);

        sx.reset(delay).unwrap();

        //Wakeup
        sx.wakeup(spi, delay);

        sx.set_standby(spi, delay, conf.standby_config)?;

        #[cfg(feature = "tcxo")]
        {
            sx.set_dio3_as_tcxo_ctrl(spi, delay, conf.tcxo_voltage, conf.tcxo_delay)?;
            sx.calibrate(spi, delay, 0x7F.into())?;
        }
        sx.set_dio2_as_rf_switch_ctrl(spi, delay, true)?;

        sx.set_packet_type(spi, delay, conf.packet_type)?;
        sx.set_sync_word(spi, delay, conf.sync_word)?;

        Ok(sx)
    }

    pub fn set_sync_word(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        sync_word: u16,
    ) -> Result<(), SpiWriteError<TSPI>> {
        self.write_register(
            spi,
            delay,
            Register::LoRaSyncWordMsb,
            &sync_word.to_le_bytes(),
        )
    }
    pub fn set_packet_type(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        packet_type: PacketType,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay);
        spi.write(&[0x8A, packet_type as u8])
    }

    pub fn set_standby(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        standby_config: StandbyConfig,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay);
        spi.write(&[0x80, standby_config as u8])
    }

    pub fn get_status(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<Status, SpiTransferError<TSPI>> {
        let s = self.slave_select(delay);
        let mut result = [NOP];
        spi.transfer(&mut [0xC0])
            .and_then(|_| spi.transfer(&mut result))?;

        Ok(result[0].into())
    }

    pub fn calibrate_image(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        freq_1: impl Into<u8>,
        freq_2: impl Into<u8>,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay);
        spi.write(&[0x98, freq_1.into(), freq_2.into()])
    }

    pub fn calibrate(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        calib_param: CalibParam,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay);
        spi.write(&[0x89, calib_param.into()])
    }

    pub fn write_register(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        register: Register,
        data: &[u8],
    ) -> Result<(), SpiWriteError<TSPI>> {
        let start_addr = (register as u16).to_le_bytes();

        let s = self.slave_select(delay);
        let r = spi
            .write(&[0x0D])
            .and_then(|_| spi.write(&start_addr))
            .and_then(|_| spi.write(data));

        r
    }

    pub fn read_register(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        start_addr: u16,
        result: &mut [u8],
    ) -> Result<(), SpiTransferError<TSPI>> {
        debug_assert!(result.len() >= 1);
        let mut start_addr = start_addr.to_le_bytes();
        let s = self.slave_select(delay).unwrap();

        spi.transfer(&mut [0x1D])
            .and_then(|_| spi.transfer(&mut start_addr))
            .and_then(|_| spi.transfer(result))?;
        Ok(())
    }

    pub fn write_buffer(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        offset: u8,
        data: &[u8],
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay).unwrap();
        spi.write(&[0x0E, offset]).and_then(|_| spi.write(data))
    }

    pub fn read_buffer(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        offset: u8,
        result: &mut [u8],
    ) -> Result<(), SpiTransferError<TSPI>> {
        let s = self.slave_select(delay).unwrap();
        let mut header = [0x1E, offset, NOP];
        spi.transfer(&mut header)
            .and_then(|_| spi.transfer(result))
            .map(|_| {})
    }

    pub fn set_dio2_as_rf_switch_ctrl(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        enable: bool,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay).unwrap();
        spi.write(&[0x9D, enable as u8])
    }

    pub fn set_dio3_as_tcxo_ctrl(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        tcxo_voltage: TcxoVoltage,
        tcxo_delay: TcxoDelay,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay).unwrap();
        let tcxo_delay: [u8; 3] = tcxo_delay.into();
        let r = spi
            .write(&[0x97, tcxo_voltage as u8])
            .and_then(|_| spi.write(&tcxo_delay));
        let s = s;
        r
    }

    pub fn clear_device_errors(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let s = self.slave_select(delay);
        spi.write(&[0x07, 0x00, 0x00])
    }

    pub fn get_device_errors(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<DeviceErrors, SpiTransferError<TSPI>> {
        let s = self.slave_select(delay).unwrap();
        let mut result = [NOP; 2];
        spi.transfer(&mut [0x17, NOP])
            .and_then(|_| spi.transfer(&mut result))?;
        Ok(DeviceErrors::from(u16::from_le_bytes(result)))
    }

    pub fn reset(&mut self, delay: &mut impl DelayUs<u32>) -> Result<(), OutputPinError<TNRST>> {
        cortex_m::interrupt::free(|_| {
            self.nrst_pin.set_low()?;
            // 8.1: The pin should be held low for typically 100 μs for the Reset to happen
            delay.delay_us(200);
            self.nrst_pin.set_high()
        })
    }

    pub fn set_ant_enabled(&mut self, enabled: bool) -> Result<(), OutputPinError<TANT>> {
        if enabled {
            self.ant_pin.set_high()
        } else {
            self.ant_pin.set_low()
        }
    }

    fn wait_on_busy(&mut self, delay: &mut impl DelayUs<u32>) {
        // 8.3.1: The max value for T SW from NSS rising edge to the BUSY rising edge is, in all cases, 600 ns
        delay.delay_us(1);
        while self.busy_pin.is_high().unwrap() {
            cortex_m::asm::nop();
        }
    }

    fn wakeup(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<(), SpiTransferError<TSPI>> {
        cortex_m::interrupt::free(|_| {
            self.get_status(spi, delay)?;
            self.wait_on_busy(delay);
            // TODO: switch on antenna
            Ok(())
        })?;
        self.set_ant_enabled(true).unwrap();
        Ok(())
    }

    fn slave_select(
        &mut self,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<SlaveSelectGuard<TNSS>, OutputPinError<TNSS>> {
        self.wait_on_busy(delay);
        let s = self.slave_select.select()?;
        // Table 8-1: Data sheet specifies a minumum delay of 32ns between falling edge of nss and sck setup,
        // though embedded_hal provides no trait for delaying in nanosecond resolution.
        delay.delay_us(1);
        Ok(s)
    }
}

struct SlaveSelect<TNSS: OutputPin> {
    nss: TNSS,
}

impl<TNSS: OutputPin> SlaveSelect<TNSS> {
    pub fn new(nss: TNSS) -> Self {
        Self { nss }
    }
}

struct SlaveSelectGuard<'nss, TNSS: OutputPin> {
    nss: &'nss mut TNSS,
}

impl<TNSS: OutputPin> SlaveSelect<TNSS> {
    fn select(&mut self) -> Result<SlaveSelectGuard<TNSS>, OutputPinError<TNSS>> {
        self.nss.set_low()?;
        Ok(SlaveSelectGuard { nss: &mut self.nss })
    }
}

impl<'nss, TNSS: OutputPin> Drop for SlaveSelectGuard<'nss, TNSS> {
    fn drop(&mut self) {
        let _ = self.nss.set_high();
    }
}
