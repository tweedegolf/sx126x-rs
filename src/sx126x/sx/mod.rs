mod slave_select;

use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use super::conf::Config;
use super::op::*;
use super::reg::*;
use slave_select::*;

type InputPinError<TPIN> = <TPIN as InputPin>::Error;
type OutputPinError<TPIN> = <TPIN as OutputPin>::Error;

type SpiWriteError<TSPI> = <TSPI as Write<u8>>::Error;
type SpiTransferError<TSPI> = <TSPI as Transfer<u8>>::Error;

const NOP: u8 = 0x00;

pub struct SX126x<TSPI, TNSS: OutputPin, TNRST, TBUSY, TANT, TDIO1, TDIO2> {
    spi: PhantomData<TSPI>,
    slave_select: SlaveSelect<TNSS>,
    nrst_pin: TNRST,
    busy_pin: TBUSY,
    ant_pin: TANT,
    dio1_pin: TDIO1,
    dio2_pin: TDIO2,
}

impl<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1, TDIO2>
    SX126x<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1, TDIO2>
where
    TSPI: Write<u8> + Transfer<u8>,
    TNSS: OutputPin<Error = Infallible>,
    TNRST: OutputPin<Error = Infallible>,
    TBUSY: InputPin<Error = Infallible>,
    TANT: OutputPin<Error = Infallible>,
    TDIO1: InputPin<Error = Infallible>,
    TDIO2: InputPin<Error = Infallible>,
{
    pub fn init(
        spi: &mut TSPI,
        delay: &mut (impl DelayUs<u32> + DelayMs<u32>),
        pins: (TNSS, TNRST, TBUSY, TANT, TDIO1, TDIO2),
        conf: Config,
    ) -> Result<Self, SpiWriteError<TSPI>> {
        let (mut nss_pin, mut nrst_pin, busy_pin, ant_pin, dio1_pin, dio2_pin) = pins;
        nrst_pin.set_high().unwrap();
        nss_pin.set_high().unwrap();

        let mut sx = Self {
            spi: PhantomData,
            slave_select: SlaveSelect::new(nss_pin),
            nrst_pin,
            busy_pin,
            ant_pin,
            dio1_pin,
            dio2_pin,
        };

        // Reset the sx
        sx.reset(delay).unwrap();

        //Wakeup
        // TODO: return on error
        let _ = sx.wakeup(spi, delay);

        // Go to standby mode
        sx.set_standby(spi, delay, conf.standby_config)?;

        // Configure pins
        #[cfg(feature = "tcxo")]
        {
            sx.set_dio3_as_tcxo_ctrl(spi, delay, conf.tcxo_voltage, conf.tcxo_delay)?;
            sx.calibrate(spi, delay, conf.calib_param)?;
        }
        sx.set_dio2_as_rf_switch_ctrl(spi, delay, true)?;
        let _ = sx.set_ant_enabled(true);

        // Configure communication parameters
        sx.set_packet_type(spi, delay, conf.packet_type)?;
        sx.set_sync_word(spi, delay, conf.sync_word)?;
        sx.set_packet_params(spi, delay, conf.packet_params)?;
        sx.set_mod_params(spi, delay, conf.mod_params)?;
        sx.set_tx_params(spi, delay, conf.tx_params)?;
        sx.calibrate_image(spi, delay, CalibImageFreq::from_rf_freq(conf.rf_freq))?;
        sx.set_rf_frequency(spi, delay, conf.rf_freq)?;

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
            &sync_word.to_be_bytes(),
        )
    }
    pub fn set_packet_type<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        packet_type: PacketType,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x8A, packet_type as u8])
    }

    pub fn set_standby<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        standby_config: StandbyConfig,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x80, standby_config as u8])
    }

    pub fn get_status<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<Status, SpiTransferError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let mut result = [NOP];
        spi.transfer(&mut [0xC0])
            .and_then(|_| spi.transfer(&mut result))?;

        Ok(result[0].into())
    }

    pub fn calibrate_image<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        freq: CalibImageFreq,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let freq: [u8; 2] = freq.into();
        spi.write(&[0x98]).and_then(|_| spi.write(&freq))
    }

    pub fn calibrate<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        calib_param: CalibParam,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x89, calib_param.into()])
    }

    pub fn write_register<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        register: Register,
        data: &[u8],
    ) -> Result<(), SpiWriteError<TSPI>> {
        let start_addr = (register as u16).to_be_bytes();

        let mut spi = self.slave_select(spi, delay).unwrap();
        let r = spi
            .write(&[0x0D])
            .and_then(|_| spi.write(&start_addr))
            .and_then(|_| spi.write(data));

        r
    }

    pub fn read_register<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        start_addr: u16,
        result: &mut [u8],
    ) -> Result<(), SpiTransferError<TSPI>> {
        debug_assert!(result.len() >= 1);
        let mut start_addr = start_addr.to_be_bytes();
        let mut spi = self.slave_select(spi, delay).unwrap();

        spi.transfer(&mut [0x1D])
            .and_then(|_| spi.transfer(&mut start_addr))
            .and_then(|_| spi.transfer(result))?;
        Ok(())
    }

    pub fn write_buffer<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        offset: u8,
        data: &[u8],
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x0E, offset]).and_then(|_| spi.write(data))
    }

    pub fn read_buffer<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        offset: u8,
        result: &mut [u8],
    ) -> Result<(), SpiTransferError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let mut header = [0x1E, offset, NOP];
        spi.transfer(&mut header)
            .and_then(|_| spi.transfer(result))
            .map(|_| {})
    }

    pub fn set_dio2_as_rf_switch_ctrl<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        enable: bool,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x9D, enable as u8])
    }

    pub fn set_dio3_as_tcxo_ctrl<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        tcxo_voltage: TcxoVoltage,
        tcxo_delay: TcxoDelay,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let tcxo_delay: [u8; 3] = tcxo_delay.into();
        spi.write(&[0x97, tcxo_voltage as u8])
            .and_then(|_| spi.write(&tcxo_delay))
    }

    pub fn clear_device_errors<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x07, 0x00, 0x00])
    }

    pub fn get_device_errors<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<DeviceErrors, SpiTransferError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
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

    pub fn set_dio_irq_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        irq_mask: IrqMask,
        dio1_mask: IrqMask,
        dio2_mask: IrqMask,
        dio3_mask: IrqMask,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.write(&[0x08])
            .and_then(|_| spi.write(&(Into::<u16>::into(irq_mask)).to_be_bytes()))
            .and_then(|_| spi.write(&(Into::<u16>::into(dio1_mask)).to_be_bytes()))
            .and_then(|_| spi.write(&(Into::<u16>::into(dio2_mask)).to_be_bytes()))
            .and_then(|_| spi.write(&(Into::<u16>::into(dio3_mask)).to_be_bytes()))
    }

    pub fn set_tx<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        timeout: TxTimeout,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let timeout: [u8; 3] = timeout.into();
        spi.write(&[0x83]).and_then(|_| spi.write(&timeout))
    }

    pub fn set_packet_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        params: PacketParams,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let params: [u8; 9] = params.into();
        spi.write(&[0x8C]).and_then(|_| spi.write(&params))
    }

    pub fn set_mod_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        params: ModParams,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let params: [u8; 8] = params.into();
        spi.write(&[0x8B]).and_then(|_| spi.write(&params))
    }

    pub fn set_tx_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        params: TxParams,
    ) -> Result<(), SpiWriteError<TSPI>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        let params: [u8; 2] = params.into();
        spi.write(&[0x8E]).and_then(|_| spi.write(&params))
    }

    /// 13.4.1.: RFfrequecy = (RFfreq * Fxtal) / 2^25
    /// -> RFfrequecy ~ ((RFfreq >> 12) * (Fxtal >> 12)) >> 1
    pub fn set_rf_frequency<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        rf_freq: u32,
    ) -> Result<(), SpiWriteError<TSPI>> {
        // TODO: support other XTAL frequencies
        const XTAL_FREQ_HZ: u32 = 32_000_000; // 32 MHz
                                              // Shifting right by 12 avoids an overflow for all supported frequencies
        const XTAL_FREQ_SHIFT: u8 = 12;
        const XTAL_FREQ_SHIFTED: u32 = XTAL_FREQ_HZ >> XTAL_FREQ_SHIFT;
        const DIVISOR_FREQ_SHIFT: u8 = 25 - (2 * XTAL_FREQ_SHIFT);

        let mut spi = self.slave_select(spi, delay).unwrap();

        let x = rf_freq >> XTAL_FREQ_SHIFT;
        let y = x * XTAL_FREQ_SHIFTED;
        let z = y >> DIVISOR_FREQ_SHIFT;

        let freq = z.to_be_bytes();
        spi.write(&[0x86]).and_then(|_| spi.write(&freq))
    }

    pub fn write_bytes<'spi, 'data>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        data: &'data [u8],
        timeout: TxTimeout,
    ) -> Result<(), SpiWriteError<TSPI>> {
        // Write data to buffer
        self.write_buffer(spi, delay, 0x00, data)?;
        // Set tx mode
        self.set_tx(spi, delay, timeout)?;
        // Wait for busy line to go low
        self.wait_on_busy(delay);
        // Write completed!
        Ok(())
    }

    fn wait_on_busy(&mut self, delay: &mut impl DelayUs<u32>) {
        // 8.3.1: The max value for T SW from NSS rising edge to the BUSY rising edge is, in all cases, 600 ns
        delay.delay_us(1);
        while self.busy_pin.is_high().unwrap() {
            cortex_m::asm::nop();
        }
    }

    fn wakeup<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
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

    fn slave_select<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<SlaveSelectGuard<TNSS, TSPI>, OutputPinError<TNSS>> {
        self.wait_on_busy(delay);
        let s = self.slave_select.select(spi)?;
        // Table 8-1: Data sheet specifies a minumum delay of 32ns between falling edge of nss and sck setup,
        // though embedded_hal provides no trait for delaying in nanosecond resolution.
        // delay.delay_us(1);
        Ok(s)
    }
}
