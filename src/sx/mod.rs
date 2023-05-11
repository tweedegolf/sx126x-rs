pub(crate) mod err;
mod slave_select;

use core::marker::PhantomData;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use crate::conf::Config;
use crate::op::*;
use crate::reg::*;
// use err::OutputPinError;
use slave_select::*;

use self::err::{PinError, SxError};

type Pins<TNSS, TNRST, TBUSY, TANT, TDIO1> = (TNSS, TNRST, TBUSY, TANT, TDIO1);

const NOP: u8 = 0x00;

/// Calculates the rf_freq value that should be passed to SX126x::set_rf_frequency
/// based on the desired RF frequency and the XTAL frequency.
///
/// Example calculation for 868MHz:
/// 13.4.1.: RFfrequecy = (RFfreq * Fxtal) / 2^25 = 868M
/// -> RFfreq =
/// -> RFfrequecy ~ ((RFfreq >> 12) * (Fxtal >> 12)) >> 1
pub fn calc_rf_freq(rf_frequency: f32, f_xtal: f32) -> u32 {
    (rf_frequency * (33554432. / f_xtal)) as u32
}

/// Wrapper around a Semtech SX1261/62 LoRa modem
pub struct SX126x<TSPI, TNSS: OutputPin, TNRST, TBUSY, TANT, TDIO1> {
    spi: PhantomData<TSPI>,
    slave_select: SlaveSelect<TNSS>,
    nrst_pin: TNRST,
    busy_pin: TBUSY,
    ant_pin: TANT,
    dio1_pin: TDIO1,
}

impl<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1, TSPIERR, TPINERR>
    SX126x<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1>
where
    TPINERR: core::fmt::Debug,
    TSPI: Write<u8, Error = TSPIERR> + Transfer<u8, Error = TSPIERR>,
    TNSS: OutputPin<Error = TPINERR>,
    TNRST: OutputPin<Error = TPINERR>,
    TBUSY: InputPin<Error = TPINERR>,
    TANT: OutputPin<Error = TPINERR>,
    TDIO1: InputPin<Error = TPINERR>,
{
    // Create a new SX126x
    pub fn new(pins: Pins<TNSS, TNRST, TBUSY, TANT, TDIO1>) -> Self {
        let (nss_pin, nrst_pin, busy_pin, ant_pin, dio1_pin) = pins;
        Self {
            spi: PhantomData,
            slave_select: SlaveSelect::new(nss_pin),
            nrst_pin,
            busy_pin,
            ant_pin,
            dio1_pin,
        }
    }

    // Initialize and configure the SX126x using the provided Config
    pub fn init(
        &mut self,
        spi: &mut TSPI,
        delay: &mut (impl DelayUs<u32> + DelayMs<u32>),
        conf: Config,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        // Reset the sx
        self.reset(delay)?;

        // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
        self.set_standby(spi, delay, crate::op::StandbyConfig::StbyRc)?;

        // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
        self.set_packet_type(spi, delay, conf.packet_type)?;

        // 3. Define the RF frequency with the command SetRfFrequency(...)
        self.set_rf_frequency(spi, delay, conf.rf_freq)?;

        if let Some((tcxo_voltage, tcxo_delay)) = conf.tcxo_opts {
            self.set_dio3_as_tcxo_ctrl(spi, delay, tcxo_voltage, tcxo_delay)?;
        }

        // Calibrate
        self.calibrate(spi, delay, conf.calib_param)?;
        self.calibrate_image(
            spi,
            delay,
            CalibImageFreq::from_rf_frequency(conf.rf_frequency),
        )?;

        // 4. Define the Power Amplifier configuration with the command SetPaConfig(...)
        self.set_pa_config(spi, delay, conf.pa_config)?;

        // 5. Define output power and ramping time with the command SetTxParams(...)
        self.set_tx_params(spi, delay, conf.tx_params)?;

        // 6. Define where the data payload will be stored with the command SetBufferBaseAddress(...)
        self.set_buffer_base_address(spi, delay, 0x00, 0x00)?;

        // 7. Send the payload to the data buffer with the command WriteBuffer(...)
        // This is done later in SX126x::write_bytes

        // 8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...) 1
        self.set_mod_params(spi, delay, conf.mod_params)?;

        // 9. Define the frame format to be used with the command SetPacketParams(...) 2
        if let Some(packet_params) = conf.packet_params {
            self.set_packet_params(spi, delay, packet_params)?;
        }

        // 10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1,
        // DIO2 or DIO3)
        self.set_dio_irq_params(
            spi,
            delay,
            conf.dio1_irq_mask,
            conf.dio1_irq_mask,
            conf.dio2_irq_mask,
            conf.dio3_irq_mask,
        )?;
        self.set_dio2_as_rf_switch_ctrl(spi, delay, true)?;

        // 11. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access
        self.set_sync_word(spi, delay, conf.sync_word)

        // The rest of the steps are done by the user
    }

    /// Set the LoRa Sync word
    /// Use 0x3444 for public networks like TTN
    /// Use 0x1424 for private networks
    pub fn set_sync_word(
        &mut self,
        spi: &mut TSPI,
        delay: &mut impl DelayUs<u32>,
        sync_word: u16,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        self.write_register(
            spi,
            delay,
            Register::LoRaSyncWordMsb,
            &sync_word.to_be_bytes(),
        )
    }

    /// Set the modem packet type, which can be either GFSK of LoRa
    /// Note: GFSK is not fully supported by this crate at the moment
    pub fn set_packet_type<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        packet_type: PacketType,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x8A, packet_type as u8]).map_err(Into::into)
    }

    /// Put the modem in standby mode
    pub fn set_standby<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        standby_config: StandbyConfig,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x80, standby_config as u8]).map_err(Into::into)
    }

    /// Get the current status of the modem
    pub fn get_status<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut result = [NOP];
        spi.transfer(&mut [0xC0])
            .and_then(|_| spi.transfer(&mut result))?;

        Ok(result[0].into())
    }

    pub fn set_fs<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay).unwrap();
        spi.transfer(&mut [0xC1])?;
        Ok(())
    }

    pub fn get_stats<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<Stats, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut result = [NOP; 7];
        spi.transfer(&mut [0x10])
            .and_then(|_| spi.transfer(&mut result))?;

        Ok(result.into())
    }

    /// Calibrate image
    pub fn calibrate_image<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        freq: CalibImageFreq,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let freq: [u8; 2] = freq.into();
        spi.write(&[0x98])
            .and_then(|_| spi.write(&freq))
            .map_err(Into::into)
    }

    /// Calibrate modem
    pub fn calibrate<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        calib_param: CalibParam,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x89, calib_param.into()]).map_err(Into::into)
    }

    /// Write data into a register
    pub fn write_register<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        register: Register,
        data: &[u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let start_addr = (register as u16).to_be_bytes();

        let mut spi = self.slave_select(spi, delay)?;
        let r = spi
            .write(&[0x0D])
            .and_then(|_| spi.write(&start_addr))
            .and_then(|_| spi.write(data))?;
        Ok(r)
    }

    /// Read data from a register
    pub fn read_register<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        start_addr: u16,
        result: &mut [u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        debug_assert!(result.len() >= 1);
        let mut start_addr = start_addr.to_be_bytes();
        let mut spi = self.slave_select(spi, delay)?;

        spi.transfer(&mut [0x1D])
            .and_then(|_| spi.transfer(&mut start_addr))
            .and_then(|_| spi.transfer(result))?;
        Ok(())
    }

    /// Write data into the buffer at the defined offset
    pub fn write_buffer<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        offset: u8,
        data: &[u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x0E, offset])
            .and_then(|_| spi.write(data))
            .map_err(Into::into)
    }

    /// Read data from the data from the defined offset
    pub fn read_buffer<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        offset: u8,
        result: &mut [u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut header = [0x1E, offset, NOP];
        spi.transfer(&mut header)
            .and_then(|_| spi.transfer(result))
            .map(|_| {})
            .map_err(Into::into)
    }

    /// Configure the dio2 pin as RF control switch
    pub fn set_dio2_as_rf_switch_ctrl<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        enable: bool,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x9D, enable as u8]).map_err(Into::into)
    }

    pub fn get_packet_status<'spu>(
        &'spu mut self,
        spi: &'spu mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<PacketStatus, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut result = [NOP; 3];
        spi.transfer(&mut [0x14, NOP])
            .and_then(|_| spi.transfer(&mut result))?;

        Ok(result.into())
    }

    /// Configure the dio3 pin as TCXO control switch
    pub fn set_dio3_as_tcxo_ctrl<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        tcxo_voltage: TcxoVoltage,
        tcxo_delay: TcxoDelay,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let tcxo_delay: [u8; 3] = tcxo_delay.into();
        spi.write(&[0x97, tcxo_voltage as u8])
            .and_then(|_| spi.write(&tcxo_delay))
            .map_err(Into::into)
    }

    /// Clear device error register
    pub fn clear_device_errors<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x07, NOP, NOP]).map_err(Into::into)
    }

    /// Get current device errors
    pub fn get_device_errors<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<DeviceErrors, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut result = [NOP; 2];
        spi.transfer(&mut [0x17, NOP])
            .and_then(|_| spi.transfer(&mut result))?;
        Ok(DeviceErrors::from(u16::from_le_bytes(result)))
    }

    /// Reset the device py pulling nrst low for a while
    pub fn reset(&mut self, delay: &mut impl DelayUs<u32>) -> Result<(), PinError<TPINERR>> {
        cortex_m::interrupt::free(|_| {
            self.nrst_pin.set_low().map_err(PinError::Output)?;
            // 8.1: The pin should be held low for typically 100 μs for the Reset to happen
            delay.delay_us(200);
            self.nrst_pin.set_high().map_err(PinError::Output)
        })
    }

    /// Enable antenna
    pub fn set_ant_enabled(&mut self, enabled: bool) -> Result<(), TPINERR> {
        if enabled {
            self.ant_pin.set_high()
        } else {
            self.ant_pin.set_low()
        }
    }

    /// Configure IRQ
    pub fn set_dio_irq_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        irq_mask: IrqMask,
        dio1_mask: IrqMask,
        dio2_mask: IrqMask,
        dio3_mask: IrqMask,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x08])
            .and_then(|_| spi.write(&(Into::<u16>::into(irq_mask)).to_be_bytes()))
            .and_then(|_| spi.write(&(Into::<u16>::into(dio1_mask)).to_be_bytes()))
            .and_then(|_| spi.write(&(Into::<u16>::into(dio2_mask)).to_be_bytes()))
            .and_then(|_| spi.write(&(Into::<u16>::into(dio3_mask)).to_be_bytes()))
            .map_err(Into::into)
    }

    /// Get the current IRQ status
    pub fn get_irq_status<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<IrqStatus, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x12])?;
        let mut status = [NOP, NOP];
        spi.transfer(&mut status)?;
        Ok(u16::from_be_bytes(status).into())
    }

    /// Clear the IRQ status
    pub fn clear_irq_status<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        mask: IrqMask,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mask: u16 = mask.into();
        spi.write(&[0x02])
            .and_then(|_| spi.write(&mask.to_be_bytes()))
            .map_err(Into::into)
    }

    /// Put the device in TX mode. It will start sending the data written in the buffer,
    /// starting at the configured offset
    pub fn set_tx<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        timeout: RxTxTimeout,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut timeout: [u8; 3] = timeout.into();

        let _ = spi.write(&[0x83]);
        spi.transfer(&mut timeout)?;
        Ok(timeout[0].into())
    }

    pub fn set_rx<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        timeout: RxTxTimeout,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut timeout: [u8; 3] = timeout.into();

        let _ = spi.write(&[0x82]);
        spi.transfer(&mut timeout)?;
        Ok(timeout[0].into())
    }

    /// Set packet parameters
    pub fn set_packet_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        params: PacketParams,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let params: [u8; 9] = params.into();
        spi.write(&[0x8C])
            .and_then(|_| spi.write(&params))
            .map_err(Into::into)
    }

    /// Set modulation parameters
    pub fn set_mod_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        params: ModParams,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let params: [u8; 8] = params.into();
        spi.write(&[0x8B])
            .and_then(|_| spi.write(&params))
            .map_err(Into::into)
    }

    /// Set TX parameters
    pub fn set_tx_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        params: TxParams,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let params: [u8; 2] = params.into();
        spi.write(&[0x8E])
            .and_then(|_| spi.write(&params))
            .map_err(Into::into)
    }

    /// Set RF frequency. This writes the passed rf_freq directly to the modem.
    /// Use sx1262::calc_rf_freq to calulate the correct value based
    /// On the XTAL frequency and the desired RF frequency
    pub fn set_rf_frequency<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        rf_freq: u32,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x86])
            .and_then(|_| spi.write(&rf_freq.to_be_bytes()))
            .map_err(Into::into)
    }

    /// Set Power Amplifier configuration
    pub fn set_pa_config<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        pa_config: PaConfig,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let pa_config: [u8; 4] = pa_config.into();
        spi.write(&[0x95])
            .and_then(|_| spi.write(&pa_config))
            .map_err(Into::into)
    }

    /// Configure the base addresses in the buffer
    pub fn set_buffer_base_address<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        tx_base_addr: u8,
        rx_base_addr: u8,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        spi.write(&[0x8F, tx_base_addr, rx_base_addr])
            .map_err(Into::into)
    }

    /// High level method to send a message. This methods writes the data in the buffer,
    /// puts the device in TX mode, and waits until the devices
    /// is done sending the data or a timeout occurs.
    /// Please note that this method updates the packet params
    pub fn write_bytes<'spi, 'data>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
        data: &'data [u8],
        timeout: RxTxTimeout,
        preamble_len: u16,
        crc_type: packet::lora::LoRaCrcType,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        use packet::lora::LoRaPacketParams;
        // Write data to buffer
        self.write_buffer(spi, delay, 0x00, data)?;

        // Set packet params
        let params = LoRaPacketParams::default()
            .set_preamble_len(preamble_len)
            .set_payload_len(data.len() as u8)
            .set_crc_type(crc_type)
            .into();

        self.set_packet_params(spi, delay, params)?;

        // Set tx mode
        let status = self.set_tx(spi, delay, timeout)?;
        // Wait for busy line to go low
        self.wait_on_busy(delay)?;
        // Wait on dio1 going high
        self.wait_on_dio1()?;
        // Clear IRQ
        self.clear_irq_status(spi, delay, IrqMask::all())?;
        // Write completed!
        Ok(status)
    }

    /// Get Rx buffer status, containing the length of the last received packet
    /// and the address of the first byte received.
    pub fn get_rx_buffer_status<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<RxBufferStatus, SxError<TSPIERR, TPINERR>> {
        let mut spi = self.slave_select(spi, delay)?;
        let mut result = [NOP, NOP];
        spi.transfer(&mut [0x13, NOP])
            .and_then(|_| spi.transfer(&mut result))?;

        Ok(result.into())
    }

    /// Busily wait for the busy pin to go low
    fn wait_on_busy(&mut self, delay: &mut impl DelayUs<u32>) -> Result<(), PinError<TPINERR>> {
        // 8.3.1: The max value for T SW from NSS rising edge to the BUSY rising edge is, in all cases, 600 ns
        delay.delay_us(1);
        while let Ok(true) = self.busy_pin.is_high() {
            cortex_m::asm::nop();
        }
        Ok(())
    }

    /// Busily wait for the dio1 pin to go high
    fn wait_on_dio1(&mut self) -> Result<(), PinError<TPINERR>> {
        while let Ok(true) = self.dio1_pin.is_low() {
            cortex_m::asm::nop();
        }
        Ok(())
    }

    /// Waits until the busy pin goes low, then pulls the nss pin low,
    /// and waits for a microsecond before returning a SlaveSelectGuard,
    /// which can be used to write data
    fn slave_select<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        delay: &mut impl DelayUs<u32>,
    ) -> Result<SlaveSelectGuard<TNSS, TSPI>, PinError<TPINERR>> {
        self.wait_on_busy(delay)?;
        let s = self.slave_select.select(spi)?;
        // Table 8-1: Data sheet specifies a minumum delay of 32ns between falling edge of nss and sck setup,
        // though embedded_hal provides no trait for delaying in nanosecond resolution.
        delay.delay_us(1);
        Ok(s)
    }
}
