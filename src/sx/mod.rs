pub(crate) mod err;

use core::convert::TryInto;
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::Operation;
use embedded_hal::spi::SpiDevice;
use err::SpiError;

use crate::conf::Config;
use crate::op::*;
use crate::reg::*;
// use err::OutputPinError;

use self::err::{PinError, SxError};

type Pins<TNRST, TBUSY, TANT, TDIO1> = (TNRST, TBUSY, TANT, TDIO1);

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
pub struct SX126x<TSPI: SpiDevice, TNRST, TBUSY, TANT, TDIO1> {
    spi: PhantomData<TSPI>,
    nrst_pin: TNRST,
    busy_pin: TBUSY,
    ant_pin: TANT,
    dio1_pin: TDIO1,
}

impl<TSPI, TNRST, TBUSY, TANT, TDIO1, TSPIERR, TPINERR> SX126x<TSPI, TNRST, TBUSY, TANT, TDIO1>
where
    TPINERR: core::fmt::Debug,
    TSPI: SpiDevice<Error = TSPIERR>,
    TNRST: OutputPin<Error = TPINERR>,
    TBUSY: InputPin<Error = TPINERR>,
    TANT: OutputPin<Error = TPINERR>,
    TDIO1: InputPin<Error = TPINERR>,
{
    // Create a new SX126x
    pub fn new(pins: Pins<TNRST, TBUSY, TANT, TDIO1>) -> Self {
        let (nrst_pin, busy_pin, ant_pin, dio1_pin) = pins;
        Self {
            spi: PhantomData,
            nrst_pin,
            busy_pin,
            ant_pin,
            dio1_pin,
        }
    }

    // Initialize and configure the SX126x using the provided Config
    pub fn init(&mut self, spi: &mut TSPI, conf: Config) -> Result<(), SxError<TSPIERR, TPINERR>> {
        // Reset the sx
        self.reset(spi)?;
        self.wait_on_busy(spi)?;

        // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
        self.set_standby(spi, crate::op::StandbyConfig::StbyRc)?;
        self.wait_on_busy(spi)?;

        // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
        self.set_packet_type(spi, conf.packet_type)?;
        self.wait_on_busy(spi)?;

        // 3. Define the RF frequency with the command SetRfFrequency(...)
        self.set_rf_frequency(spi, conf.rf_freq)?;
        self.wait_on_busy(spi)?;

        if let Some((tcxo_voltage, tcxo_delay)) = conf.tcxo_opts {
            self.set_dio3_as_tcxo_ctrl(spi, tcxo_voltage, tcxo_delay)?;
            self.wait_on_busy(spi)?;
        }

        // Calibrate
        self.calibrate(spi, conf.calib_param)?;
        self.wait_on_busy(spi)?;
        self.calibrate_image(spi, CalibImageFreq::from_rf_frequency(conf.rf_frequency))?;
        self.wait_on_busy(spi)?;

        // 4. Define the Power Amplifier configuration with the command SetPaConfig(...)
        self.set_pa_config(spi, conf.pa_config)?;
        self.wait_on_busy(spi)?;

        // 5. Define output power and ramping time with the command SetTxParams(...)
        self.set_tx_params(spi, conf.tx_params)?;
        self.wait_on_busy(spi)?;

        // 6. Define where the data payload will be stored with the command SetBufferBaseAddress(...)
        self.set_buffer_base_address(spi, 0x00, 0x00)?;
        self.wait_on_busy(spi)?;

        // 7. Send the payload to the data buffer with the command WriteBuffer(...)
        // This is done later in SX126x::write_bytes

        // 8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...) 1
        self.set_mod_params(spi, conf.mod_params)?;
        self.wait_on_busy(spi)?;

        // 9. Define the frame format to be used with the command SetPacketParams(...) 2
        if let Some(packet_params) = conf.packet_params {
            self.set_packet_params(spi, packet_params)?;
            self.wait_on_busy(spi)?;
        }

        // 10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1,
        // DIO2 or DIO3)
        self.set_dio_irq_params(
            spi,
            conf.dio1_irq_mask,
            conf.dio1_irq_mask,
            conf.dio2_irq_mask,
            conf.dio3_irq_mask,
        )?;
        self.wait_on_busy(spi)?;
        self.set_dio2_as_rf_switch_ctrl(spi, true)?;
        self.wait_on_busy(spi)?;

        // 11. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access
        self.set_sync_word(spi, conf.sync_word)?;
        self.wait_on_busy(spi)?;

        // The rest of the steps are done by the user
        Ok(())
    }

    /// Set the LoRa Sync word
    /// Use 0x3444 for public networks like TTN
    /// Use 0x1424 for private networks
    pub fn set_sync_word(
        &mut self,
        spi: &mut TSPI,
        sync_word: u16,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        self.write_register(spi, Register::LoRaSyncWordMsb, &sync_word.to_be_bytes())
    }

    /// Set the modem packet type, which can be either GFSK of LoRa
    /// Note: GFSK is not fully supported by this crate at the moment
    pub fn set_packet_type<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        packet_type: PacketType,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0x8A, packet_type as u8])
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Put the modem in standby mode
    pub fn set_standby<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        standby_config: StandbyConfig,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0x80, standby_config as u8])
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Get the current status of the modem
    pub fn get_status<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        let mut result = [0xC0, NOP];
        spi.transfer_in_place(&mut result)
            .map_err(SpiError::Transfer)?;

        Ok(result[1].into())
    }

    pub fn set_fs<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0xC1]).map_err(SpiError::Write)?;
        Ok(())
    }

    pub fn get_stats<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
    ) -> Result<Stats, SxError<TSPIERR, TPINERR>> {
        let mut result = [0x10, NOP, NOP, NOP, NOP, NOP, NOP, NOP];
        spi.transfer_in_place(&mut result)
            .map_err(SpiError::Transfer)?;

        Ok(TryInto::<[u8; 7]>::try_into(&result[1..]).unwrap().into())
    }

    /// Calibrate image
    pub fn calibrate_image<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        freq: CalibImageFreq,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let freq: [u8; 2] = freq.into();
        let mut ops = [Operation::Write(&[0x98]), Operation::Write(&freq)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Calibrate modem
    pub fn calibrate<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        calib_param: CalibParam,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0x89, calib_param.into()])
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Write data into a register
    pub fn write_register<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        register: Register,
        data: &[u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let start_addr = (register as u16).to_be_bytes();
        let mut ops = [
            Operation::Write(&[0x0D]),
            Operation::Write(&start_addr),
            Operation::Write(data),
        ];

        let r = spi.transaction(&mut ops).map_err(SpiError::Write)?;
        Ok(r)
    }

    /// Read data from a register
    pub fn read_register<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        start_addr: u16,
        result: &mut [u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        debug_assert!(result.len() >= 1);
        let start_addr = start_addr.to_be_bytes();

        let mut ops = [
            Operation::Write(&[0x1D]),
            Operation::Write(&start_addr),
            Operation::Read(result),
        ];

        spi.transaction(&mut ops).map_err(SpiError::Transfer)?;
        Ok(())
    }

    /// Write data into the buffer at the defined offset
    pub fn write_buffer<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        offset: u8,
        data: &[u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let header = [0x0E, offset];
        let mut ops = [Operation::Write(&header), Operation::Write(data)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Read data from the data from the defined offset
    pub fn read_buffer<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        offset: u8,
        result: &mut [u8],
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let header = [0x1E, offset, NOP];
        let mut ops = [Operation::Write(&header), Operation::Read(result)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Transfer)
            .map_err(Into::into)
    }

    /// Configure the dio2 pin as RF control switch
    pub fn set_dio2_as_rf_switch_ctrl<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        enable: bool,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0x9D, enable as u8])
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    pub fn get_packet_status<'spu>(
        &'spu mut self,
        spi: &'spu mut TSPI,
    ) -> Result<PacketStatus, SxError<TSPIERR, TPINERR>> {
        let header = [0x14, NOP];
        let mut result = [NOP; 3];
        let mut ops = [Operation::Write(&header), Operation::Read(&mut result)];
        spi.transaction(&mut ops).map_err(SpiError::Transfer)?;

        Ok(result.into())
    }

    /// Configure the dio3 pin as TCXO control switch
    pub fn set_dio3_as_tcxo_ctrl<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
        tcxo_voltage: TcxoVoltage,
        tcxo_delay: TcxoDelay,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let header = [0x97, tcxo_voltage as u8];
        let tcxo_delay: [u8; 3] = tcxo_delay.into();
        let mut ops = [Operation::Write(&header), Operation::Write(&tcxo_delay)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Clear device error register
    pub fn clear_device_errors<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0x07, NOP, NOP])
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Get current device errors
    pub fn get_device_errors<'spi>(
        &'spi mut self,
        spi: &'spi mut TSPI,
    ) -> Result<DeviceErrors, SxError<TSPIERR, TPINERR>> {
        let mut result = [0x17, NOP, NOP, NOP];
        spi.transfer_in_place(&mut result)
            .map_err(SpiError::Transfer)?;
        Ok(DeviceErrors::from(u16::from_le_bytes(
            result[2..].try_into().unwrap(),
        )))
    }

    /// Reset the device py pulling nrst low for a while
    pub fn reset<'spi>(&mut self, spi: &'spi mut TSPI) -> Result<(), SxError<TSPIERR, TPINERR>> {
        cortex_m::interrupt::free(|_| {
            self.nrst_pin.set_low().map_err(PinError::Output)?;
            // 8.1: The pin should be held low for typically 100 μs for the Reset to happen
            spi.transaction(&mut [Operation::DelayNs(200_000)])
                .map_err(SpiError::Write)?;
            self.nrst_pin
                .set_high()
                .map_err(PinError::Output)
                .map_err(Into::into)
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
        irq_mask: IrqMask,
        dio1_mask: IrqMask,
        dio2_mask: IrqMask,
        dio3_mask: IrqMask,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let irq = (Into::<u16>::into(irq_mask)).to_be_bytes();
        let dio1 = (Into::<u16>::into(dio1_mask)).to_be_bytes();
        let dio2 = (Into::<u16>::into(dio2_mask)).to_be_bytes();
        let dio3 = (Into::<u16>::into(dio3_mask)).to_be_bytes();
        let mut ops = [
            Operation::Write(&[0x08]),
            Operation::Write(&irq),
            Operation::Write(&dio1),
            Operation::Write(&dio2),
            Operation::Write(&dio3),
        ];
        spi.transaction(&mut ops)
            .map_err(SpiError::Transfer)
            .map_err(Into::into)
    }

    /// Get the current IRQ status
    pub fn get_irq_status<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
    ) -> Result<IrqStatus, SxError<TSPIERR, TPINERR>> {
        let mut status = [NOP, NOP];
        let mut ops = [Operation::Write(&[0x12]), Operation::Read(&mut status)];
        spi.transaction(&mut ops).map_err(SpiError::Transfer)?;
        Ok(u16::from_be_bytes(status).into())
    }

    /// Clear the IRQ status
    pub fn clear_irq_status<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        mask: IrqMask,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let mask = Into::<u16>::into(mask).to_be_bytes();
        let mut ops = [Operation::Write(&[0x02]), Operation::Write(&mask)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Put the device in TX mode. It will start sending the data written in the buffer,
    /// starting at the configured offset
    pub fn set_tx<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        timeout: RxTxTimeout,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        let mut buf = [0x83u8; 4];
        let timeout: [u8; 3] = timeout.into();
        buf[1..].copy_from_slice(&timeout);

        spi.transfer_in_place(&mut buf)
            .map_err(SpiError::Transfer)?;
        Ok(timeout[1].into())
    }

    pub fn set_rx<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        timeout: RxTxTimeout,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        let mut buf = [0x82u8; 4];
        let timeout: [u8; 3] = timeout.into();
        buf[1..].copy_from_slice(&timeout);

        spi.write(&buf).map_err(SpiError::Transfer)?;
        Ok(timeout[0].into())
    }

    /// Set packet parameters
    pub fn set_packet_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        params: PacketParams,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let params: [u8; 9] = params.into();
        let mut ops = [Operation::Write(&[0x8C]), Operation::Write(&params)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Set modulation parameters
    pub fn set_mod_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        params: ModParams,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let params: [u8; 8] = params.into();
        let mut ops = [Operation::Write(&[0x8B]), Operation::Write(&params)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Set TX parameters
    pub fn set_tx_params<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        params: TxParams,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let params: [u8; 2] = params.into();
        let mut ops = [Operation::Write(&[0x8E]), Operation::Write(&params)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Set RF frequency. This writes the passed rf_freq directly to the modem.
    /// Use sx1262::calc_rf_freq to calulate the correct value based
    /// On the XTAL frequency and the desired RF frequency
    pub fn set_rf_frequency<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        rf_freq: u32,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let rf_freq = rf_freq.to_be_bytes();
        let mut ops = [Operation::Write(&[0x86]), Operation::Write(&rf_freq)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Set Power Amplifier configuration
    pub fn set_pa_config<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        pa_config: PaConfig,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        let pa_config: [u8; 4] = pa_config.into();
        let mut ops = [Operation::Write(&[0x95]), Operation::Write(&pa_config)];
        spi.transaction(&mut ops)
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// Configure the base addresses in the buffer
    pub fn set_buffer_base_address<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
        tx_base_addr: u8,
        rx_base_addr: u8,
    ) -> Result<(), SxError<TSPIERR, TPINERR>> {
        spi.write(&[0x8F, tx_base_addr, rx_base_addr])
            .map_err(SpiError::Write)
            .map_err(Into::into)
    }

    /// High level method to send a message. This methods writes the data in the buffer,
    /// puts the device in TX mode, and waits until the devices
    /// is done sending the data or a timeout occurs.
    /// Please note that this method updates the packet params
    pub fn write_bytes<'spi, 'data>(
        &mut self,
        spi: &'spi mut TSPI,
        data: &'data [u8],
        timeout: RxTxTimeout,
        preamble_len: u16,
        crc_type: packet::lora::LoRaCrcType,
    ) -> Result<Status, SxError<TSPIERR, TPINERR>> {
        use packet::lora::LoRaPacketParams;
        // Write data to buffer
        self.write_buffer(spi, 0x00, data)?;

        // Set packet params
        let params = LoRaPacketParams::default()
            .set_preamble_len(preamble_len)
            .set_payload_len(data.len() as u8)
            .set_crc_type(crc_type)
            .into();

        self.set_packet_params(spi, params)?;

        // Set tx mode
        let status = self.set_tx(spi, timeout)?;
        // Wait for busy line to go low
        self.wait_on_busy(spi).map_err(SxError::Spi)?;
        // Wait on dio1 going high
        self.wait_on_dio1()?;
        // Clear IRQ
        self.clear_irq_status(spi, IrqMask::all())?;
        // Write completed!
        Ok(status)
    }

    /// Get Rx buffer status, containing the length of the last received packet
    /// and the address of the first byte received.
    pub fn get_rx_buffer_status<'spi>(
        &mut self,
        spi: &'spi mut TSPI,
    ) -> Result<RxBufferStatus, SxError<TSPIERR, TPINERR>> {
        let mut result = [0x13, NOP, NOP, NOP];
        spi.transfer_in_place(&mut result)
            .map_err(SpiError::Transfer)?;
        Ok(TryInto::<[u8; 2]>::try_into(&result[2..]).unwrap().into())
    }

    /// Busily wait for the busy pin to go low
    pub fn wait_on_busy<'spi>(&mut self, spi: &'spi mut TSPI) -> Result<(), SpiError<TSPIERR>> {
        spi.transaction(&mut [Operation::DelayNs(1000)])
            .map_err(SpiError::Transfer)?;
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
}
