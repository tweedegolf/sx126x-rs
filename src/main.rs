#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32f1xx_hal::prelude::*;

mod gpio;
mod ring_buf;
mod sx126x;
mod usart;

use gpio::DisconnectedPin;
use sx126x::conf::Config as LoRaConfig;
use sx126x::op::{
    calib::CalibParam,
    irq::{IrqMask, IrqMaskBit::Timeout, IrqMaskBit::TxDone},
    modulation::lora::LoraModParams,
    packet::lora::LoRaPacketParams,
    tx::{DeviceSel::SX1261, PaConfig, RampTime, TxParams},
    PacketType::LoRa,
    StandbyConfig::StbyRc,
    TcxoDelay,
    TcxoVoltage::Volt1_7,
};
use sx126x::SX126x;

use embedded_hal::digital::v2::OutputPin;
use panic_semihosting as _;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::gpio::State::High;
use stm32f1xx_hal::spi::{Mode as SpiMode, Phase, Polarity, Spi};
use stm32f1xx_hal::stm32;

#[entry]
fn main() -> ! {
    let peripherals = stm32::Peripherals::take().unwrap();
    let core_peripherals = stm32::CorePeripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);

    // Init SPI1
    let spi1_sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let spi1_miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let spi1_mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let spi1_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi1_freq = 100.khz();

    let spi1_pins = (
        spi1_sck,  // D13
        spi1_miso, // D12
        spi1_mosi, // D11
    );

    let spi1 = &mut Spi::spi1(
        peripherals.SPI1,
        spi1_pins,
        &mut afio.mapr,
        spi1_mode,
        spi1_freq,
        clocks,
        &mut rcc.apb2,
    );

    // Init pins
    let lora_nreset = gpioa
        .pa0
        .into_push_pull_output_with_state(&mut gpioa.crl, High);
    let lora_nss = gpioa
        .pa8
        .into_push_pull_output_with_state(&mut gpioa.crh, High);
    let lora_busy = gpiob.pb5.into_floating_input(&mut gpiob.crl);
    let lora_dio1 = gpiob.pb10.into_floating_input(&mut gpiob.crh);
    let lora_dio2 = DisconnectedPin;
    let lora_ant = gpioa
        .pa9
        .into_push_pull_output_with_state(&mut gpioa.crh, High);

    let lora_pins = (
        lora_nss,    // D7
        lora_nreset, // A0
        lora_busy,   // D4
        lora_ant,    // D8
        lora_dio1,   // D6
        lora_dio2,   // NC
    );

    let delay = &mut Delay::new(core_peripherals.SYST, clocks);

    // // Init LoRa modem
    let packet_params = LoRaPacketParams::default()
        .set_preamble_len(8)
        .set_payload_len(32)
        .into();
    let mod_params = LoraModParams::default().into();
    let tx_params = TxParams::default()
        .set_power_dbm(13)
        .set_ramp_time(RampTime::Ramp200u);
    let pa_config = PaConfig::default()
        .set_device_sel(SX1261)
        .set_pa_duty_cycle(0x04);

    let dio1_irq_mask = IrqMask::none().combine(TxDone).combine(Timeout);

    let conf = LoRaConfig {
        packet_type: LoRa,
        standby_config: StbyRc,
        sync_word: 0x1424, // Private networks
        #[cfg(feature = "tcxo")]
        tcxo_delay: TcxoDelay::from_us(5000),
        #[cfg(feature = "tcxo")]
        tcxo_voltage: Volt1_7,
        calib_param: CalibParam::from(0x7F & 0b1111_1111),
        packet_params,
        mod_params,
        tx_params,
        pa_config,
        dio1_irq_mask,
        rf_freq: 868_000_000, // 868MHz (EU)
    };

    let mut lora = SX126x::init(spi1, delay, lora_pins, conf).unwrap();

    let timeout = sx126x::op::tx::TxTimeout::from(0x000000); // timeout disabled

    // Blink LED to indicate the whole program has run to completion
    let mut led_pin = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);
    loop {
        led_pin.set_high().unwrap();
        // Send LoRa message

        let last_status = lora
            .write_bytes(spi1, delay, b"Hello, LoRa World!", timeout)
            .unwrap();
        cortex_m_semihosting::hprintln!("{:?}", last_status).unwrap();
        led_pin.set_low().unwrap();

        delay.delay_ms(1000u16);
    }
}
