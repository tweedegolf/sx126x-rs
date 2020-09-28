#![no_std]
#![no_main]

use sx126x::conf::Config as LoRaConfig;
use sx126x::op::*;
use sx126x::SX126x;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use panic_semihosting as _;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::gpio::State::High;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::spi::{Mode as SpiMode, Phase, Polarity, Spi};
use stm32f1xx_hal::stm32;

const RF_FREQUENCY: u32 = 868_000_000; // 868MHz (EU)
const F_XTAL: u32 = 32_000_000;

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

    // ===== Init SPI1 =====
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

    // ===== Init pins =====
    let lora_nreset = gpioa
        .pa0
        .into_push_pull_output_with_state(&mut gpioa.crl, High);
    let lora_nss = gpioa
        .pa8
        .into_push_pull_output_with_state(&mut gpioa.crh, High);
    let lora_busy = gpiob.pb5.into_floating_input(&mut gpiob.crl);
    let lora_dio1 = gpiob.pb10.into_floating_input(&mut gpiob.crh);
    let lora_ant = gpioa
        .pa9
        .into_push_pull_output_with_state(&mut gpioa.crh, High);

    let lora_pins = (
        lora_nss,    // D7
        lora_nreset, // A0
        lora_busy,   // D4
        lora_ant,    // D8
        lora_dio1,   // D6
    );

    let delay = &mut Delay::new(core_peripherals.SYST, clocks);

    // ===== Init LoRa modem =====
    let conf = build_config();
    let mut lora = SX126x::new(lora_pins);
    lora.init(spi1, delay, conf).unwrap();

    let timeout = 0.into(); // timeout disabled
    let crc_type = packet::lora::LoRaCrcType::CrcOn;

    // Blink LED to indicate the whole program has run to completion
    let mut led_pin = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

    loop {
        led_pin.set_high().unwrap();
        // Send LoRa message

        lora.write_bytes(spi1, delay, b"Hello, LoRa World!", timeout, 8, crc_type)
            .unwrap();

        led_pin.set_low().unwrap();

        delay.delay_ms(1000u16);
    }
}

fn build_config() -> LoRaConfig {
    use sx126x::op::{
        irq::{IrqMaskBit::Timeout, IrqMaskBit::TxDone},
        modulation::lora::LoraModParams,
        rxtx::DeviceSel::SX1261,
        PacketType::LoRa,
    };

    let mod_params = LoraModParams::default().into();
    let tx_params = TxParams::default()
        .set_power_dbm(14)
        .set_ramp_time(RampTime::Ramp200u);
    let pa_config = PaConfig::default()
        .set_device_sel(SX1261)
        .set_pa_duty_cycle(0x04);

    let dio1_irq_mask = IrqMask::none().combine(TxDone).combine(Timeout);

    let rf_freq = sx126x::calc_rf_freq(RF_FREQUENCY as f32, F_XTAL as f32);

    LoRaConfig {
        packet_type: LoRa,
        sync_word: 0x1424, // Private networks
        calib_param: CalibParam::from(0x7F & 0b1111_1111),
        mod_params,
        tx_params,
        pa_config,
        // We want to set these just before sending a message,
        // as it contains the payload length
        packet_params: None,
        dio1_irq_mask,
        dio2_irq_mask: IrqMask::none(),
        dio3_irq_mask: IrqMask::none(),
        rf_frequency: RF_FREQUENCY,
        rf_freq,
    }
}
