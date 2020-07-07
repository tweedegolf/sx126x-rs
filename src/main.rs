#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::prelude::*;

mod ring_buf;
mod sx126x;
mod usart;

use ring_buf::RingBuf;
use sx126x::{SX126x, Config as LoRaConfig};
use sx126x::op::{PacketType::LORA, StandbyConfig::StbyRc, TcxoDelay, TcxoVoltage::Volt_1_7};

use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::serial::Config;
use stm32f1xx_hal::spi::{Mode as SpiMode, Phase, Polarity, Spi};
use stm32f1xx_hal::stm32;

#[panic_handler]
unsafe fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    cortex_m_semihosting::hprintln!("ERROR! {:?}", info).unwrap();
    loop {}
}

#[entry]
fn main() -> ! {

    use embedded_hal::digital::v2::OutputPin;
    // hprintln!("init").unwrap();

    let peripherals = stm32::Peripherals::take().unwrap();
    let core_peripherals = stm32::CorePeripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

    let clocks = rcc.cfgr.use_hse(16.mhz()).freeze(&mut flash.acr);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);

    let usart2_txd = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let usart2_rxd = gpioa.pa3.into_floating_input(&mut gpioa.crl);

    let usart2 = stm32f1xx_hal::serial::Serial::usart2(
        peripherals.USART2,
        (usart2_txd, usart2_rxd),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1,
    );

    let (mut usart2_tx, mut usart2_rx) = usart2.split();
    let mut serial_tx = usart::UsartWrite::init(usart2_tx);

    // hprintln!("Setting op SPI 3 for LoRa modem").unwrap();

    let spi1_sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let spi1_miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let spi1_mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let spi1_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi1_freq = 16.mhz();

    let mut spi1 = Spi::spi1(
        peripherals.SPI1,
        (spi1_sck, spi1_miso, spi1_mosi),
        &mut afio.mapr,
        spi1_mode,
        spi1_freq,
        clocks,
        &mut rcc.apb2,
    );
    let lora_nreset = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);
    let lora_nss = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    let lora_busy = gpiob.pb5.into_floating_input(&mut gpiob.crl);
    let _lora_dio_1 = gpioa.pa10.into_floating_input(&mut gpioa.crh);
    let lora_ant = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);

    let mut delay = Delay::new(core_peripherals.SYST, clocks);

    // hprintln!("Setting op LoRa modem").unwrap();

    let conf = LoRaConfig {
        freq_1: 0xD7,
        freq_2: 0xDB,
        packet_type: LORA,
        standby_config: StbyRc,
        
        sync_word: 0x1424, // Private networks
        tcxo_delay: TcxoDelay::from_millis(100),
        tcxo_voltage: Volt_1_7,

    };

    let mut lora = SX126x::init(
        &mut spi1,
        &mut delay,
        (lora_nreset, lora_nss, lora_busy, lora_ant),
        conf,
    )
    .unwrap();

    // hprintln!("Done setting op LoRa modem").unwrap();

    let mut led_pin = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(100u8);
        led_pin.set_low().unwrap();
        delay.delay_ms(100u8);
    }
}
