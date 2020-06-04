#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{hprintln};
use stm32f3xx_hal::prelude::*;

mod ring_buf;
mod sx126x;

use ring_buf::RingBuf;
use sx126x::SX126x;
use stm32f3xx_hal::spi::{Spi, Mode as SpiMode, Polarity, Phase};
use stm32f3xx_hal::delay::Delay;

#[panic_handler]
unsafe fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    cortex_m_semihosting::hprintln!("ERROR! {:?}", info).unwrap();
    loop {}
}

#[entry]
fn main() -> ! {
    hprintln!("init").unwrap();
    
    let peripherals = stm32f3xx_hal::stm32::Peripherals::take().unwrap();
    let core_peripherals = stm32f3xx_hal::stm32::CorePeripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let clocks = rcc.cfgr.use_hse(16.mhz()).freeze(&mut flash.acr);
    
    let mut gpioa = peripherals.GPIOA.split(&mut rcc.ahb);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.ahb);

    let usart1_txd = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let usart1_rxd = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);

    let usart1 = stm32f3xx_hal::serial::Serial::usart1(
        peripherals.USART1,
        (usart1_txd, usart1_rxd),
        9600.bps(),
        clocks,
        &mut rcc.apb2,
    );

    let (mut usart1_tx, mut usart1_rx) = usart1.split();
    hprintln!("Setting op SPI 3 for LoRa modem").unwrap();
    
    let spi3_sck = gpioc.pc10.into_af6(&mut gpioc.moder, &mut gpioc.afrh);
    let spi3_miso = gpioc.pc11.into_af6(&mut gpioc.moder, &mut gpioc.afrh);
    let spi3_mosi = gpioc.pc12.into_af6(&mut gpioc.moder, &mut gpioc.afrh);
    let spi3_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi3_freq = 16.mhz();

    let mut spi3 = Spi::spi3(peripherals.SPI3, (spi3_sck, spi3_miso, spi3_mosi), spi3_mode, spi3_freq, clocks, &mut rcc.apb1);
    let lora_nreset = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    let lora_nss = gpioc.pc14.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    let lora_busy = gpioc.pc15.into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);

    let mut delay = Delay::new(core_peripherals.SYST, clocks);

    hprintln!("Setting op LoRa modem").unwrap();
    
    let mut lora = SX126x::init(&mut spi3, &mut delay, (lora_nreset, lora_nss, lora_busy)).unwrap();

    hprintln!("Done setting op LoRa modem").unwrap();
    let mut buf = RingBuf::new();

    loop {        
        if let Ok(b) = usart1_rx.read() {
            buf.push_back(b);
        }
        
        if let Ok(b) = buf.get(0) {
            if let Ok(_) = usart1_tx.write(b) {
                buf.pop_front();
            }
        }
    }
}
