//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{Clock},
    gpio,
    gpio::bank0::Gpio18,
    gpio::bank0::Gpio19,
    i2c::I2C,
    //gpio::Pins,
    pac,
    sio::Sio,
};

use bsp::Pins;

// The trait used by formatting macros like write! and writeln!
// use core::fmt::Write as FmtWrite;

// The macro for our start-up function
use cortex_m_rt::entry;

// I2C HAL traits & Types.
use embedded_hal::blocking::i2c::{/*Operation, Read, Transactional,*/ Write};

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use bsp::hal::prelude::*;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use bsp::hal;

use cortex_m::delay::Delay;

const LCD_ADDRESS: u8 = 0x27;
const En: u8 = 0x04;
const Backlight: u8 = 0x08;

use hal::gpio::bank0::BankPinId;
use hal::gpio::FunctionI2C;
use hal::gpio::Pin;
use hal::gpio::PinId;
use hal::i2c::SclPin;
use hal::i2c::SdaPin;
use pac::I2C1;

type MyPins = I2C<
    I2C1,
    (
        gpio::Pin<Gpio18, gpio::Function<gpio::I2C>>,
        gpio::Pin<Gpio19, gpio::Function<gpio::I2C>>,
    ),
>;

type MyWritePins = I2C<
    I2C1,
    (
        gpio::Pin<Gpio18, gpio::Function<gpio::I2C>>,
        gpio::Pin<Gpio19, gpio::Function<gpio::I2C>>,
    ),
>;

fn write4bits(delay: &mut Delay, i2c: &mut MyWritePins, data: u8) {
    i2c.write(LCD_ADDRESS, &[data | En | Backlight]);
    delay.delay_ms(1);
    i2c.write(LCD_ADDRESS, &[Backlight]);
    delay.delay_ms(5);
}

fn send(delay: &mut Delay, i2c: &mut MyWritePins, data: u8, mode: u8) {
    let high_bits: u8 = data & 0xf0;
    let low_bits: u8 = (data << 4) & 0xf0;
    write4bits(delay, i2c, high_bits | mode);
    write4bits(delay, i2c, low_bits | mode);
}

fn command(delay: &mut Delay, i2c: &mut MyWritePins, data: u8) {
    send(delay, i2c, data, 0x00);
}

fn write_lcd(delay: &mut Delay, i2c: &mut MyWritePins, data: u8) {
    send(delay, i2c, data, 0x01);
}

#[entry]
fn main() -> ! {
    //info!("Program start");
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());


    let mut i2c: MyPins = I2C::i2c1(
        pac.I2C1,
        pins.gpio18.into_mode(), // sda
        pins.gpio19.into_mode(), // scl
        400.kHz(),
        //100.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq().integer().Hz(),
    );

    // Scan for devices on the bus by attempting to read from them
    /*
    use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
    for i in 0..=127 {
        let mut readbuf: [u8; 1] = [0; 1];
        let result = i2c.read(i, &mut readbuf);
        if let Ok(d) = result {
            // Do whatever work you want to do with found devices
            // writeln!(uart, "Device found at address{:?}", i).unwrap();
        }
    }
    */

    /* initialize the LCD */
    write4bits(&mut delay, &mut i2c, 0x03 << 4); // Set to 8-bit mode
    delay.delay_ms(5);
    write4bits(&mut delay, &mut i2c, 0x03 << 4); // Set to 8-bit mode
    delay.delay_ms(5);
    write4bits(&mut delay, &mut i2c, 0x03 << 4); // Set to 8-bit mode
    delay.delay_ms(5);
    write4bits(&mut delay, &mut i2c, 0x02 << 4); // Set to 4-bit mode (while in 8-bit mode)

    led_pin.set_low().unwrap();

    command(
        &mut delay,
        &mut i2c,
        0x20 as u8 | // Function set command
        0x00 as u8 | // 5x8 display
        0x08 as u8   // Two line display
    );

    command(
        &mut delay,
        &mut i2c,
        0x08 as u8 | // Display control command
        0x04 as u8 | // Display on
        0x02 as u8 | // Cursor on
        0x01 as u8   // Blink on
    );

     command(
        &mut delay,
        &mut i2c,
        0x01 as u8 // Clear display
    );

    command(
        &mut delay,
        &mut i2c,
        0x04 as u8 | // Entry mode command
        0x02 as u8   // Entry right
    );

    let hello: &'static str = "Hello, World!\nHAHAHA";
    for c in hello.chars() {
        write_lcd(&mut delay, &mut i2c, c as u8);
    }

    loop {
        //info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        //info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
