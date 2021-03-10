#![no_main]
#![no_std]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use rtt_target::{rprintln, rtt_init_print};

// access to board peripherals:
use nrf52840_hal::{
    self as hal,
    gpio::{p0::Parts as P0Parts, p1::Parts as P1Parts, Level},
    prelude::*,
    spim::{self, Spim},
    twim::{self, Error, Instance, Twim},
    Temp, Timer,
};

use sgp40::*;

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let mut delay = Timer::new(p.TIMER1);

    let pins_1 = hal::gpio::p1::Parts::new(p.P1);

    let scl1 = pins_1.p1_13.into_floating_input().degrade();
    let sda1 = pins_1.p1_15.into_floating_input().degrade();

    let pins1 = twim::Pins { scl: scl1, sda: sda1 };

    let i2c1 = Twim::new(p.TWIM1, pins1, twim::Frequency::K100);

    let sgp40 = Sgp40::new(i2c1, 0x31, delay.clone());

    loop {}
}
