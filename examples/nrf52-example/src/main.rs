#![no_main]
#![no_std]

use rtt_target::{rprintln, rtt_init_print};

// access to board peripherals:
use nrf52840_hal::{
    self as hal,
    twim::{self, Twim},
    Timer,
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
    let delay = Timer::new(p.TIMER1);

    let pins_1 = hal::gpio::p1::Parts::new(p.P1);

    let scl = pins_1.p1_15.into_floating_input().degrade();
    let sda = pins_1.p1_13.into_floating_input().degrade();

    let pins = twim::Pins { scl, sda };

    let i2c = Twim::new(p.TWIM1, pins, twim::Frequency::K100);

    let mut sensor = Sgp40::new(i2c, 0x59, delay);

    loop {
        if let Ok(result) = sensor.measure_voc_index() {
            rprintln!("VOC index: {}", result);
        } else {
            rprintln!("Failed I2C reading");
        }
    }
}
