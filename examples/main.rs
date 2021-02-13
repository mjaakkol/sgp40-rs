// Sensirion SGP40 sensor device driver example.
use linux_embedded_hal as hal;
use hal::{Delay, I2cdev};

use std::thread;
use std::time::Duration;

use sgp40::Sgp40;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();

    let mut sensor = Sgp40::new(dev, 0x59, Delay);

    //let serial = sensor.serial().unwrap();
    //println!("Serial number is {:x}", serial);

    loop {
        //let raw = sensor.measure_raw().unwrap();

        //println!("RAW: {}", raw);

        let index = sensor.measure_voc_index().unwrap();

        println!("VOC index: {}", index);

        //thread::sleep(Duration::new(1_u64, 0));

        //let raw = sensor.measure_raw_with_rht(50,25).unwrap();
        //println!("RAW {}", raw);

        thread::sleep(Duration::new(1_u64, 0));
    }
}
