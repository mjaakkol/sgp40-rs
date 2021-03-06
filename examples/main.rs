use hal::{Delay, I2cdev};
/// Sensirion SGP40 sensor device driver example.
use linux_embedded_hal as hal;

use std::thread;
use std::time::Duration;

use sgp40::Sgp40;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();

    let mut sensor = Sgp40::new(dev, 0x59, Delay);

    loop {
        // This is not the recommended way to operate the sensor but turned out to be
        // super useful when running two sensors parallel.
        if let Ok(result) = sensor.measure_voc_index() {
            println!("VOC index: {}", result);
        } else {
            println!("Failed I2C reading");
        }

        thread::sleep(Duration::new(1_u64, 0));
    }
}
