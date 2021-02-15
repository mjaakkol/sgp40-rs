//! Platform agnostic Rust driver for Sensirion SGP40 device with
//! gas, temperature and humidity sensors based on
//! the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## Sensirion SGP40
//!
//! Sensirion SGP40 is a low-power accurate gas sensor for air quality application.
//! The sensor has different sampling rates to optimize power-consumption per application
//! bases as well as ability save and set the baseline for faster start-up accuracy.
//! The sensor uses I²C interface and measures TVOC (*Total Volatile Organic Compounds*)
//!
//! Datasheet: https://www.sensirion.com/file/datasheet_sgp40
//!
//! ## Usage
//!
//! ### Instantiating
//!
//! Import this crate and an `embedded_hal` implementation, then instantiate
//! the device:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//!
//! use hal::{Delay, I2cdev};
//! use sgp40::Sgp40;
//!
//! fn main() {
//!     let dev = I2cdev::new("/dev/i2c-1").unwrap();
//!     let mut sgp = Sgp40::new(dev, 0x6a, Delay);
//! }
//! ```
//! ### Doing Measurements
//!
//! The device is doing measurements independently of the driver and calls to the device
//! will just fetch the latest information making the usage easy.
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//! use hal::{Delay, I2cdev};
//!
//! use std::time::Duration;
//! use std::thread;
//!
//! use sgp40::Sgp40;
//!
//! fn main() {
//!     let dev = I2cdev::new("/dev/i2c-1").unwrap();
//!
//!     let mut sensor = Sgp40::new(dev, 0x59, Delay);
//!
//!     // Discard the first 45 samples as the algorithm is just warming up.
//!     for _ in 1..45 {
//!         sensor.measure_voc_index().unwrap();
//!     }
//!
//!     loop {
//!         if let Ok(result) = sensor.measure_voc_index() {
//!             println!("VOC index: {}", result);
//!         }
//!         else {
//!             println!("Failed I2C reading");
//!         }
//!
//!         thread::sleep(Duration::new(1_u64, 0));
//!     }
//! }
//! ```
#![cfg_attr(not(test), no_std)]
#![allow(non_snake_case)]
#![allow(dead_code)]

use embedded_hal as hal;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use sensirion_i2c::{crc8, i2c};

mod vocalg;

use crate::vocalg::VocAlgorithm;

/// Sgp40 errors
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
    /// Self test failed
    SelfTest
}

impl<E, I2cWrite, I2cRead> From<i2c::Error<I2cWrite, I2cRead>> for Error<E>
where
    I2cWrite: Write<Error = E>,
    I2cRead: Read<Error = E>,
{
    fn from(err: i2c::Error<I2cWrite, I2cRead>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum Command {
    /// Measures raw signal
    MeasurementRaw,
    /// Gets chips serial number
    Serial,
    /// Stops the measurement
    HeaterOff,
    /// Build-in self-test. This should be normally needed by any application
    MeasureTest,
    /// Get chipset featureset
    //FeatureSet,
    /// This is I²C wide command resetting all devices connected to the same bus
    SoftReset,
}

impl Command {
    /// Command and the requested delay in ms
    fn as_tuple(self) -> (u16, u32) {
        match self {
            Command::MeasurementRaw => (0x260f, 30),
            Command::Serial => (0x3682, 1),
            Command::HeaterOff => (0x3615, 1),
            Command::MeasureTest => (0x280e, 250),
            //Command::FeatureSet => (0x202f, 1),
            Command::SoftReset => (0x0006, 1),

        }
    }
}

/// Sgp40 driver instance
///
/// Create the driver instance with valid I²C address (0x59) and then it is just
/// rock'n'roll. This driver doesn't require special starting but once can start to
/// make measurements right away. However, the initial values after start-up will
/// unstable so you will want to throw away some of them.
pub struct Sgp40<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
    temperature_offset: i16,
    voc: VocAlgorithm,
}

impl<I2C, D, E> Sgp40<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u32>,
{
    /// Creates Sgp40 driver
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        Sgp40 {
            i2c,
            address,
            delay,
            temperature_offset: 0,
            voc : VocAlgorithm::new(),
        }
    }

    /// Command for reading values from the sensor
    fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd)?;
        i2c::read_words_with_crc(&mut self.i2c, self.address, data)?;
        Ok(())
    }

    /// Writes commands with arguments
    fn write_command_with_args(&mut self, cmd: Command, data: &[u8]) -> Result<(), Error<E>> {
        const MAX_TX_BUFFER: usize = 14; //cmd (2 bytes) + max args (12 bytes)

        let mut transfer_buffer = [0; MAX_TX_BUFFER];

        let size = data.len();

        // 2 for command, size of transferred bytes and CRC per each two bytes.
        assert!(size < 2 + size + size / 2);
        let (command, delay) = cmd.as_tuple();

        transfer_buffer[0..2].copy_from_slice(&command.to_be_bytes());

        let mut i = 2;
        for chunk in data.chunks(2) {
            let end = i+2;
            transfer_buffer[i..end].copy_from_slice(chunk);
            transfer_buffer[end] = crc8::calculate(chunk);
            i += 3;
        }

        self.i2c
            .write(self.address, &transfer_buffer[0..i])
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay);

        Ok(())
    }

    /// Writes commands without additional arguments.
    fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay) = cmd.as_tuple();
        i2c::write_command(&mut self.i2c, self.address, command).map_err(Error::I2c)?;
        self.delay.delay_ms(delay);
        Ok(())
    }

    /// Sensor self-test.
    ///
    /// Performs sensor self-test. This is intended for production line and testing and verification only and
    /// shouldn't be needed for normal use.
    pub fn self_test(&mut self) -> Result<&mut Self, Error<E>> {
        const MEASURE_TEST_OK: u16 = 0xd400;
        let mut data = [0; 3];

        self.delayed_read_cmd(Command::MeasureTest, &mut data)?;

        let result = u16::from_be_bytes([data[0], data[1]]);

        if result != MEASURE_TEST_OK {
            Err(Error::SelfTest)
        } else {
            Ok(self)
        }
    }


    /// Turn sensor heater off and places it in idle-mode.
    ///
    /// Stops running the measurements, places heater into idle by turning the heaters off.
    #[inline]
    pub fn turn_heater_off(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::HeaterOff)?;
        Ok(self)
    }

    /// Resets the sensor.
    ///
    /// Executes a reset on the device. The caller must wait 100ms before starting to use the device again.
    #[inline]
    pub fn reset(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::SoftReset)?;
        Ok(self)
    }

    /// Reads the voc index from the sensor.
    ///
    /// Reads VOC index. Driver is using Sensirion proprietary algortihm and it takes minimum
    /// 45 reads to start working. These reads should be made with 1Hz interval to keep the
    /// algoritm working.
    #[inline]
    pub fn measure_voc_index(&mut self) -> Result<u16, Error<E>> {
        let raw = self.measure_raw_with_rht(50000, 25000)?;

        Ok(self.voc.process(raw as i32) as u16)
    }

    /// Reads the voc index from the sensor with humidity and temperature compensation.
    ///
    /// Reads VOC index with humidity and temperature compensation. Both values us milli-notation where
    /// 25°C is equivalent of 25000 and 50% humidity equals 50000.
    ///
    /// Driver is using Sensirion proprietary algortihm and it takes minimum
    /// 45 reads to start working. These reads should be made with 1Hz interval to keep the
    /// algoritm working.
    #[inline]
    pub fn measure_voc_index_with_rht(&mut self, humidity: u16, temperature: i16) -> Result<u16, Error<E>> {
        let raw = self.measure_raw_with_rht(humidity, temperature)?;

        Ok(self.voc.process(raw as i32) as u16)
    }


    /// Reads the raw signal from the sensor.
    ///
    /// Raw signal without temperature and humidity compensation. This is not
    /// VOC index but needs to be processed through different algorithm for that.
    #[inline]
    pub fn measure_raw(&mut self) -> Result<u16, Error<E>> {
        self.measure_raw_with_rht(50000, 25000)
    }

    /// Reads the raw signal from the sensor.
    ///
    /// Raw signal with temperature and humidity compensation. This is not
    /// VOC index but needs to be processed through different algorithm for that.
    pub fn measure_raw_with_rht(&mut self, humidity: u16, temperature: i16) -> Result<u16, Error<E>> {
        let mut data = [0; 3];

        let (hum_ticks, temp_ticks) = self.convert_rht(humidity as u32, temperature as i32);

        let params = [hum_ticks.to_be_bytes(), temp_ticks.to_be_bytes()].concat();

        self.write_command_with_args(Command::MeasurementRaw, &params)?;
        i2c::read_words_with_crc(&mut self.i2c, self.address, &mut data)?;

        Ok(u16::from_be_bytes([data[0], data[1]]))
    }


    // Returns tick converted values
    fn convert_rht(&self, humidity: u32, temperature: i32) -> (u16, u16){
        let mut temperature = temperature;
        let mut humidity = humidity;
        if humidity > 100000 {
            humidity = 100000;
        }

        temperature += self.temperature_offset as i32;

        if temperature < -45000 {
            temperature = -45000;
        } else if temperature > 129760 {
            temperature = 129760;
        }

        /* humidity_sensor_format = humidity / 100000 * 65535;
            * 65535 / 100000 = 0.65535 -> 0.65535 * 2^5 = 20.9712 / 2^10 ~= 671
        */
        let humidity_sensor_format = ((humidity * 671) >> 10) as u16;

        /* temperature_sensor_format[1] = (temperature + 45000) / 175000 * 65535;
        * 65535 / 175000 ~= 0.375 -> 0.375 * 2^3 = 2.996 ~= 3
        */
        let temperature_sensor_format = (((temperature + 45000) * 3) >> 3) as u16;

        (humidity_sensor_format, temperature_sensor_format)
    }

    /// Sets the temperature offset.
    ///
    /// This command sets the temperature offset used for the compensation of subsequent RHT measurements.RawSignals
    /// The parameter provides the temperature offset (in °C) with a scaling factor of 200, e.g., an output of +400 corresponds to +2.00 °C.
    #[inline]
    pub fn set_temperature_offset(&mut self, offset: i16) -> Result<&mut Self, Error<E>> {
        self.temperature_offset += offset;
        Ok(self)
    }

    /// Gets the temperature offset
    ///
    /// Gets the temperature compensation offset issues to the device.
    /// TODO: For some reason, this functions results CRC error.
    pub fn get_temperature_offset(&mut self) -> Result<i16, Error<E>> {
        Ok(self.temperature_offset)
    }

    /// Acquires the sensor serial number.
    ///
    /// Sensor serial number is only 48-bits long so the remaining 16-bits are zeros.
    pub fn serial(&mut self) -> Result<u64, Error<E>> {
        let mut serial = [0; 9];

        self.delayed_read_cmd(Command::Serial, &mut serial)?;

        let serial = u64::from(serial[0]) << 40
            | u64::from(serial[1]) << 32
            | u64::from(serial[3]) << 24
            | u64::from(serial[4]) << 16
            | u64::from(serial[6]) << 8
            | u64::from(serial[7]);
        Ok(serial)
    }
}

// Testing is focused on checking the primitive transactions. It is assumed that during
// the real sensor testing, the basic flows in the command structure has been caught.
#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    const SGP40_ADDR:u8 = 0x59;

    /// Tests that the commands without parameters work
    #[test]
    fn test_basic_command() {
        let (cmd, _) = Command::MeasurementRaw.as_tuple();
        let expectations = [
            Transaction::write(SGP40_ADDR, [cmd.to_be_bytes().to_vec(), [0x7f, 0xfb, 0x4b, 0x66, 0x8a, 0x2f].to_vec()].concat()),
            Transaction::read(SGP40_ADDR, vec![0x12, 0x34, 0x37]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgp40::new(mock, SGP40_ADDR, DelayMock);
        let result = sensor.measure_raw().unwrap();
        assert_eq!(result, 0x1234);
    }

    /// Test the `serial` function
    #[test]
    fn serial() {
        let (cmd, _) = Command::Serial.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(
                0x58,
                vec![0xde, 0xad, 0x98, 0xbe, 0xef, 0x92, 0xde, 0xad, 0x98],
            ),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgp40::new(mock, 0x58, DelayMock);
        let serial = sensor.serial().unwrap();
        assert_eq!(serial, 0x00deadbeefdead);
    }

    #[test]
    fn test_crc_error() {
        let (cmd, _) = Command::MeasureTest.as_tuple();
        let expectations = [
            Transaction::write(SGP40_ADDR, cmd.to_be_bytes().to_vec()),
            Transaction::read(SGP40_ADDR, vec![0xD4, 0x00, 0x00]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgp40::new(mock, SGP40_ADDR, DelayMock);

        match sensor.self_test() {
            Err(Error::Crc) => {},
            Err(_) => panic!("Unexpected error in CRC test"),
            Ok(_) => panic!("Unexpected success in CRC test")
        }
    }
}
