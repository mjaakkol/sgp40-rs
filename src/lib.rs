//! Platform agnostic Rust driver for Sensirion SVM40 device with
//! gas, temperature and humidity sensors based on
//! the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## Sensirion SVM40
//!
//! Sensirion SGPC3 is a low-power accurate gas sensor for air quality application.
//! The sensor has different sampling rates to optimize power-consumption per application
//! bases as well as ability save and set the baseline for faster start-up accuracy.
//! The sensor uses I²C interface and measures TVOC (*Total Volatile Organic Compounds*)
//!
//! Evaluation board: https://www.sensirion.com/cn/environmental-sensors/evaluation-kit-sek-svm40/
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
//! use svm40::Svm40;
//!
//! fn main() {
//!     let dev = I2cdev::new("/dev/i2c-1").unwrap();
//!     let mut sgp = Svm40::new(dev, 0x6a, Delay);
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
//! use svm40::Svm40;
//!
//! fn main() {
//!     let dev = I2cdev::new("/dev/i2c-1").unwrap();
//!
//!     let mut sensor = Svm40::new(dev, 0x6A, Delay);
//!
//!     let version = sensor.version().unwrap();
//!
//!     println!("Version information {:?}", version);
//!
//!     let mut serial = [0; 26];
//!
//!     sensor.serial(&mut serial).unwrap();
//!
//!     println!("Serial {:?}", serial);
//!
//!     sensor.start_measurement().unwrap();
//!
//!     thread::sleep(Duration::new(2_u64, 0));
//!
//!     for _ in 1..20 {
//!         let signals = sensor.get_measurements().unwrap();
//!         println!("Measurements: {:?}", signals);
//!         thread::sleep(Duration::new(1_u64, 0));
//!
//!         let signals = sensor.get_raw_measurements().unwrap();
//!         println!("Measurements: {:?}", signals);
//!         thread::sleep(Duration::new(1_u64, 0));
//!     }
//!     sensor.stop_measurement().unwrap();
//! }
//! ```
#![cfg_attr(not(test), no_std)]

use embedded_hal as hal;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use sensirion_i2c::{crc8, i2c};

/// Standard signal measurement
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Signals {
    /// VOC algorithm output with a scaling value of 10.
    voc_index: u16,
    /// Compensated ambient humidity in %RH with a scaling factor of 100.
    relative_humidity: u16,
    /// Compensated ambient temperature in degree celsius with a scaling factor of 200.
    temperature: u16,
}

impl Signals {
    fn parse(data: &[u8]) -> Self {
        Signals {
            voc_index: u16::from_be_bytes([data[0], data[1]]),
            relative_humidity: u16::from_be_bytes([data[6], data[7]]),
            temperature: u16::from_be_bytes([data[3], data[4]]),
        }
    }
}

/// Raw signal measurement. Raw signals include the standard signals.
#[derive(Debug, Copy, Clone)]
pub struct RawSignals {
    standard: Signals,
    /// Raw VOC output ticks as read from the SGP sensor.
    voc_ticks_raw: u16,
    /// Uncompensated raw humidity in %RH as read from the SHT40 with a scaling factor of 100.
    uncompensated_relative_humidity: u16,
    /// Uncompensated raw temperature in degrees celsius as read from the SHT40 with a scaling of 200.
    uncompensated_temperature: u16,
}

impl RawSignals {
    fn parse(data: &[u8]) -> Self {
        let standard = Signals::parse(&data[0..9]);

        RawSignals {
            standard,
            voc_ticks_raw: u16::from_be_bytes([data[9], data[10]]),
            uncompensated_relative_humidity: u16::from_be_bytes([data[15], data[16]]),
            uncompensated_temperature: u16::from_be_bytes([data[12], data[13]]),
        }
    }
}


/// Svm40 errors
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
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
    /// Starts the measurement
    StartMeasurement,
    /// Gets signals
    GetSignals,
    /// Gets raw signals
    GetRawSignals,
    /// Stops the measurement
    StopMeasurement,
    /// Gets temperature offset
    GetTemperatureOffset,
    /// Sets the temperature offset
    SetTemperatureOffset,
    /// Gets VOC parameters
    GetVocParameters,
    /// Sets VOC parameters
    SetVocParameters,
    /// Stores input parameters
    StoreInputParameters,
    /// Gets VOC states
    GetVocStates,
    /// Sets VOC states
    SetVocStates,
    /// Gets the sensor version information
    GetVersion,
    /// Resets the device
    Reset,
    // TODO: Add get serial - supported in the code but not in spec.
    Serial
}

impl Command {
    /// Command and the requested delay in ms
    fn as_tuple(self) -> (u16, u32) {
        match self {
            Command::StartMeasurement => (0x0010, 1),
            Command::GetSignals => (0x03a6, 1),
            Command::GetRawSignals => (0x03b0, 1),
            Command::StopMeasurement => (0x0104, 1),
            Command::GetTemperatureOffset => (0x6014, 1),
            Command::SetTemperatureOffset => (0x6014, 1),
            Command::GetVocParameters => (0x6083, 1),
            Command::SetVocParameters => (0x6083, 1),
            Command::StoreInputParameters => (0x6002, 1),
            Command::GetVocStates => (0x6181, 1),
            Command::SetVocStates => (0x6181, 1),
            Command::GetVersion => (0xd100, 1),
            Command::Reset => (0xd304, 1),
            Command::Serial => (0xd033, 1)
        }
    }
}

#[derive(Debug)]
pub struct Version {
    sw_major_ver: u8,
    sw_minor_ver: u8,
    debug_state: bool,
    hw_major_ver: u8,
    hw_minor_ver: u8,
    protocol_major_ver: u8,
    protocol_minor_ver: u8,
}

#[derive(Debug, Default)]
pub struct Svm40<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
}

impl<I2C, D, E> Svm40<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u32>,
{
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        Svm40 {
            i2c,
            address,
            delay,
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
        const MAX_TX_BUFFER: usize = 8;

        let mut transfer_buffer = [0; MAX_TX_BUFFER];

        let size = data.len();

        // 2 for command, size of transferred bytes and CRC per each two bytes.
        assert!(size < 2 + size + size / 2);
        let (command, delay) = cmd.as_tuple();

        transfer_buffer[0..2].copy_from_slice(&command.to_be_bytes());
        let slice = &data[..2];
        transfer_buffer[2..4].copy_from_slice(slice);
        transfer_buffer[4] = crc8::calculate(slice);

        let transfer_buffer = if size > 2 {
            let slice = &data[2..4];
            transfer_buffer[5..7].copy_from_slice(slice);
            transfer_buffer[7] = crc8::calculate(slice);
            &transfer_buffer[..]
        } else {
            &transfer_buffer[0..5]
        };

        self.i2c
            .write(self.address, transfer_buffer)
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

    /// Starts measurement
    ///
    /// The device starts measuring continuously providing new sample every 1s. If the user gets the signals earlier,
    /// the same values are returned.
    #[inline]
    pub fn start_measurement(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::StartMeasurement)?;
        Ok(self)
    }

    /// Stops measurement
    ///
    /// Stops running the measurements. The user will need to wait 50ms until the device is ready again.
    #[inline]
    pub fn stop_measurement(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::StopMeasurement)?;
        Ok(self)
    }

    /// Resets the sensor.
    ///
    /// Executes a reset on the device. The caller must wait 100ms before starting to use the device again.
    #[inline]
    pub fn reset(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::Reset)?;
        Ok(self)
    }

    /// Acquires the sensor serial number.
    ///
    /// Sensor serial number is only 48-bits long so the remaining 16-bits are zeros.
    pub fn version(&mut self) -> Result<Version, Error<E>> {
        let mut version = [0; 12];

        self.delayed_read_cmd(Command::GetVersion, &mut version)?;

        Ok(Version {
            sw_major_ver: version[0],
            sw_minor_ver: version[1],
            debug_state: version[3] != 0,
            hw_major_ver: version[4],
            hw_minor_ver: version[6],
            protocol_major_ver: version[7],
            protocol_minor_ver: version[9],
        })
    }


    /// Read the current measured values.
    ///
    /// The firmware updates the measurement values every second. Polling data
    /// faster will return the same values. The first measurement is available one
    /// second after the start measurement command is issued. Any readout prior to
    /// this will return zero initialized values.
    ///
    /// This method can only be used after calling ['start_measurement'].
    pub fn get_measurements(&mut self) -> Result<Signals, Error<E>> {
        let mut data = [0; 9];

        self.delayed_read_cmd(Command::GetSignals, &mut data)?;
        Ok(Signals::parse(&data))
    }

    /// Returns the new measurement results as integers along with the raw voc ticks and uncompensated RH/T values.
    ///
    /// This method reads out VOC Index, relative humidity, and temperature (like ['get_measurements']) and additionally
    /// the raw signal of SGP40 (proportional to the logarithm of the resistance of the MOX layer) as well as relative
    /// humidity and temperature which are not compensated for temperature offset. The firmware updates the measurement
    /// values every second. Polling data faster will return the same values. The first measurement is available on
    /// second after the start measurement command is issued. Any readout prior to this will return zero initialized values.
    ///
    /// This method can only be used after calling ['start_measurement'].
    pub fn get_raw_measurements(&mut self) -> Result<RawSignals, Error<E>> {
        let mut data = [0; 18];

        self.delayed_read_cmd(Command::GetRawSignals, &mut data)?;
        Ok(RawSignals::parse(&data))
    }

    /// Gets the temperature offset
    ///
    /// Gets the temperature compensation offset issues to the device.
    /// TODO: For some reason, this functions results CRC error.
    pub fn get_temperature_offset(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0; 3];
        self.delayed_read_cmd(Command::GetTemperatureOffset, &mut buffer)?;

        Ok(u16::from_be_bytes([buffer[0], buffer[1]]))
    }

    /// Sets the temperature offset.
    ///
    /// This command sets the temperature offset used for the compensation of subsequent RHT measurements.RawSignals
    /// The parameter provides the temperature offset (in °C) with a scaling factor of 200, e.g., an output of +400 corresponds to +2.00 °C.
    #[inline]
    pub fn set_temperature_offset(&mut self, offset: u16) -> Result<&mut Self, Error<E>> {
        self.write_command_with_args(Command::SetTemperatureOffset, &offset.to_be_bytes())?;
        Ok(self)
    }

    /// Gets the device serial number
    pub fn serial(&mut self, serial: &mut [u8;26]) -> Result<&Self, Error<E>> {
        let mut buffer = [0; 39];
        self.delayed_read_cmd(Command::Serial, &mut buffer)?;

        let mut i = 0;

        for chunk in buffer.chunks(3) {
            serial[i] = chunk[0];
            i += 1;
            serial[i] = chunk[1];
            i += 1;
        }
        Ok(self)
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

    const SVM40_ADD:u8 = 0x6a;

    /// Tests that the commands without parameters work
    #[test]
    fn test_basic_command() {
        let (cmd, _) = Command::StartMeasurement.as_tuple();
        let expectations = [ Transaction::write(SVM40_ADD, cmd.to_be_bytes().to_vec()) ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Svm40::new(mock, SVM40_ADD, DelayMock);
        sensor.start_measurement().unwrap();
    }

    /// Test the `serial` function
    #[test]
    fn test_basic_read() {
        let (cmd, _) = Command::GetTemperatureOffset.as_tuple();
        let expectations = [
            Transaction::write(SVM40_ADD, cmd.to_be_bytes().to_vec()),
            Transaction::read(SVM40_ADD, vec![0x00, 0x00, 0x81]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Svm40::new(mock, SVM40_ADD, DelayMock);
        let offset = sensor.get_temperature_offset().unwrap();
        assert_eq!(offset, 0);
    }


    #[test]
    fn test_crc_error() {
        let (cmd, _) = Command::GetTemperatureOffset.as_tuple();
        let expectations = [
            Transaction::write(SVM40_ADD, cmd.to_be_bytes().to_vec()),
            Transaction::read(SVM40_ADD, vec![0xD4, 0x00, 0x00]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Svm40::new(mock, SVM40_ADD, DelayMock);

        match sensor.self_test() {
            Err(Error::Crc) => {},
            Err(_) => panic!("Unexpected error in CRC test"),
            Ok(_) => panic!("Unexpected success in CRC test")
        }
    }

    #[test]
    fn test_version() {
        let (cmd, _) = Command::GetVersion.as_tuple();
        let expectations = [
            Transaction::write(SVM40_ADD, cmd.to_be_bytes().to_vec()),
            Transaction::read(SVM40_ADD, vec![0x01, 0x00, 0x75, 0x00, 0x01, 0xb0, 0x00, 0x01, 0xb0, 00, 00, 0x81]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Svm40::new(mock, SVM40_ADD, DelayMock);
        let version = sensor.version()().unwrap();
        assert_eq!(version.sw_major_ver, 1);
        assert_eq!(version.sw_minor_ver, 0);
        assert_eq!(version.debug_state, false);
        assert_eq!(version.hw_major_ver, 1);
        assert_eq!(version.hw_minor_ver, 0);
        assert_eq!(version.protocol_major_ver, 1);
        assert_eq!(version.protocol_minor_ver, 0);
    }
}
