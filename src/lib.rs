//! A driver for the register interface on the Nuvoton NAU88C22 CODEC
//!
//! Works in I²C mode only, for now.
//!
//! ```rust,no_run
//! # use embedded_hal::i2c::{self as hali2c, SevenBitAddress, I2c, Operation, ErrorKind};
//! # pub struct I2c0;
//! # #[derive(Debug, Copy, Clone, Eq, PartialEq)]
//! # pub enum Error { }
//! # impl hali2c::Error for Error {
//! #     fn kind(&self) -> hali2c::ErrorKind {
//! #         ErrorKind::Other
//! #     }
//! # }
//! # impl hali2c::ErrorType for I2c0 {
//! #     type Error = Error;
//! # }
//! # impl I2c<SevenBitAddress> for I2c0 {
//! #     fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
//! #       Ok(())
//! #     }
//! # }
//! # struct Uart0;
//! # impl core::fmt::Write for Uart0 {
//! #   fn write_str(&mut self, _string: &str) -> core::fmt::Result { Ok(()) }
//! # }
//! # struct Hal;
//! # impl Hal {
//! #   fn i2c(&self) -> I2c0 {
//! #     I2c0
//! #   }
//! #   fn uart(&self) -> Uart0 {
//! #     Uart0
//! #   }
//! #   fn delay_ms(&self, _ms: u32) {}
//! # }
//! # fn main() -> Result<(), nau88c22::Error<Error>> {
//! # let hal = Hal;
//! use core::fmt::Write;
//! let mut uart = hal.uart();
//! let i2c = hal.i2c();
//! let mut codec = nau88c22::Codec::new(i2c);
//! // Do a Software Reset on the chip to put registers into a known state. This
//! // fails if we don't get an I2C ACK:
//! codec.reset()?;
//! // You can then either poll a register for a known value, or just wait for
//! // the reset sequence to complete:
//! hal.delay_ms(100);
//! // First you should check the Device ID is correct:
//! codec.check_device_id()?;
//! // Every register has a `read_xxx()` method:
//! let pm1 = codec.read_powermanagement1()?;
//! // You can view the fields with a debug print:
//! writeln!(uart, "powermanagement1 = {:?}", pm1).unwrap();
//! // Or access them individually:
//! writeln!(uart, "powermanagement1.dcbufen = {}", pm1.dcbufen()).unwrap();
//! // You can also modify registers with a closure:
//! codec.modify_powermanagement1(|mut w| {
//!     // the closure is given a proxy object, usually called `w`
//!     // use it to turn the fields on or off
//!     w.iobufen_set(true);
//!     w.dcbufen_set(true);
//!     // you must return the proxy object from the closure
//!     w
//! })?;
//! # Ok(())
//! # }
//! ```

// SPDX-FileCopyrightText: 2023 Jonathan 'theJPster' Pallant <github@thejpster.org.uk>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![deny(missing_docs)]

pub mod registers;

use embedded_hal::i2c::{I2c, SevenBitAddress};

#[doc(inline)]
pub use registers::Register;

/// Represents the NAU882CC CODEC
///
/// All methods change the CODEC settings in real-time.
///
/// In general, for every register *Foo* there is:
///
/// * A type `struct Foo`, with methods to read and write the fields within it
/// * A method `fn read_foo(&mut self) -> Result<Foo, Error>`
/// * A method `fn write_foo(&mut self, value: Foo) -> Result<(), Error>`
/// * A method `fn modify_foo<F>(&mut self, f: F) -> Result<(), Error> where F: FnOnce(Foo) -> Foo`
#[derive(Debug, Clone)]
pub struct Codec<I> {
    interface: I,
}

/// Represents the ways that this library can fail
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// An I2C Error occurred
    I2c(E),
    /// The wrong Device ID was returned
    WrongDeviceId,
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Error::I2c(value)
    }
}

impl<I> Codec<I>
where
    I: I2c,
{
    /// Our I2C device address
    const DEVICE_ADDR: SevenBitAddress = 0b0011010;
    /// Expected in the Device ID register
    const DEVICE_ID: u16 = 0x1A;

    /// Create a new CODEC object.
    ///
    /// Holds on to the given I²C interface so it can perform I²C transactions
    /// whenever its methods are called.
    pub const fn new(interface: I) -> Codec<I> {
        Codec { interface }
    }

    /// Read the Device ID register as a check we actually have a CODEC
    pub fn check_device_id(&mut self) -> Result<(), Error<I::Error>> {
        let device_id = self.read_register(Register::DeviceId)?;
        #[cfg(feature = "defmt")]
        defmt::info!("Device ID = 0x{:03x}", device_id);
        if device_id == Self::DEVICE_ID {
            Ok(())
        } else {
            Err(Error::WrongDeviceId)
        }
    }

    /// Reset the chip
    pub fn reset(&mut self) -> Result<(), Error<I::Error>> {
        // write anything to this register to reset it
        self.write_register(Register::SoftwareReset, 0x1FF)
    }

    /// Read the *Power Management 1 register* contents
    pub fn read_powermanagement1(
        &mut self,
    ) -> Result<registers::PowerManagement1, Error<I::Error>> {
        let value = self.read_register(Register::PowerManagement1)?;
        Ok(registers::PowerManagement1(value))
    }

    /// Write the *Power Management 1 register* contents
    pub fn write_powermanagement1(
        &mut self,
        value: registers::PowerManagement1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PowerManagement1, value.0)
    }

    /// Modify the *Power Management 1 register* contents
    pub fn modify_powermanagement1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PowerManagement1) -> registers::PowerManagement1,
    {
        let value = self.read_powermanagement1()?;
        let new_value = f(value);
        self.write_powermanagement1(new_value)
    }

    /// Read the *Power Management 2 register* contents
    pub fn read_powermanagement2(
        &mut self,
    ) -> Result<registers::PowerManagement2, Error<I::Error>> {
        let value = self.read_register(Register::PowerManagement2)?;
        Ok(registers::PowerManagement2(value))
    }

    /// Write the *Power Management 2 register* contents
    pub fn write_powermanagement2(
        &mut self,
        value: registers::PowerManagement2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PowerManagement2, value.0)
    }

    /// Modify the *Power Management 2 register* contents
    pub fn modify_powermanagement2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PowerManagement2) -> registers::PowerManagement2,
    {
        let value = self.read_powermanagement2()?;
        let new_value = f(value);
        self.write_powermanagement2(new_value)
    }

    /// Read the *Power Management 3 register* contents
    pub fn read_powermanagement3(
        &mut self,
    ) -> Result<registers::PowerManagement3, Error<I::Error>> {
        let value = self.read_register(Register::PowerManagement3)?;
        Ok(registers::PowerManagement3(value))
    }

    /// Write the *Power Management 3 register* contents
    pub fn write_powermanagement3(
        &mut self,
        value: registers::PowerManagement3,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PowerManagement3, value.0)
    }

    /// Modify the *Power Management 3 register* contents
    pub fn modify_powermanagement3<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PowerManagement3) -> registers::PowerManagement3,
    {
        let value = self.read_powermanagement3()?;
        let new_value = f(value);
        self.write_powermanagement3(new_value)
    }

    /// Read the *Audio Interface register* contents
    pub fn read_audiointerface(&mut self) -> Result<registers::AudioInterface, Error<I::Error>> {
        let value = self.read_register(Register::AudioInterface)?;
        Ok(registers::AudioInterface(value))
    }

    /// Write the *Audio Interface register* contents
    pub fn write_audiointerface(
        &mut self,
        value: registers::AudioInterface,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::AudioInterface, value.0)
    }

    /// Modify the *Audio Interface register* contents
    pub fn modify_audiointerface<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::AudioInterface) -> registers::AudioInterface,
    {
        let value = self.read_audiointerface()?;
        let new_value = f(value);
        self.write_audiointerface(new_value)
    }

    /// Read the *Companding register* contents
    pub fn read_companding(&mut self) -> Result<registers::Companding, Error<I::Error>> {
        let value = self.read_register(Register::Companding)?;
        Ok(registers::Companding(value))
    }

    /// Write the *Companding register* contents
    pub fn write_companding(
        &mut self,
        value: registers::Companding,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::Companding, value.0)
    }

    /// Modify the *Companding register* contents
    pub fn modify_companding<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::Companding) -> registers::Companding,
    {
        let value = self.read_companding()?;
        let new_value = f(value);
        self.write_companding(new_value)
    }

    /// Read the *Clock Control 1 register* contents
    pub fn read_clockcontrol1(&mut self) -> Result<registers::ClockControl1, Error<I::Error>> {
        let value = self.read_register(Register::ClockControl1)?;
        Ok(registers::ClockControl1(value))
    }

    /// Write the *Clock Control 1 register* contents
    pub fn write_clockcontrol1(
        &mut self,
        value: registers::ClockControl1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ClockControl1, value.0)
    }

    /// Modify the *Clock Control 1 register* contents
    pub fn modify_clockcontrol1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ClockControl1) -> registers::ClockControl1,
    {
        let value = self.read_clockcontrol1()?;
        let new_value = f(value);
        self.write_clockcontrol1(new_value)
    }

    /// Read the *Clock Control 2 register* contents
    pub fn read_clockcontrol2(&mut self) -> Result<registers::ClockControl2, Error<I::Error>> {
        let value = self.read_register(Register::ClockControl2)?;
        Ok(registers::ClockControl2(value))
    }

    /// Write the *Clock Control 2 register* contents
    pub fn write_clockcontrol2(
        &mut self,
        value: registers::ClockControl2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ClockControl2, value.0)
    }

    /// Modify the *Clock Control 2 register* contents
    pub fn modify_clockcontrol2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ClockControl2) -> registers::ClockControl2,
    {
        let value = self.read_clockcontrol2()?;
        let new_value = f(value);
        self.write_clockcontrol2(new_value)
    }

    /// Read the *GPIO register* contents
    pub fn read_gpio(&mut self) -> Result<registers::GPIO, Error<I::Error>> {
        let value = self.read_register(Register::GPIO)?;
        Ok(registers::GPIO(value))
    }

    /// Write the *GPIO register* contents
    pub fn write_gpio(&mut self, value: registers::GPIO) -> Result<(), Error<I::Error>> {
        self.write_register(Register::GPIO, value.0)
    }

    /// Modify the *GPIO register* contents
    pub fn modify_gpio<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::GPIO) -> registers::GPIO,
    {
        let value = self.read_gpio()?;
        let new_value = f(value);
        self.write_gpio(new_value)
    }

    /// Read the *Jack Detect 1 register* contents
    pub fn read_jackdetect1(&mut self) -> Result<registers::JackDetect1, Error<I::Error>> {
        let value = self.read_register(Register::JackDetect1)?;
        Ok(registers::JackDetect1(value))
    }

    /// Write the *Jack Detect 1 register* contents
    pub fn write_jackdetect1(
        &mut self,
        value: registers::JackDetect1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::JackDetect1, value.0)
    }

    /// Modify the *Jack Detect 1 register* contents
    pub fn modify_jackdetect1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::JackDetect1) -> registers::JackDetect1,
    {
        let value = self.read_jackdetect1()?;
        let new_value = f(value);
        self.write_jackdetect1(new_value)
    }

    /// Read the *DAC Control register* contents
    pub fn read_daccontrol(&mut self) -> Result<registers::DACControl, Error<I::Error>> {
        let value = self.read_register(Register::DACControl)?;
        Ok(registers::DACControl(value))
    }

    /// Write the *DAC Control register* contents
    pub fn write_daccontrol(
        &mut self,
        value: registers::DACControl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::DACControl, value.0)
    }

    /// Modify the *DAC Control register* contents
    pub fn modify_daccontrol<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::DACControl) -> registers::DACControl,
    {
        let value = self.read_daccontrol()?;
        let new_value = f(value);
        self.write_daccontrol(new_value)
    }

    /// Read the *Left DAC Volume register* contents
    pub fn read_leftdacvolume(&mut self) -> Result<registers::LeftDACVolume, Error<I::Error>> {
        let value = self.read_register(Register::LeftDACVolume)?;
        Ok(registers::LeftDACVolume(value))
    }

    /// Write the *Left DAC Volume register* contents
    pub fn write_leftdacvolume(
        &mut self,
        value: registers::LeftDACVolume,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LeftDACVolume, value.0)
    }

    /// Modify the *Left DAC Volume register* contents
    pub fn modify_leftdacvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LeftDACVolume) -> registers::LeftDACVolume,
    {
        let value = self.read_leftdacvolume()?;
        let new_value = f(value);
        self.write_leftdacvolume(new_value)
    }

    /// Read the *Right DAC Volume register* contents
    pub fn read_rightdacvolume(&mut self) -> Result<registers::RightDACVolume, Error<I::Error>> {
        let value = self.read_register(Register::RightDACVolume)?;
        Ok(registers::RightDACVolume(value))
    }

    /// Write the *Right DAC Volume register* contents
    pub fn write_rightdacvolume(
        &mut self,
        value: registers::RightDACVolume,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightDACVolume, value.0)
    }

    /// Modify the *Right DAC Volume register* contents
    pub fn modify_rightdacvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightDACVolume) -> registers::RightDACVolume,
    {
        let value = self.read_rightdacvolume()?;
        let new_value = f(value);
        self.write_rightdacvolume(new_value)
    }

    /// Read the *Jack Detect 2 register* contents
    pub fn read_jackdetect2(&mut self) -> Result<registers::JackDetect2, Error<I::Error>> {
        let value = self.read_register(Register::JackDetect2)?;
        Ok(registers::JackDetect2(value))
    }

    /// Write the *Jack Detect 2 register* contents
    pub fn write_jackdetect2(
        &mut self,
        value: registers::JackDetect2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::JackDetect2, value.0)
    }

    /// Modify the *Jack Detect 2 register* contents
    pub fn modify_jackdetect2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::JackDetect2) -> registers::JackDetect2,
    {
        let value = self.read_jackdetect2()?;
        let new_value = f(value);
        self.write_jackdetect2(new_value)
    }

    /// Read the *ADC Control register* contents
    pub fn read_adccontrol(&mut self) -> Result<registers::ADCControl, Error<I::Error>> {
        let value = self.read_register(Register::ADCControl)?;
        Ok(registers::ADCControl(value))
    }

    /// Write the *ADC Control register* contents
    pub fn write_adccontrol(
        &mut self,
        value: registers::ADCControl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ADCControl, value.0)
    }

    /// Modify the *ADC Control register* contents
    pub fn modify_adccontrol<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ADCControl) -> registers::ADCControl,
    {
        let value = self.read_adccontrol()?;
        let new_value = f(value);
        self.write_adccontrol(new_value)
    }

    /// Read the *Left ADC Volume register* contents
    pub fn read_leftadcvolume(&mut self) -> Result<registers::LeftADCVolume, Error<I::Error>> {
        let value = self.read_register(Register::LeftADCVolume)?;
        Ok(registers::LeftADCVolume(value))
    }

    /// Write the *Left ADC Volume register* contents
    pub fn write_leftadcvolume(
        &mut self,
        value: registers::LeftADCVolume,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LeftADCVolume, value.0)
    }

    /// Modify the *Left ADC Volume register* contents
    pub fn modify_leftadcvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LeftADCVolume) -> registers::LeftADCVolume,
    {
        let value = self.read_leftadcvolume()?;
        let new_value = f(value);
        self.write_leftadcvolume(new_value)
    }

    /// Read the *Right ADC Volume register* contents
    pub fn read_rightadcvolume(&mut self) -> Result<registers::RightADCVolume, Error<I::Error>> {
        let value = self.read_register(Register::RightADCVolume)?;
        Ok(registers::RightADCVolume(value))
    }

    /// Write the *Right ADC Volume register* contents
    pub fn write_rightadcvolume(
        &mut self,
        value: registers::RightADCVolume,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightADCVolume, value.0)
    }

    /// Modify the *Right ADC Volume register* contents
    pub fn modify_rightadcvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightADCVolume) -> registers::RightADCVolume,
    {
        let value = self.read_rightadcvolume()?;
        let new_value = f(value);
        self.write_rightadcvolume(new_value)
    }

    /// Read the *EQ1-high cutoff register* contents
    pub fn read_eq1highcutoff(&mut self) -> Result<registers::EQ1HighCutoff, Error<I::Error>> {
        let value = self.read_register(Register::EQ1HighCutoff)?;
        Ok(registers::EQ1HighCutoff(value))
    }

    /// Write the *EQ1-high cutoff register* contents
    pub fn write_eq1highcutoff(
        &mut self,
        value: registers::EQ1HighCutoff,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::EQ1HighCutoff, value.0)
    }

    /// Modify the *EQ1-high cutoff register* contents
    pub fn modify_eq1highcutoff<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::EQ1HighCutoff) -> registers::EQ1HighCutoff,
    {
        let value = self.read_eq1highcutoff()?;
        let new_value = f(value);
        self.write_eq1highcutoff(new_value)
    }

    /// Read the *EQ2-peak 1 register* contents
    pub fn read_eq2peak1(&mut self) -> Result<registers::EQ2Peak1, Error<I::Error>> {
        let value = self.read_register(Register::EQ2Peak1)?;
        Ok(registers::EQ2Peak1(value))
    }

    /// Write the *EQ2-peak 1 register* contents
    pub fn write_eq2peak1(&mut self, value: registers::EQ2Peak1) -> Result<(), Error<I::Error>> {
        self.write_register(Register::EQ2Peak1, value.0)
    }

    /// Modify the *EQ2-peak 1 register* contents
    pub fn modify_eq2peak1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::EQ2Peak1) -> registers::EQ2Peak1,
    {
        let value = self.read_eq2peak1()?;
        let new_value = f(value);
        self.write_eq2peak1(new_value)
    }

    /// Read the *EQ3-peak 2 register* contents
    pub fn read_eq3peak2(&mut self) -> Result<registers::EQ3Peak2, Error<I::Error>> {
        let value = self.read_register(Register::EQ3Peak2)?;
        Ok(registers::EQ3Peak2(value))
    }

    /// Write the *EQ3-peak 2 register* contents
    pub fn write_eq3peak2(&mut self, value: registers::EQ3Peak2) -> Result<(), Error<I::Error>> {
        self.write_register(Register::EQ3Peak2, value.0)
    }

    /// Modify the *EQ3-peak 2 register* contents
    pub fn modify_eq3peak2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::EQ3Peak2) -> registers::EQ3Peak2,
    {
        let value = self.read_eq3peak2()?;
        let new_value = f(value);
        self.write_eq3peak2(new_value)
    }

    /// Read the *EQ4-peak 3 register* contents
    pub fn read_eq4peak3(&mut self) -> Result<registers::EQ4Peak3, Error<I::Error>> {
        let value = self.read_register(Register::EQ4Peak3)?;
        Ok(registers::EQ4Peak3(value))
    }

    /// Write the *EQ4-peak 3 register* contents
    pub fn write_eq4peak3(&mut self, value: registers::EQ4Peak3) -> Result<(), Error<I::Error>> {
        self.write_register(Register::EQ4Peak3, value.0)
    }

    /// Modify the *EQ4-peak 3 register* contents
    pub fn modify_eq4peak3<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::EQ4Peak3) -> registers::EQ4Peak3,
    {
        let value = self.read_eq4peak3()?;
        let new_value = f(value);
        self.write_eq4peak3(new_value)
    }

    /// Read the *EQ5-low cutoff register* contents
    pub fn read_eq5lowcutoff(&mut self) -> Result<registers::EQ5LowCutoff, Error<I::Error>> {
        let value = self.read_register(Register::EQ5LowCutoff)?;
        Ok(registers::EQ5LowCutoff(value))
    }

    /// Write the *EQ5-low cutoff register* contents
    pub fn write_eq5lowcutoff(
        &mut self,
        value: registers::EQ5LowCutoff,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::EQ5LowCutoff, value.0)
    }

    /// Modify the *EQ5-low cutoff register* contents
    pub fn modify_eq5lowcutoff<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::EQ5LowCutoff) -> registers::EQ5LowCutoff,
    {
        let value = self.read_eq5lowcutoff()?;
        let new_value = f(value);
        self.write_eq5lowcutoff(new_value)
    }

    /// Read the *DAC Limiter 1 register* contents
    pub fn read_daclimiter1(&mut self) -> Result<registers::DACLimiter1, Error<I::Error>> {
        let value = self.read_register(Register::DACLimiter1)?;
        Ok(registers::DACLimiter1(value))
    }

    /// Write the *DAC Limiter 1 register* contents
    pub fn write_daclimiter1(
        &mut self,
        value: registers::DACLimiter1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::DACLimiter1, value.0)
    }

    /// Modify the *DAC Limiter 1 register* contents
    pub fn modify_daclimiter1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::DACLimiter1) -> registers::DACLimiter1,
    {
        let value = self.read_daclimiter1()?;
        let new_value = f(value);
        self.write_daclimiter1(new_value)
    }

    /// Read the *DAC Limiter 2 register* contents
    pub fn read_daclimiter2(&mut self) -> Result<registers::DACLimiter2, Error<I::Error>> {
        let value = self.read_register(Register::DACLimiter2)?;
        Ok(registers::DACLimiter2(value))
    }

    /// Write the *DAC Limiter 2 register* contents
    pub fn write_daclimiter2(
        &mut self,
        value: registers::DACLimiter2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::DACLimiter2, value.0)
    }

    /// Modify the *DAC Limiter 2 register* contents
    pub fn modify_daclimiter2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::DACLimiter2) -> registers::DACLimiter2,
    {
        let value = self.read_daclimiter2()?;
        let new_value = f(value);
        self.write_daclimiter2(new_value)
    }

    /// Read the *Notch Filter 1 register* contents
    pub fn read_notchfilter1(&mut self) -> Result<registers::NotchFilter1, Error<I::Error>> {
        let value = self.read_register(Register::NotchFilter1)?;
        Ok(registers::NotchFilter1(value))
    }

    /// Write the *Notch Filter 1 register* contents
    pub fn write_notchfilter1(
        &mut self,
        value: registers::NotchFilter1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::NotchFilter1, value.0)
    }

    /// Modify the *Notch Filter 1 register* contents
    pub fn modify_notchfilter1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::NotchFilter1) -> registers::NotchFilter1,
    {
        let value = self.read_notchfilter1()?;
        let new_value = f(value);
        self.write_notchfilter1(new_value)
    }

    /// Read the *Notch Filter 2 register* contents
    pub fn read_notchfilter2(&mut self) -> Result<registers::NotchFilter2, Error<I::Error>> {
        let value = self.read_register(Register::NotchFilter2)?;
        Ok(registers::NotchFilter2(value))
    }

    /// Write the *Notch Filter 2 register* contents
    pub fn write_notchfilter2(
        &mut self,
        value: registers::NotchFilter2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::NotchFilter2, value.0)
    }

    /// Modify the *Notch Filter 2 register* contents
    pub fn modify_notchfilter2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::NotchFilter2) -> registers::NotchFilter2,
    {
        let value = self.read_notchfilter2()?;
        let new_value = f(value);
        self.write_notchfilter2(new_value)
    }

    /// Read the *Notch Filter 3 register* contents
    pub fn read_notchfilter3(&mut self) -> Result<registers::NotchFilter3, Error<I::Error>> {
        let value = self.read_register(Register::NotchFilter3)?;
        Ok(registers::NotchFilter3(value))
    }

    /// Write the *Notch Filter 3 register* contents
    pub fn write_notchfilter3(
        &mut self,
        value: registers::NotchFilter3,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::NotchFilter3, value.0)
    }

    /// Modify the *Notch Filter 3 register* contents
    pub fn modify_notchfilter3<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::NotchFilter3) -> registers::NotchFilter3,
    {
        let value = self.read_notchfilter3()?;
        let new_value = f(value);
        self.write_notchfilter3(new_value)
    }

    /// Read the *Notch Filter 4 register* contents
    pub fn read_notchfilter4(&mut self) -> Result<registers::NotchFilter4, Error<I::Error>> {
        let value = self.read_register(Register::NotchFilter4)?;
        Ok(registers::NotchFilter4(value))
    }

    /// Write the *Notch Filter 4 register* contents
    pub fn write_notchfilter4(
        &mut self,
        value: registers::NotchFilter4,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::NotchFilter4, value.0)
    }

    /// Modify the *Notch Filter 4 register* contents
    pub fn modify_notchfilter4<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::NotchFilter4) -> registers::NotchFilter4,
    {
        let value = self.read_notchfilter4()?;
        let new_value = f(value);
        self.write_notchfilter4(new_value)
    }

    /// Read the *ALC Control 1 register* contents
    pub fn read_alccontrol1(&mut self) -> Result<registers::ALCControl1, Error<I::Error>> {
        let value = self.read_register(Register::ALCControl1)?;
        Ok(registers::ALCControl1(value))
    }

    /// Write the *ALC Control 1 register* contents
    pub fn write_alccontrol1(
        &mut self,
        value: registers::ALCControl1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ALCControl1, value.0)
    }

    /// Modify the *ALC Control 1 register* contents
    pub fn modify_alccontrol1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ALCControl1) -> registers::ALCControl1,
    {
        let value = self.read_alccontrol1()?;
        let new_value = f(value);
        self.write_alccontrol1(new_value)
    }

    /// Read the *ALC Control 2 register* contents
    pub fn read_alccontrol2(&mut self) -> Result<registers::ALCControl2, Error<I::Error>> {
        let value = self.read_register(Register::ALCControl2)?;
        Ok(registers::ALCControl2(value))
    }

    /// Write the *ALC Control 2 register* contents
    pub fn write_alccontrol2(
        &mut self,
        value: registers::ALCControl2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ALCControl2, value.0)
    }

    /// Modify the *ALC Control 2 register* contents
    pub fn modify_alccontrol2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ALCControl2) -> registers::ALCControl2,
    {
        let value = self.read_alccontrol2()?;
        let new_value = f(value);
        self.write_alccontrol2(new_value)
    }

    /// Read the *ALC Control 3 register* contents
    pub fn read_alccontrol3(&mut self) -> Result<registers::ALCControl3, Error<I::Error>> {
        let value = self.read_register(Register::ALCControl3)?;
        Ok(registers::ALCControl3(value))
    }

    /// Write the *ALC Control 3 register* contents
    pub fn write_alccontrol3(
        &mut self,
        value: registers::ALCControl3,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ALCControl3, value.0)
    }

    /// Modify the *ALC Control 3 register* contents
    pub fn modify_alccontrol3<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ALCControl3) -> registers::ALCControl3,
    {
        let value = self.read_alccontrol3()?;
        let new_value = f(value);
        self.write_alccontrol3(new_value)
    }

    /// Read the *Noise Gate register* contents
    pub fn read_noisegate(&mut self) -> Result<registers::NoiseGate, Error<I::Error>> {
        let value = self.read_register(Register::NoiseGate)?;
        Ok(registers::NoiseGate(value))
    }

    /// Write the *Noise Gate register* contents
    pub fn write_noisegate(&mut self, value: registers::NoiseGate) -> Result<(), Error<I::Error>> {
        self.write_register(Register::NoiseGate, value.0)
    }

    /// Modify the *Noise Gate register* contents
    pub fn modify_noisegate<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::NoiseGate) -> registers::NoiseGate,
    {
        let value = self.read_noisegate()?;
        let new_value = f(value);
        self.write_noisegate(new_value)
    }

    /// Read the *PLL N register* contents
    pub fn read_plln(&mut self) -> Result<registers::PllN, Error<I::Error>> {
        let value = self.read_register(Register::PllN)?;
        Ok(registers::PllN(value))
    }

    /// Write the *PLL N register* contents
    pub fn write_plln(&mut self, value: registers::PllN) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PllN, value.0)
    }

    /// Modify the *PLL N register* contents
    pub fn modify_plln<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PllN) -> registers::PllN,
    {
        let value = self.read_plln()?;
        let new_value = f(value);
        self.write_plln(new_value)
    }

    /// Read the *PLL K 1 register* contents
    pub fn read_pllk1(&mut self) -> Result<registers::PllK1, Error<I::Error>> {
        let value = self.read_register(Register::PllK1)?;
        Ok(registers::PllK1(value))
    }

    /// Write the *PLL K 1 register* contents
    pub fn write_pllk1(&mut self, value: registers::PllK1) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PllK1, value.0)
    }

    /// Modify the *PLL K 1 register* contents
    pub fn modify_pllk1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PllK1) -> registers::PllK1,
    {
        let value = self.read_pllk1()?;
        let new_value = f(value);
        self.write_pllk1(new_value)
    }

    /// Read the *PLL K 2 register* contents
    pub fn read_pllk2(&mut self) -> Result<registers::PllK2, Error<I::Error>> {
        let value = self.read_register(Register::PllK2)?;
        Ok(registers::PllK2(value))
    }

    /// Write the *PLL K 2 register* contents
    pub fn write_pllk2(&mut self, value: registers::PllK2) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PllK2, value.0)
    }

    /// Modify the *PLL K 2 register* contents
    pub fn modify_pllk2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PllK2) -> registers::PllK2,
    {
        let value = self.read_pllk2()?;
        let new_value = f(value);
        self.write_pllk2(new_value)
    }

    /// Read the *PLL K 3 register* contents
    pub fn read_pllk3(&mut self) -> Result<registers::PllK3, Error<I::Error>> {
        let value = self.read_register(Register::PllK3)?;
        Ok(registers::PllK3(value))
    }

    /// Write the *PLL K 3 register* contents
    pub fn write_pllk3(&mut self, value: registers::PllK3) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PllK3, value.0)
    }

    /// Modify the *PLL K 3 register* contents
    pub fn modify_pllk3<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PllK3) -> registers::PllK3,
    {
        let value = self.read_pllk3()?;
        let new_value = f(value);
        self.write_pllk3(new_value)
    }

    /// Read the *3D control register* contents
    pub fn read_threedcontrol(&mut self) -> Result<registers::ThreeDControl, Error<I::Error>> {
        let value = self.read_register(Register::ThreeDControl)?;
        Ok(registers::ThreeDControl(value))
    }

    /// Write the *3D control register* contents
    pub fn write_threedcontrol(
        &mut self,
        value: registers::ThreeDControl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ThreeDControl, value.0)
    }

    /// Modify the *3D control register* contents
    pub fn modify_threedcontrol<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ThreeDControl) -> registers::ThreeDControl,
    {
        let value = self.read_threedcontrol()?;
        let new_value = f(value);
        self.write_threedcontrol(new_value)
    }

    /// Read the *Right Speaker Submix register* contents
    pub fn read_rightspeakersubmix(
        &mut self,
    ) -> Result<registers::RightSpeakerSubmix, Error<I::Error>> {
        let value = self.read_register(Register::RightSpeakerSubmix)?;
        Ok(registers::RightSpeakerSubmix(value))
    }

    /// Write the *Right Speaker Submix register* contents
    pub fn write_rightspeakersubmix(
        &mut self,
        value: registers::RightSpeakerSubmix,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightSpeakerSubmix, value.0)
    }

    /// Modify the *Right Speaker Submix register* contents
    pub fn modify_rightspeakersubmix<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightSpeakerSubmix) -> registers::RightSpeakerSubmix,
    {
        let value = self.read_rightspeakersubmix()?;
        let new_value = f(value);
        self.write_rightspeakersubmix(new_value)
    }

    /// Read the *Input Control register* contents
    pub fn read_inputcontrol(&mut self) -> Result<registers::InputControl, Error<I::Error>> {
        let value = self.read_register(Register::InputControl)?;
        Ok(registers::InputControl(value))
    }

    /// Write the *Input Control register* contents
    pub fn write_inputcontrol(
        &mut self,
        value: registers::InputControl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::InputControl, value.0)
    }

    /// Modify the *Input Control register* contents
    pub fn modify_inputcontrol<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::InputControl) -> registers::InputControl,
    {
        let value = self.read_inputcontrol()?;
        let new_value = f(value);
        self.write_inputcontrol(new_value)
    }

    /// Read the *Left Input PGA Gain register* contents
    pub fn read_leftinputpgagain(
        &mut self,
    ) -> Result<registers::LeftInputPGAGain, Error<I::Error>> {
        let value = self.read_register(Register::LeftInputPGAGain)?;
        Ok(registers::LeftInputPGAGain(value))
    }

    /// Write the *Left Input PGA Gain register* contents
    pub fn write_leftinputpgagain(
        &mut self,
        value: registers::LeftInputPGAGain,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LeftInputPGAGain, value.0)
    }

    /// Modify the *Left Input PGA Gain register* contents
    pub fn modify_leftinputpgagain<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LeftInputPGAGain) -> registers::LeftInputPGAGain,
    {
        let value = self.read_leftinputpgagain()?;
        let new_value = f(value);
        self.write_leftinputpgagain(new_value)
    }

    /// Read the *Right Input PGA Gain register* contents
    pub fn read_rightinputpgagain(
        &mut self,
    ) -> Result<registers::RightInputPGAGain, Error<I::Error>> {
        let value = self.read_register(Register::RightInputPGAGain)?;
        Ok(registers::RightInputPGAGain(value))
    }

    /// Write the *Right Input PGA Gain register* contents
    pub fn write_rightinputpgagain(
        &mut self,
        value: registers::RightInputPGAGain,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightInputPGAGain, value.0)
    }

    /// Modify the *Right Input PGA Gain register* contents
    pub fn modify_rightinputpgagain<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightInputPGAGain) -> registers::RightInputPGAGain,
    {
        let value = self.read_rightinputpgagain()?;
        let new_value = f(value);
        self.write_rightinputpgagain(new_value)
    }

    /// Read the *Left ADC Boost register* contents
    pub fn read_leftadcboost(&mut self) -> Result<registers::LeftADCBoost, Error<I::Error>> {
        let value = self.read_register(Register::LeftADCBoost)?;
        Ok(registers::LeftADCBoost(value))
    }

    /// Write the *Left ADC Boost register* contents
    pub fn write_leftadcboost(
        &mut self,
        value: registers::LeftADCBoost,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LeftADCBoost, value.0)
    }

    /// Modify the *Left ADC Boost register* contents
    pub fn modify_leftadcboost<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LeftADCBoost) -> registers::LeftADCBoost,
    {
        let value = self.read_leftadcboost()?;
        let new_value = f(value);
        self.write_leftadcboost(new_value)
    }

    /// Read the *Right ADC Boost register* contents
    pub fn read_rightadcboost(&mut self) -> Result<registers::RightADCBoost, Error<I::Error>> {
        let value = self.read_register(Register::RightADCBoost)?;
        Ok(registers::RightADCBoost(value))
    }

    /// Write the *Right ADC Boost register* contents
    pub fn write_rightadcboost(
        &mut self,
        value: registers::RightADCBoost,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightADCBoost, value.0)
    }

    /// Modify the *Right ADC Boost register* contents
    pub fn modify_rightadcboost<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightADCBoost) -> registers::RightADCBoost,
    {
        let value = self.read_rightadcboost()?;
        let new_value = f(value);
        self.write_rightadcboost(new_value)
    }

    /// Read the *Output Control register* contents
    pub fn read_outputcontrol(&mut self) -> Result<registers::OutputControl, Error<I::Error>> {
        let value = self.read_register(Register::OutputControl)?;
        Ok(registers::OutputControl(value))
    }

    /// Write the *Output Control register* contents
    pub fn write_outputcontrol(
        &mut self,
        value: registers::OutputControl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::OutputControl, value.0)
    }

    /// Modify the *Output Control register* contents
    pub fn modify_outputcontrol<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::OutputControl) -> registers::OutputControl,
    {
        let value = self.read_outputcontrol()?;
        let new_value = f(value);
        self.write_outputcontrol(new_value)
    }

    /// Read the *Left Mixer register* contents
    pub fn read_leftmixer(&mut self) -> Result<registers::LeftMixer, Error<I::Error>> {
        let value = self.read_register(Register::LeftMixer)?;
        Ok(registers::LeftMixer(value))
    }

    /// Write the *Left Mixer register* contents
    pub fn write_leftmixer(&mut self, value: registers::LeftMixer) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LeftMixer, value.0)
    }

    /// Modify the *Left Mixer register* contents
    pub fn modify_leftmixer<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LeftMixer) -> registers::LeftMixer,
    {
        let value = self.read_leftmixer()?;
        let new_value = f(value);
        self.write_leftmixer(new_value)
    }

    /// Read the *Right Mixer register* contents
    pub fn read_rightmixer(&mut self) -> Result<registers::RightMixer, Error<I::Error>> {
        let value = self.read_register(Register::RightMixer)?;
        Ok(registers::RightMixer(value))
    }

    /// Write the *Right Mixer register* contents
    pub fn write_rightmixer(
        &mut self,
        value: registers::RightMixer,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightMixer, value.0)
    }

    /// Modify the *Right Mixer register* contents
    pub fn modify_rightmixer<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightMixer) -> registers::RightMixer,
    {
        let value = self.read_rightmixer()?;
        let new_value = f(value);
        self.write_rightmixer(new_value)
    }

    /// Read the *LHP Volume register* contents
    pub fn read_lhpvolume(&mut self) -> Result<registers::LHPVolume, Error<I::Error>> {
        let value = self.read_register(Register::LHPVolume)?;
        Ok(registers::LHPVolume(value))
    }

    /// Write the *LHP Volume register* contents
    pub fn write_lhpvolume(&mut self, value: registers::LHPVolume) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LHPVolume, value.0)
    }

    /// Modify the *LHP Volume register* contents
    pub fn modify_lhpvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LHPVolume) -> registers::LHPVolume,
    {
        let value = self.read_lhpvolume()?;
        let new_value = f(value);
        self.write_lhpvolume(new_value)
    }

    /// Read the *RHP Volume register* contents
    pub fn read_rhpvolume(&mut self) -> Result<registers::RHPVolume, Error<I::Error>> {
        let value = self.read_register(Register::RHPVolume)?;
        Ok(registers::RHPVolume(value))
    }

    /// Write the *RHP Volume register* contents
    pub fn write_rhpvolume(&mut self, value: registers::RHPVolume) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RHPVolume, value.0)
    }

    /// Modify the *RHP Volume register* contents
    pub fn modify_rhpvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RHPVolume) -> registers::RHPVolume,
    {
        let value = self.read_rhpvolume()?;
        let new_value = f(value);
        self.write_rhpvolume(new_value)
    }

    /// Read the *LSPKOUT Volume register* contents
    pub fn read_lspkoutvolume(&mut self) -> Result<registers::LSPKOUTVolume, Error<I::Error>> {
        let value = self.read_register(Register::LSPKOUTVolume)?;
        Ok(registers::LSPKOUTVolume(value))
    }

    /// Write the *LSPKOUT Volume register* contents
    pub fn write_lspkoutvolume(
        &mut self,
        value: registers::LSPKOUTVolume,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LSPKOUTVolume, value.0)
    }

    /// Modify the *LSPKOUT Volume register* contents
    pub fn modify_lspkoutvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LSPKOUTVolume) -> registers::LSPKOUTVolume,
    {
        let value = self.read_lspkoutvolume()?;
        let new_value = f(value);
        self.write_lspkoutvolume(new_value)
    }

    /// Read the *RSPKOUT Volume register* contents
    pub fn read_rspkoutvolume(&mut self) -> Result<registers::RSPKOUTVolume, Error<I::Error>> {
        let value = self.read_register(Register::RSPKOUTVolume)?;
        Ok(registers::RSPKOUTVolume(value))
    }

    /// Write the *RSPKOUT Volume register* contents
    pub fn write_rspkoutvolume(
        &mut self,
        value: registers::RSPKOUTVolume,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RSPKOUTVolume, value.0)
    }

    /// Modify the *RSPKOUT Volume register* contents
    pub fn modify_rspkoutvolume<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RSPKOUTVolume) -> registers::RSPKOUTVolume,
    {
        let value = self.read_rspkoutvolume()?;
        let new_value = f(value);
        self.write_rspkoutvolume(new_value)
    }

    /// Read the *AUX2 Mixer register* contents
    pub fn read_aux2mixer(&mut self) -> Result<registers::AUX2Mixer, Error<I::Error>> {
        let value = self.read_register(Register::AUX2Mixer)?;
        Ok(registers::AUX2Mixer(value))
    }

    /// Write the *AUX2 Mixer register* contents
    pub fn write_aux2mixer(&mut self, value: registers::AUX2Mixer) -> Result<(), Error<I::Error>> {
        self.write_register(Register::AUX2Mixer, value.0)
    }

    /// Modify the *AUX2 Mixer register* contents
    pub fn modify_aux2mixer<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::AUX2Mixer) -> registers::AUX2Mixer,
    {
        let value = self.read_aux2mixer()?;
        let new_value = f(value);
        self.write_aux2mixer(new_value)
    }

    /// Read the *AUX1 Mixer register* contents
    pub fn read_aux1mixer(&mut self) -> Result<registers::AUX1Mixer, Error<I::Error>> {
        let value = self.read_register(Register::AUX1Mixer)?;
        Ok(registers::AUX1Mixer(value))
    }

    /// Write the *AUX1 Mixer register* contents
    pub fn write_aux1mixer(&mut self, value: registers::AUX1Mixer) -> Result<(), Error<I::Error>> {
        self.write_register(Register::AUX1Mixer, value.0)
    }

    /// Modify the *AUX1 Mixer register* contents
    pub fn modify_aux1mixer<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::AUX1Mixer) -> registers::AUX1Mixer,
    {
        let value = self.read_aux1mixer()?;
        let new_value = f(value);
        self.write_aux1mixer(new_value)
    }

    /// Read the *Power Management register* contents
    pub fn read_powermanagement(&mut self) -> Result<registers::PowerManagement, Error<I::Error>> {
        let value = self.read_register(Register::PowerManagement)?;
        Ok(registers::PowerManagement(value))
    }

    /// Write the *Power Management register* contents
    pub fn write_powermanagement(
        &mut self,
        value: registers::PowerManagement,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PowerManagement, value.0)
    }

    /// Modify the *Power Management register* contents
    pub fn modify_powermanagement<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PowerManagement) -> registers::PowerManagement,
    {
        let value = self.read_powermanagement()?;
        let new_value = f(value);
        self.write_powermanagement(new_value)
    }

    /// Read the *Left Time Slot register* contents
    pub fn read_lefttimeslot(&mut self) -> Result<registers::LeftTimeSlot, Error<I::Error>> {
        let value = self.read_register(Register::LeftTimeSlot)?;
        Ok(registers::LeftTimeSlot(value))
    }

    /// Write the *Left Time Slot register* contents
    pub fn write_lefttimeslot(
        &mut self,
        value: registers::LeftTimeSlot,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::LeftTimeSlot, value.0)
    }

    /// Modify the *Left Time Slot register* contents
    pub fn modify_lefttimeslot<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::LeftTimeSlot) -> registers::LeftTimeSlot,
    {
        let value = self.read_lefttimeslot()?;
        let new_value = f(value);
        self.write_lefttimeslot(new_value)
    }

    /// Read the *Misc register* contents
    pub fn read_misc(&mut self) -> Result<registers::Misc, Error<I::Error>> {
        let value = self.read_register(Register::Misc)?;
        Ok(registers::Misc(value))
    }

    /// Write the *Misc register* contents
    pub fn write_misc(&mut self, value: registers::Misc) -> Result<(), Error<I::Error>> {
        self.write_register(Register::Misc, value.0)
    }

    /// Modify the *Misc register* contents
    pub fn modify_misc<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::Misc) -> registers::Misc,
    {
        let value = self.read_misc()?;
        let new_value = f(value);
        self.write_misc(new_value)
    }

    /// Read the *Right Time Slot register* contents
    pub fn read_righttimeslot(&mut self) -> Result<registers::RightTimeSlot, Error<I::Error>> {
        let value = self.read_register(Register::RightTimeSlot)?;
        Ok(registers::RightTimeSlot(value))
    }

    /// Write the *Right Time Slot register* contents
    pub fn write_righttimeslot(
        &mut self,
        value: registers::RightTimeSlot,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::RightTimeSlot, value.0)
    }

    /// Modify the *Right Time Slot register* contents
    pub fn modify_righttimeslot<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::RightTimeSlot) -> registers::RightTimeSlot,
    {
        let value = self.read_righttimeslot()?;
        let new_value = f(value);
        self.write_righttimeslot(new_value)
    }

    /// Read the *Device Revision # register* contents
    pub fn read_devicerevisionno(
        &mut self,
    ) -> Result<registers::DeviceRevisionNo, Error<I::Error>> {
        let value = self.read_register(Register::DeviceRevisionNo)?;
        Ok(registers::DeviceRevisionNo(value))
    }

    /// Write the *Device Revision # register* contents
    pub fn write_devicerevisionno(
        &mut self,
        value: registers::DeviceRevisionNo,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::DeviceRevisionNo, value.0)
    }

    /// Modify the *Device Revision # register* contents
    pub fn modify_devicerevisionno<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::DeviceRevisionNo) -> registers::DeviceRevisionNo,
    {
        let value = self.read_devicerevisionno()?;
        let new_value = f(value);
        self.write_devicerevisionno(new_value)
    }

    /// Read the *Device ID register* contents
    pub fn read_deviceid(&mut self) -> Result<registers::DeviceId, Error<I::Error>> {
        let value = self.read_register(Register::DeviceId)?;
        Ok(registers::DeviceId(value))
    }

    /// Write the *Device ID register* contents
    pub fn write_deviceid(&mut self, value: registers::DeviceId) -> Result<(), Error<I::Error>> {
        self.write_register(Register::DeviceId, value.0)
    }

    /// Modify the *Device ID register* contents
    pub fn modify_deviceid<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::DeviceId) -> registers::DeviceId,
    {
        let value = self.read_deviceid()?;
        let new_value = f(value);
        self.write_deviceid(new_value)
    }

    /// Read the *DAC Dither register* contents
    pub fn read_dacdither(&mut self) -> Result<registers::DacDither, Error<I::Error>> {
        let value = self.read_register(Register::DacDither)?;
        Ok(registers::DacDither(value))
    }

    /// Write the *DAC Dither register* contents
    pub fn write_dacdither(&mut self, value: registers::DacDither) -> Result<(), Error<I::Error>> {
        self.write_register(Register::DacDither, value.0)
    }

    /// Modify the *DAC Dither register* contents
    pub fn modify_dacdither<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::DacDither) -> registers::DacDither,
    {
        let value = self.read_dacdither()?;
        let new_value = f(value);
        self.write_dacdither(new_value)
    }

    /// Read the *ALC Enhancements 1 register* contents
    pub fn read_alcenhancements1(
        &mut self,
    ) -> Result<registers::AlcEnhancements1, Error<I::Error>> {
        let value = self.read_register(Register::AlcEnhancements1)?;
        Ok(registers::AlcEnhancements1(value))
    }

    /// Write the *ALC Enhancements 1 register* contents
    pub fn write_alcenhancements1(
        &mut self,
        value: registers::AlcEnhancements1,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::AlcEnhancements1, value.0)
    }

    /// Modify the *ALC Enhancements 1 register* contents
    pub fn modify_alcenhancements1<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::AlcEnhancements1) -> registers::AlcEnhancements1,
    {
        let value = self.read_alcenhancements1()?;
        let new_value = f(value);
        self.write_alcenhancements1(new_value)
    }

    /// Read the *ALC Enhancements 2 register* contents
    pub fn read_alcenhancements2(
        &mut self,
    ) -> Result<registers::AlcEnhancements2, Error<I::Error>> {
        let value = self.read_register(Register::AlcEnhancements2)?;
        Ok(registers::AlcEnhancements2(value))
    }

    /// Write the *ALC Enhancements 2 register* contents
    pub fn write_alcenhancements2(
        &mut self,
        value: registers::AlcEnhancements2,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::AlcEnhancements2, value.0)
    }

    /// Modify the *ALC Enhancements 2 register* contents
    pub fn modify_alcenhancements2<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::AlcEnhancements2) -> registers::AlcEnhancements2,
    {
        let value = self.read_alcenhancements2()?;
        let new_value = f(value);
        self.write_alcenhancements2(new_value)
    }

    /// Read the *Misc Controls register* contents
    pub fn read_misccontrols(&mut self) -> Result<registers::MiscControls, Error<I::Error>> {
        let value = self.read_register(Register::MiscControls)?;
        Ok(registers::MiscControls(value))
    }

    /// Write the *Misc Controls register* contents
    pub fn write_misccontrols(
        &mut self,
        value: registers::MiscControls,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::MiscControls, value.0)
    }

    /// Modify the *Misc Controls register* contents
    pub fn modify_misccontrols<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::MiscControls) -> registers::MiscControls,
    {
        let value = self.read_misccontrols()?;
        let new_value = f(value);
        self.write_misccontrols(new_value)
    }

    /// Read the *Tie-Off Overrides register* contents
    pub fn read_tieoffoverrides(&mut self) -> Result<registers::TieOffOverrides, Error<I::Error>> {
        let value = self.read_register(Register::TieOffOverrides)?;
        Ok(registers::TieOffOverrides(value))
    }

    /// Write the *Tie-Off Overrides register* contents
    pub fn write_tieoffoverrides(
        &mut self,
        value: registers::TieOffOverrides,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::TieOffOverrides, value.0)
    }

    /// Modify the *Tie-Off Overrides register* contents
    pub fn modify_tieoffoverrides<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::TieOffOverrides) -> registers::TieOffOverrides,
    {
        let value = self.read_tieoffoverrides()?;
        let new_value = f(value);
        self.write_tieoffoverrides(new_value)
    }

    /// Read the *Power/Tie-off Ctrl register* contents
    pub fn read_powertieoffctrl(&mut self) -> Result<registers::PowerTieOffCtrl, Error<I::Error>> {
        let value = self.read_register(Register::PowerTieOffCtrl)?;
        Ok(registers::PowerTieOffCtrl(value))
    }

    /// Write the *Power/Tie-off Ctrl register* contents
    pub fn write_powertieoffctrl(
        &mut self,
        value: registers::PowerTieOffCtrl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PowerTieOffCtrl, value.0)
    }

    /// Modify the *Power/Tie-off Ctrl register* contents
    pub fn modify_powertieoffctrl<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PowerTieOffCtrl) -> registers::PowerTieOffCtrl,
    {
        let value = self.read_powertieoffctrl()?;
        let new_value = f(value);
        self.write_powertieoffctrl(new_value)
    }

    /// Read the *P2P Detector Read register* contents
    pub fn read_p2pdetectorread(&mut self) -> Result<registers::P2PDetectorRead, Error<I::Error>> {
        let value = self.read_register(Register::P2PDetectorRead)?;
        Ok(registers::P2PDetectorRead(value))
    }

    /// Write the *P2P Detector Read register* contents
    pub fn write_p2pdetectorread(
        &mut self,
        value: registers::P2PDetectorRead,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::P2PDetectorRead, value.0)
    }

    /// Modify the *P2P Detector Read register* contents
    pub fn modify_p2pdetectorread<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::P2PDetectorRead) -> registers::P2PDetectorRead,
    {
        let value = self.read_p2pdetectorread()?;
        let new_value = f(value);
        self.write_p2pdetectorread(new_value)
    }

    /// Read the *Peak Detector Read register* contents
    pub fn read_peakdetectorread(
        &mut self,
    ) -> Result<registers::PeakDetectorRead, Error<I::Error>> {
        let value = self.read_register(Register::PeakDetectorRead)?;
        Ok(registers::PeakDetectorRead(value))
    }

    /// Write the *Peak Detector Read register* contents
    pub fn write_peakdetectorread(
        &mut self,
        value: registers::PeakDetectorRead,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::PeakDetectorRead, value.0)
    }

    /// Modify the *Peak Detector Read register* contents
    pub fn modify_peakdetectorread<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::PeakDetectorRead) -> registers::PeakDetectorRead,
    {
        let value = self.read_peakdetectorread()?;
        let new_value = f(value);
        self.write_peakdetectorread(new_value)
    }

    /// Read the *Control and Status register* contents
    pub fn read_controlandstatus(
        &mut self,
    ) -> Result<registers::ControlAndStatus, Error<I::Error>> {
        let value = self.read_register(Register::ControlAndStatus)?;
        Ok(registers::ControlAndStatus(value))
    }

    /// Write the *Control and Status register* contents
    pub fn write_controlandstatus(
        &mut self,
        value: registers::ControlAndStatus,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::ControlAndStatus, value.0)
    }

    /// Modify the *Control and Status register* contents
    pub fn modify_controlandstatus<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::ControlAndStatus) -> registers::ControlAndStatus,
    {
        let value = self.read_controlandstatus()?;
        let new_value = f(value);
        self.write_controlandstatus(new_value)
    }

    /// Read the *Output tie-off control register* contents
    pub fn read_outputtieoffcontrol(
        &mut self,
    ) -> Result<registers::OutputTieOffControl, Error<I::Error>> {
        let value = self.read_register(Register::OutputTieOffControl)?;
        Ok(registers::OutputTieOffControl(value))
    }

    /// Write the *Output tie-off control register* contents
    pub fn write_outputtieoffcontrol(
        &mut self,
        value: registers::OutputTieOffControl,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(Register::OutputTieOffControl, value.0)
    }

    /// Modify the *Output tie-off control register* contents
    pub fn modify_outputtieoffcontrol<F>(&mut self, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(registers::OutputTieOffControl) -> registers::OutputTieOffControl,
    {
        let value = self.read_outputtieoffcontrol()?;
        let new_value = f(value);
        self.write_outputtieoffcontrol(new_value)
    }

    /// Read a nine-bit register from the chip.
    ///
    /// The value comes back in the lowest 9 bits of a `u16`.
    ///
    /// ```
    /// # use nau88c22::{Codec, Register, Error};
    /// # fn example<I>(codec: &mut Codec<I>) -> Result<(), Error<I::Error>> where I: embedded_hal::i2c::I2c {
    /// let device_id = codec.read_register(Register::DeviceId)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn read_register(&mut self, register: Register) -> Result<u16, Error<I::Error>> {
        let mut buffer = [0u8; 2];
        self.interface
            .write_read(Self::DEVICE_ADDR, &[(register as u8) * 2], &mut buffer)?;
        let mut result = ((buffer[0] as u16) & 1) << 8;
        result |= buffer[1] as u16;
        Ok(result)
    }

    /// Write a nine-bit register to the chip.
    ///
    /// The value should be given in the lowest 9 bits of a `u16`.
    ///
    /// ```
    /// # use nau88c22::{Codec, Register, Error};
    /// # fn example<I>(codec: &mut Codec<I>) -> Result<(), Error<I::Error>> where I: embedded_hal::i2c::I2c {
    /// codec.write_register(Register::ThreeDControl, 0x5)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn write_register(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<I::Error>> {
        let buffer = [((register as u8) * 2) | (value >> 8) as u8, value as u8];
        self.interface.write(Self::DEVICE_ADDR, &buffer)?;
        Ok(())
    }

    /// Modify a nine-bit register on the chip.
    ///
    /// Performs a register read, then runs the given closure `f`, then performs
    /// a register write.
    ///
    /// ```
    /// # use nau88c22::{Codec, Register, Error};
    /// # fn example<I>(codec: &mut Codec<I>) -> Result<(), Error<I::Error>> where I: embedded_hal::i2c::I2c {
    /// codec.modify_register(Register::AudioInterface, |w| {
    ///     w | 0x01
    /// })?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn modify_register<F>(&mut self, register: Register, f: F) -> Result<(), Error<I::Error>>
    where
        F: FnOnce(u16) -> u16,
    {
        let value = self.read_register(register)?;
        let new_value = f(value);
        self.write_register(register, new_value)
    }
}

// End of file
