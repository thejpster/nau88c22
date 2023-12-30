//! Register constants for the NAU88C22
//!
//! See <https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev1.8.pdf>

// SPDX-FileCopyrightText: 2023 Jonathan 'theJPster' Pallant <github@thejpster.org.uk>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use bitfield::bitfield;

/// The list of registers on the chip
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Register {
    /// Software Reset
    SoftwareReset = 0x00,
    /// Power Management 1
    PowerManagement1 = 0x01,
    /// Power Management 2
    PowerManagement2 = 0x02,
    /// Power Management 3
    PowerManagement3 = 0x03,
    /// Audio Interface
    AudioInterface = 0x04,
    /// Companding
    Companding = 0x05,
    /// Clock Control 1
    ClockControl1 = 0x06,
    /// Clock Control 2
    ClockControl2 = 0x07,
    /// GPIO
    GPIO = 0x08,
    /// Jack Detect 1
    JackDetect1 = 0x09,
    /// DAC Control
    DACControl = 0x0A,
    /// Left DAC Volume
    LeftDACVolume = 0x0B,
    /// Right DAC Volume
    RightDACVolume = 0x0C,
    /// Jack Detect 2
    JackDetect2 = 0x0D,
    /// ADC Control
    ADCControl = 0x0E,
    /// Left ADC Volume
    LeftADCVolume = 0x0F,
    /// Right ADC Volume
    RightADCVolume = 0x10,
    /// EQ1-high cutoff
    EQ1HighCutoff = 0x12,
    /// EQ2-peak 1
    EQ2Peak1 = 0x13,
    /// EQ3-peak 2
    EQ3Peak2 = 0x14,
    /// EQ4-peak 3
    EQ4Peak3 = 0x15,
    /// EQ5-low cutoff
    EQ5LowCutoff = 0x16,
    /// DAC Limiter 1
    DACLimiter1 = 0x18,
    /// DAC Limiter 2
    DACLimiter2 = 0x19,
    /// Notch Filter 1
    NotchFilter1 = 0x1B,
    /// Notch Filter 2
    NotchFilter2 = 0x1C,
    /// Notch Filter 3
    NotchFilter3 = 0x1D,
    /// Notch Filter 4
    NotchFilter4 = 0x1E,
    /// ALC Control 1
    ALCControl1 = 0x20,
    /// ALC Control 2
    ALCControl2 = 0x21,
    /// ALC Control 3
    ALCControl3 = 0x22,
    /// Noise Gate
    NoiseGate = 0x23,
    /// PLL N
    PllN = 0x24,
    /// PLL K 1
    PllK1 = 0x25,
    /// PLL K 2
    PllK2 = 0x26,
    /// PLL K 3
    PllK3 = 0x27,
    /// 3D control
    ThreeDControl = 0x29,
    /// Right Speaker Submix
    RightSpeakerSubmix = 0x2B,
    /// Input Control
    InputControl = 0x2C,
    /// Left Input PGA Gain
    LeftInputPGAGain = 0x2D,
    /// Right Input PGA Gain
    RightInputPGAGain = 0x2E,
    /// Left ADC Boost
    LeftADCBoost = 0x2F,
    /// Right ADC Boost
    RightADCBoost = 0x30,
    /// Output Control
    OutputControl = 0x31,
    /// Left Mixer
    LeftMixer = 0x32,
    /// Right Mixer
    RightMixer = 0x33,
    /// LHP Volume
    LHPVolume = 0x34,
    /// RHP Volume
    RHPVolume = 0x35,
    /// LSPKOUT Volume
    LSPKOUTVolume = 0x36,
    /// RSPKOUT Volume
    RSPKOUTVolume = 0x37,
    /// AUX2 Mixer
    AUX2Mixer = 0x38,
    /// AUX1 Mixer
    AUX1Mixer = 0x39,
    /// Power Management
    PowerManagement = 0x3A,
    /// Left Time Slot
    LeftTimeSlot = 0x3B,
    /// Misc
    Misc = 0x3C,
    /// Right Time Slot
    RightTimeSlot = 0x3D,

    /// Device Revision #
    DeviceRevisionNo = 0x3E,
    /// Device ID
    DeviceId = 0x3F,
    /// DAC Dither
    DacDither = 0x41,
    /// ALC Enhancements 1
    AlcEnhancements1 = 0x46,
    /// ALC Enhancements 2
    AlcEnhancements2 = 0x47,
    /// Misc Controls
    MiscControls = 0x49,
    /// Tie-Off Overrides
    TieOffOverrides = 0x4A,
    /// Power/Tie-off Ctrl
    PowerTieOffCtrl = 0x4B,
    /// P2P Detector Read
    P2PDetectorRead = 0x4C,
    /// Peak Detector Read
    PeakDetectorRead = 0x4D,
    /// Control and Status
    ControlAndStatus = 0x4E,
    /// Output tie-off control
    OutputTieOffControl = 0x4F,
}

impl Register {
    /// Get a static slice of all the readable registers
    pub fn readable() -> &'static [Register] {
        static REGISTERS: [Register; 67] = [
            Register::PowerManagement1,
            Register::PowerManagement2,
            Register::PowerManagement3,
            Register::AudioInterface,
            Register::Companding,
            Register::ClockControl1,
            Register::ClockControl2,
            Register::GPIO,
            Register::JackDetect1,
            Register::DACControl,
            Register::LeftDACVolume,
            Register::RightDACVolume,
            Register::JackDetect2,
            Register::ADCControl,
            Register::LeftADCVolume,
            Register::RightADCVolume,
            Register::EQ1HighCutoff,
            Register::EQ2Peak1,
            Register::EQ3Peak2,
            Register::EQ4Peak3,
            Register::EQ5LowCutoff,
            Register::DACLimiter1,
            Register::DACLimiter2,
            Register::NotchFilter1,
            Register::NotchFilter2,
            Register::NotchFilter3,
            Register::NotchFilter4,
            Register::ALCControl1,
            Register::ALCControl2,
            Register::ALCControl3,
            Register::NoiseGate,
            Register::PllN,
            Register::PllK1,
            Register::PllK2,
            Register::PllK3,
            Register::ThreeDControl,
            Register::RightSpeakerSubmix,
            Register::InputControl,
            Register::LeftInputPGAGain,
            Register::RightInputPGAGain,
            Register::LeftADCBoost,
            Register::RightADCBoost,
            Register::OutputControl,
            Register::LeftMixer,
            Register::RightMixer,
            Register::LHPVolume,
            Register::RHPVolume,
            Register::LSPKOUTVolume,
            Register::RSPKOUTVolume,
            Register::AUX2Mixer,
            Register::AUX1Mixer,
            Register::PowerManagement,
            Register::LeftTimeSlot,
            Register::Misc,
            Register::RightTimeSlot,
            Register::DeviceRevisionNo,
            Register::DeviceId,
            Register::DacDither,
            Register::AlcEnhancements1,
            Register::AlcEnhancements2,
            Register::MiscControls,
            Register::TieOffOverrides,
            Register::PowerTieOffCtrl,
            Register::P2PDetectorRead,
            Register::PeakDetectorRead,
            Register::ControlAndStatus,
            Register::OutputTieOffControl,
        ];

        &REGISTERS
    }
}

bitfield! {
    /// Power Management 1 register contents
    ///
    /// See [`read_powermanagement1`](crate::Codec::read_powermanagement1),
    /// [`write_powermanagement1`](crate::Codec::write_powermanagement1) and
    /// [`modify_powermanagement1`](crate::Codec::modify_powermanagement1)
    pub struct PowerManagement1(u16);
    impl Debug;
    u8;
    /// Power control for internal tie-off buffer used in 1.5X boost conditions
    ///
    /// * `false` - internal buffer unpowered
    /// * `true` - enabled
    pub dcbufen, dcbufen_set: 8;
    /// Power control for AUX1 MIXER supporting AUXOUT1 analog output
    ///
    /// * `false` - unpowered
    /// * `true` - enabled
    pub aux1mxen, aux1mxen_set: 7;
    /// Power control for AUX2 MIXER supporting AUXOUT2 analog output
    ///
    /// * `false` - unpowered
    /// * `true` - enabled
    pub aux2mxen, aux2mxen_set: 6;
    /// Power control for internal PLL
    ///
    /// * `false` - unpowered
    /// * `true` - enabled
    pub pllen, pllen_set: 5;
    /// Power control for microphone bias buffer amplifier (MICBIAS output,
    /// pin#32)
    ///
    /// * `false` - unpowered and MICBIAS pin in high-Z condition
    /// * `true` - enabled
    pub micbiasen, micbiasen_set: 4;
    /// Power control for internal analog bias buffers
    ///
    /// * `false` - unpowered
    /// * `true` - enabled
    pub abiasen, abiasen_set: 3;
    /// Power control for internal tie-off buffer used in non-boost mode (-1.0x
    /// gain) conditions
    ///
    /// * `false` - internal buffer unpowered
    /// * `true` - enabled
    pub iobufen, iobufen_set: 2;
    /// Select impedance of reference string used to establish VREF for internal
    /// bias buffers
    ///
    /// * `0` = off (input to internal bias buffer in high-Z floating condition)
    /// * `1` = 80 kΩ nominal impedance at VREF pin
    /// * `2` = 300 kΩ nominal impedance at VREF pin
    /// * `3` = 3 kΩ nominal impedance at VREF pin
    pub refimp, refimp_set: 1,0;
}

bitfield! {
    /// Power Management 2 register contents
    ///
    /// See [`read_powermanagement2`](crate::Codec::read_powermanagement2),
    /// [`write_powermanagement2`](crate::Codec::write_powermanagement2) and
    /// [`modify_powermanagement2`](crate::Codec::modify_powermanagement2)
    pub struct PowerManagement2(u16);
    impl Debug;
    u8;
    /// Right Headphone driver enable, RHP analog output, pin#29
    ///
    /// * `false` = RHP pin in high-Z condition
    /// * `true` = enabled
    pub rhpen, rhpen_set: 8;
    /// Left Headphone driver enabled, LHP analog output pin#30
    ///
    /// * `false` = LHP pin in high-Z condition
    /// * `true` = enabled
    pub lhpen, lhpen_set: 7;
    /// Sleep enable
    ///
    /// * `false` = device in normal operating mode
    /// * `true` = device in low-power sleep condition
    pub sleep, sleep_set: 6;
    /// Right channel input mixer, RADC Mix/Boost stage power control
    ///
    /// * `false` = RADC Mix/Boost stage OFF
    /// * `true` = RADC Mix/Boost stage ON
    pub rbsten, rbsten_set: 5;
    /// Left channel input mixer, LADC Mix/Boost stage power control
    ///
    /// * `false` = LADC Mix/Boost stage OFF
    /// * `true` = LADC Mix/Boost stage ON
    pub lbsten, lbsten_set: 4;
    /// Right channel input programmable amplifier (PGA) power control
    ///
    /// * `false` = Right PGA input stage OFF
    /// * `true` = enabled
    pub rpgaen, rpgaen_set: 3;
    /// Left channel input programmable amplifier power control
    ///
    /// * `false` = Left PGA input stage OFF
    /// * `true` = enabled
    pub lpgaen, lpgaen_set: 2;
    /// Right channel analog-to-digital converter power control
    ///
    /// * `false` = Right ADC stage OFF
    /// * `true` = enabled
    pub radcen, radcen_set: 1;
    /// Left channel analog-to-digital converter power control
    ///
    /// * `false` = Left ADC stage OFF
    /// * `true` = enable
    pub ladcen, ladcen_set: 0;
}

bitfield! {
    /// Power Management 3 register contents
    ///
    /// See [`read_powermanagement3`](crate::Codec::read_powermanagement3),
    /// [`write_powermanagement3`](crate::Codec::write_powermanagement3) and
    /// [`modify_powermanagement3`](crate::Codec::modify_powermanagement3)
    pub struct PowerManagement3(u16);
    impl Debug;
    u8;
    /// AUXOUT1 analog output power control, pin#21
    ///
    /// * `false` = AUXOUT1 output driver OFF
    /// * `true` = enabled
    pub auxout1en, auxout1en_set: 8;
    /// AUXOUT2 analog output power control, pin#22
    ///
    /// * `false` = AUXOUT2 output driver OFF
    /// * `true` = enabled
    pub auxout2en, auxout2en_set: 7;
    /// LSPKOUT left speaker driver power control, pin#25
    ///
    /// * `false` = LSPKOUT output driver OFF
    /// * `true` = enabled
    pub lspken, lspken_set: 6;
    /// RSPKOUT left speaker driver power control, pin#23
    ///
    /// * `false` = RSPKOUT output driver OFF
    /// * `true` = enabled
    pub rspken, rspken_set: 5;
    /// Right main mixer power control, RMAIN MIXER internal stage
    ///
    /// * `false` = RMAIN MIXER stage OFF
    /// * `true` = enabled
    pub rmixen, rmixen_set: 3;
    /// Left main mixer power control, LMAIN MIXER internal stage
    ///
    /// * `false` = LMAIN MIXER stage OFF
    /// * `true` = enabled
    pub lmixen, lmixen_set: 2;
    /// Right channel digital-to-analog converter, RDAC, power control
    ///
    /// * `false` = RDAC stage OFF
    /// * `true` = enabled
    pub rdacen, rdacen_set: 1;
    /// Left channel digital-to-analog converter, LDAC, power control
    ///
    /// * `false` = LDAC stage OFF
    /// * `true` = enabled
    pub ldacen, ldacen_set: 0;
}

bitfield! {
    /// Audio Interface register contents
    ///
    /// See [`read_audiointerface`](crate::Codec::read_audiointerface),
    /// [`write_audiointerface`](crate::Codec::write_audiointerface) and
    /// [`modify_audiointerface`](crate::Codec::modify_audiointerface)
    pub struct AudioInterface(u16);
    impl Debug;
    u8;
    /// Bit clock phase inversion option for BCLK, pin#8
    ///
    /// * `false` = normal phase
    /// * `true` = input logic sense inverted
    pub bclkp, bclkp_set: 8;
    /// Phase control for I2S audio data bus interface, or PCMA and PCMB
    /// left/right word order control
    ///
    /// * `false` = normal phase operation, or MSB is valid on 2nd rising edge
    /// of BCLK after rising edge of FS
    /// * `true` = inverted phase operation, or MSB is valid on 1st rising edge
    /// of BCLK after rising edge of FS
    pub lrp, lrp_set: 7;
    /// Word length (24-bits default) of audio data stream
    ///
    /// * `0` = 16-bit word length
    /// * `1` = 20-bit word length
    /// * `2` = 24-bit word length
    /// * `3` = 32-bit word length
    pub wlen, wlen_set: 6,5;
    /// Audio interface data format (default setting is I2S)
    ///
    /// * `0` = right justified
    /// * `1` = left justified
    /// * `2` = standard I2S format
    /// * `3` = PCMA or PCMB audio data format option
    pub aifmt, aifmt_set: 4,3;
    /// DAC audio data left-right ordering
    ///
    /// * `false` = left DAC data in left phase of LRP
    /// * `true` = left DAC data in right phase of LRP (left-right reversed)
    pub dacphs, dacphs_set: 2;
    /// ADC audio data left-right ordering
    ///
    /// * `false` = left ADC data is output in left phase of LRP
    /// * `true` = left ADC data is output in right phase of LRP (left-right
    ///   reversed)
    pub adcphs, adcphs_set: 1;
    /// Mono operation enable
    ///
    /// * `false` = normal stereo mode of operation
    /// * `true` = mono mode with audio data in left phase of LRP
    pub mono, mono_set: 0;
}

/// Whether 8-bit companding is enabled
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum CompandingMode {
    /// Companding is off
    Off = 0,
    /// Companding is on, and using µ-law
    ULaw = 2,
    /// Companding is on, and using A-law
    ALaw = 3,
}

impl From<u8> for CompandingMode {
    fn from(value: u8) -> Self {
        match value {
            2 => CompandingMode::ULaw,
            3 => CompandingMode::ALaw,
            _ => CompandingMode::Off,
        }
    }
}

bitfield! {
    /// Companding register contents
    ///
    /// See [`read_companding`](crate::Codec::read_companding),
    /// [`write_companding`](crate::Codec::write_companding) and
    /// [`modify_companding`](crate::Codec::modify_companding)
    pub struct Companding(u16);
    impl Debug;
    u8;
    /// 8-bit word enable for companding mode of operation
    ///
    /// * `false` = normal operation (no companding)
    /// * `true` = 8-bit operation for companding mode
    pub cmb8, cmb8_set: 5;
    /// DAC companding mode control
    ///
    /// * `0` = off (normal linear operation)
    /// * `1` = reserved
    /// * `2` = u-law companding
    /// * `3` = A-law companding
    pub daccm, daccm_set: 4, 3;
    /// ADC companding mode control
    pub into CompandingMode, adccm, adccm_set: 2, 1;
    /// DAC audio data input option to route directly to ADC data stream
    ///
    /// * `false` = no passthrough, normal operation
    /// * `true` = ADC output data stream routed to DAC input data path
    pub addap, addap_set: 0;
}

bitfield! {
    /// Clock Control 1 register contents
    ///
    /// See [`read_clockcontrol1`](crate::Codec::read_clockcontrol1),
    /// [`write_clockcontrol1`](crate::Codec::write_clockcontrol1) and
    /// [`modify_clockcontrol1`](crate::Codec::modify_clockcontrol1)
    pub struct ClockControl1(u16);
    impl Debug;
    u8;
    /// Master clock source selection control
    ///
    /// * `false` = MCLK, pin#11 used as master clock
    /// * `true` = internal PLL oscillator output used as master clock
    pub clkm, clkm_set: 8;
    /// Scaling of master clock source for internal 256fs rate (divide by 2 =
    /// default)
    ///
    /// * `0` = divide by 1
    /// * `1` = divide by 1.5
    /// * `2` = divide by 2
    /// * `3` = divide by 3
    /// * `4` = divide by 4
    /// * `5` = divide by 6
    /// * `6` = divide by 8
    /// * `7` = divide by 12
    pub mclksel, mclksel_set: 7, 5;
    /// Scaling of output frequency at BCLK pin#8 when chip is in master mode
    ///
    /// * `0` = divide by 1
    /// * `1` = divide by 2
    /// * `2` = divide by 4
    /// * `3` = divide by 8
    /// * `4` = divide by 16
    /// * `5` = divide by 32
    pub bclksel, bclksel_set: 4, 2;
    /// Enables chip master mode to drive FS and BCLK outputs
    ///
    /// * `false` = FS and BCLK are inputs
    /// * `true` = FS and BCLK are driven as outputs by internally generated
    ///   clocks
    pub clkioen, clkioen_set: 0;
}

bitfield! {
    /// Clock Control 2 register contents
    ///
    /// See [`read_clockcontrol2`](crate::Codec::read_clockcontrol2),
    /// [`write_clockcontrol2`](crate::Codec::write_clockcontrol2) and
    /// [`modify_clockcontrol2`](crate::Codec::modify_clockcontrol2)
    pub struct ClockControl2(u16);
    impl Debug;
    u8;
    /// 4-wire control interface enable
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub fourwirecie, fourwirecie_set: 8;
    /// Audio data sample rate indication (48 kHz default). Sets up scaling for
    /// internal filter coefficients, but does not affect in any way the actual
    /// device sample rate. Should be set to value most closely matching the
    /// actual sample rate determined by 256fs internal node.
    ///
    /// * `0` = 48 kHz
    /// * `1` = 32 kHz
    /// * `2` = 24 kHz
    /// * `3` = 16 kHz
    /// * `4` = 12 kHz
    /// * `5` = 8 kHz
    /// * `6` = reserved
    /// * `7` = reserved
    pub smplr, smplr_set: 3, 1;
    /// Slow timer clock enable. Starts internal timer clock derived by dividing
    /// master clock.
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub sclken, sclken_set: 0;
}

/// The operating modes CSB/GPIO1 can be in
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum Gpio1Selection {
    /// use as input subject to MODE pin#18 input logic level
    Input = 0,
    /// Temperature OK status output (low = thermal shutdown)
    TemperatureOk = 2,
    /// DAC automute condition (high = one or both DACs automuted)
    DacIsAutomute = 3,
    /// output divided PLL clock
    PllClock = 4,
    /// PLL locked condition (high = PLL locked)
    PllLocked = 5,
    /// output set to high condition
    LogicHigh = 6,
    /// output set to low condition
    LogicLow = 7,
}

impl From<u8> for Gpio1Selection {
    fn from(value: u8) -> Self {
        match value {
            2 => Gpio1Selection::TemperatureOk,
            3 => Gpio1Selection::DacIsAutomute,
            4 => Gpio1Selection::PllClock,
            5 => Gpio1Selection::PllLocked,
            6 => Gpio1Selection::LogicHigh,
            7 => Gpio1Selection::LogicLow,
            _ => Gpio1Selection::Input,
        }
    }
}

bitfield! {
    /// GPIO register contents
    ///
    /// See [`read_gpio`](crate::Codec::read_gpio),
    /// [`write_gpio`](crate::Codec::write_gpio) and
    /// [`modify_gpio`](crate::Codec::modify_gpio)
    pub struct GPIO(u16);
    impl Debug;
    u8;
    /// Clock divisor applied to PLL clock for output from a GPIO pin
    ///
    /// * `0` = divide by 1
    /// * `1` = divide by 2
    /// * `2` = divide by 3
    /// * `3` = divide by 4
    pub gpio1pll, gpio1pll_set: 5, 4;
    /// GPIO1 polarity inversion control
    ///
    /// * `false` = normal logic sense of GPIO signal
    /// * `true` = inverted logic sense of GPIO signal
    pub gpio1pl, gpio1pl_set: 3;
    /// CSB/GPIO1 function select (input default)
    pub into Gpio1Selection, gpio1sel, gpio1sel_set: 2, 0;
}

bitfield! {
    /// Jack Detect 1 register contents
    ///
    /// See [`read_jackdetect1`](crate::Codec::read_jackdetect1),
    /// [`write_jackdetect1`](crate::Codec::write_jackdetect1) and
    /// [`modify_jackdetect1`](crate::Codec::modify_jackdetect1)
    pub struct JackDetect1(u16);
    impl Debug;
    u8;
    /// Automatically enable internal bias amplifiers on jack detection state as
    /// sensed through GPIO pin associated to jack detection function
    ///
    /// * `0` = disabled
    /// * `1` = enable bias amplifiers on jack at `false` level
    /// * `2` = enable bias amplifiers on jack at `true` level
    /// * `3` = reserved
    pub jckmiden, jckmiden_set: 8, 7;
    /// Jack detection feature enable
    ///
    /// * `false` = disabled
    /// * `true` = enable jack detection associated functionality
    pub jacden, jacden_set: 6;
    /// Select jack detect pin (GPIO1 default)
    ///
    /// * `0` = GPIO1 is used for jack detection feature
    /// * `1` = GPIO2 is used for jack detection feature
    /// * `2` = GPIO3 is used for jack detection feature
    /// * `3` = reserved
    pub jckdio, jckdio_set: 5, 4;
}

bitfield! {
    /// DAC Control register contents
    ///
    /// See [`read_daccontrol`](crate::Codec::read_daccontrol),
    /// [`write_daccontrol`](crate::Codec::write_daccontrol) and
    /// [`modify_daccontrol`](crate::Codec::modify_daccontrol)
    pub struct DACControl(u16);
    impl Debug;
    u8;
    /// Softmute feature control for DACs
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub softmt, softmt_set: 6;
    /// DAC oversampling rate selection (64X default)
    ///
    /// * `false` = 64x oversampling
    /// * `true` = 128x oversampling
    pub dacos, dacos_set: 3;
    /// DAC automute function enable
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub automt, automt_set: 2;
    /// DAC right channel output polarity control
    ///
    /// * `false` = normal polarity
    /// * `true` = inverted polarity
    pub rdacpl, rdacpl_set: 1;
    /// DAC left channel output polarity control
    ///
    /// * `false` = normal polarity
    /// * `true` = inverted polarity
    pub ldacpl, ldacpl_set: 0;
}

bitfield! {
    /// Left DAC Volume register contents
    ///
    /// See [`read_leftdacvolume`](crate::Codec::read_leftdacvolume),
    /// [`write_leftdacvolume`](crate::Codec::write_leftdacvolume) and
    /// [`modify_leftdacvolume`](crate::Codec::modify_leftdacvolume)
    pub struct LeftDACVolume(u16);
    impl Debug;
    u8;
    /// DAC volume update bit feature. Write-only bit for synchronized L/R DAC
    /// changes
    ///
    /// * `false` = on [`LeftDACVolume`] write, new [`LeftDACVolume`] value
    ///   stored in temporary register
    /// * `true` = on [`LeftDACVolume`] write, new [`LeftDACVolume`] and pending
    ///   [`RightDACVolume`] values become active
    pub _, ldacvu_set: 8;
    /// DAC left digital volume control (0 dB default attenuation value).
    /// Expressed as an attenuation value in 0.5 dB steps as follows:
    ///
    /// * `0` = digital mute condition
    /// * `1` = -127.0 dB (highly attenuated)
    /// * `2` = -126.5 dB attenuation
    /// * all intermediate 0.5 step values through to maximum
    /// * `254` = -0.5 dB attenuation
    /// * `255` = 0.0 dB attenuation (no attenuation
    pub ldacgain, ldacgain_set: 7, 0;
}

bitfield! {
    /// Right DAC Volume register contents
    ///
    /// See [`read_rightdacvolume`](crate::Codec::read_rightdacvolume),
    /// [`write_rightdacvolume`](crate::Codec::write_rightdacvolume) and
    /// [`modify_rightdacvolume`](crate::Codec::modify_rightdacvolume)
    pub struct RightDACVolume(u16);
    impl Debug;
    u8;
    /// DAC volume update bit feature. Write-only bit for synchronized L/R DAC
    /// changes
    ///
    /// * `false` = on [`RightDACVolume`] write, new [`RightDACVolume`] value
    ///   stored in temporary register
    /// * `true` = on [`RightDACVolume`] write, new [`RightDACVolume`] and
    ///   pending [`LeftDACVolume`] values become active
    pub _, rdacvu_set: 8;
    /// DAC right digital volume control (0 dB default attenuation value).
    /// Expressed as an attenuation value in 0.5 dB steps as follows:
    ///
    /// * `0` = digital mute condition
    /// * `1` = -127.0 dB (highly attenuated)
    /// * `2` = -126.5 dB attenuation
    /// * all intermediate 0.5 step values through to maximum
    /// * `254` = -0.5 dB attenuation
    /// * `255` = 0.0 dB attenuation (no attenuation
    pub rdacgain, rdacgain_set: 7, 0;
}

bitfield! {
    /// Jack Detect 2 register contents
    ///
    /// See [`read_jackdetect2`](crate::Codec::read_jackdetect2),
    /// [`write_jackdetect2`](crate::Codec::write_jackdetect2) and
    /// [`modify_jackdetect2`](crate::Codec::modify_jackdetect2)
    pub struct JackDetect2(u16);
    impl Debug;
    u8;
    /// Outputs drivers that are automatically enabled whenever the designated
    /// jack detection input is in the `true` condition, and the jack
    /// detection feature is enabled
    ///
    /// * Bit 0 = `1`: enable Left and Right Headphone output drivers
    /// * Bit 1 = `1`: enable Left and Right Speaker output drivers
    /// * Bit 2 = `1`: enable AUXOUT2 output driver
    /// * Bit 3 = `1`: enable AUXOUT1 output driver
    pub jckdoen1_lrhp, jckdoen1_set: 7, 4;
    /// Outputs drivers that are automatically enabled whenever the designated
    /// jack detection input is in the low condition, and the jack detection
    /// feature is enabled
    ///
    /// * Bit 0 = `1`: enable Left and Right Headphone output drivers
    /// * Bit 1 = `1`: enable Left and Right Speaker output drivers
    /// * Bit 2 = `1`: enable AUXOUT2 output driver
    /// * Bit 3 = `1`: enable AUXOUT1 output driver
    pub jckdoen0_lrhp, jckdoen0_set: 3, 0;
}

bitfield! {
    /// ADC Control register contents
    ///
    /// See [`read_adccontrol`](crate::Codec::read_adccontrol),
    /// [`write_adccontrol`](crate::Codec::write_adccontrol) and
    /// [`modify_adccontrol`](crate::Codec::modify_adccontrol)
    pub struct ADCControl(u16);
    impl Debug;
    u8;
    /// High pass filter enable control for filter of ADC output data stream
    ///
    /// * `false` = high pass filter disabled
    /// * `true` = high pass filter enabled
    pub hpfen, hpfen_set: 8;
    /// High pass filter mode selection
    ///
    /// * `false` = normal audio mode, 1st order 3.7 Hz high pass filter for DC
    ///   blocking
    /// * `true` = application specific mode, variable 2nd order high pass
    ///   filter
    pub hpfam, hpfam_set: 7;
    /// Application specific mode cutoff frequency selection
    ///
    /// See datasheet for details.
    pub hpf, hpf_set: 6, 4;
    /// ADC oversampling rate selection (64X default)
    ///
    /// * `false` = 64x oversampling rate for reduced power
    /// * `true` = 128x oversampling for better SNR
    pub adcos, adcos_set: 3;
    /// ADC right channel polarity control
    ///
    /// * `false` = normal polarity
    /// * `true` = sign of RADC output is inverted from normal polarity
    pub radcpl, radcpl_set: 1;
    /// ADC left channel polarity control
    ///
    /// * `false` = normal polarity
    /// * `true` = sign of LADC output is inverted from normal polarity
    pub ladcpl, ladcpl_set: 0;
}

bitfield! {
    /// Left ADC Volume register contents
    ///
    /// See [`read_leftadcvolume`](crate::Codec::read_leftadcvolume),
    /// [`write_leftadcvolume`](crate::Codec::write_leftadcvolume) and
    /// [`modify_leftadcvolume`](crate::Codec::modify_leftadcvolume)
    pub struct LeftADCVolume(u16);
    impl Debug;
    u8;
    /// ADC volume update bit feature. Write-only bit for synchronized L/R ADC
    /// changes.
    ///
    /// * If `false` on [`LeftADCVolume`] write, new [`LeftADCVolume`] value
    ///   stored in temporary register
    /// * If `true` on [`LeftADCVolume`] write, new [`LeftADCVolume`] and
    ///   pending [`RightADCVolume`] values become active
    pub _, ladcvu_set: 8;
    /// ADC left digital volume control (0 dB default attenuation value).
    /// Expressed as an attenuation value in 0.5 dB steps as follows:
    ///
    /// * `0` = digital mute condition
    /// * `1` = -127.0 dB (highly attenuated)
    /// * `2` = -126.5 dB attenuation
    /// * *all intermediate 0.5 step values through maximum volume*
    /// * `254` = -0.5 dB attenuation
    /// * `255` = 0.0 dB attenuation (no attenuation)
    pub ladcgain, ladcgain_set: 7, 0;
}

bitfield! {
    /// Right ADC Volume register contents
    ///
    /// See [`read_rightadcvolume`](crate::Codec::read_rightadcvolume),
    /// [`write_rightadcvolume`](crate::Codec::write_rightadcvolume) and
    /// [`modify_rightadcvolume`](crate::Codec::modify_rightadcvolume)
    pub struct RightADCVolume(u16);
    impl Debug;
    u8;
    /// ADC volume update bit feature. Write-only bit for synchronized L/R ADC
    /// changes.
    ///
    /// * If `false` on [`RightADCVolume`] write, new [`RightADCVolume`] value
    /// * stored in temporary register
    /// * If `true` on [`RightADCVolume`] write, new [`RightADCVolume`] and
    /// * pending [`LeftADCVolume`] values become active
    pub _, radcvu_set: 8;
    /// ADC right digital volume control (0 dB default attenuation value).
    /// Expressed as an attenuation value in 0.5 dB steps as follows:
    ///
    /// * `0` = digital mute condition
    /// * `1` = -127.0 dB (highly attenuated)
    /// * `2` = -126.5 dB attenuation
    /// * *all intermediate 0.5 step values through maximum volume*
    /// * `254` = -0.5 dB attenuation
    /// * `255` = 0.0 dB attenuation (no attenuation)
    pub radcgain, radcgain_set: 7, 0;
}

bitfield! {
    /// EQ1-high cutoff register contents
    ///
    /// See [`read_eq1highcutoff`](crate::Codec::read_eq1highcutoff),
    /// [`write_eq1highcutoff`](crate::Codec::write_eq1highcutoff) and
    /// [`modify_eq1highcutoff`](crate::Codec::modify_eq1highcutoff)
    pub struct EQ1HighCutoff(u16);
    impl Debug;
    u8;
    /// Equalizer and 3D audio processing block assignment.
    ///
    /// * `false` = block operates on digital stream from ADC
    /// * `true` = block operates on digital stream to DAC (default on reset)
    pub eqm, eqm_set: 8;
    /// Equalizer band 1 high pass -3 dB cut-off frequency selection
    ///
    /// * `0` = 80 Hz
    /// * `1` = 105 Hz (default)
    /// * `2` = 135 Hz
    /// * `3` = 175 Hz
    pub eq1cf, eq1cf_set: 6, 5;
    /// EQ Band 1 digital gain control. Expressed as a gain or attenuation in
    /// 1 dB steps. `12` = 0.0 dB default unity gain value.
    ///
    /// * `0` = +12 dB
    /// * `1` = +11 dB
    /// * *all intermediate 1.0 dB step values through minimum gain*
    /// * `24` = -12 dB
    /// * `25` and larger values are reserved
    pub eq1gc, eq1gc_set: 4, 0;
}

bitfield! {
    /// EQ2-peak 1 register contents
    ///
    /// See [`read_eq2peak1`](crate::Codec::read_eq2peak1),
    /// [`write_eq2peak1`](crate::Codec::write_eq2peak1) and
    /// [`modify_eq2peak1`](crate::Codec::modify_eq2peak1)
    pub struct EQ2Peak1(u16);
    impl Debug;
    u8;
    /// Equalizer Band 2 bandwidth selection
    ///
    /// * `false` = narrow band characteristic (default)
    /// * `true` = wide band characteristic
    pub eq2bw, eq2bw_set: 8;
    /// Equalizer Band 2 center frequency selection
    ///
    /// * `0` = 230 Hz
    /// * `1` = 300 Hz (default)
    /// * `2` = 385 Hz
    /// * `3` = 500 Hz
    pub eq2cf, eq2cf_set: 6, 5;
    /// EQ Band 2 digital gain control.
    ///
    /// Expressed as a gain or attenuation in 1 dB steps. `12` = 0.0 dB default
    /// unity gain value.
    ///
    /// * `0` = +12 dB
    /// * `1` = +11 dB
    /// * *all intermediate 1.0 dB step values through minimum gain*
    /// * `24` = -12 dB
    /// * `25` and larger values are reserved
    pub eq2gc, eq2gc_set: 4, 0;
}

bitfield! {
    /// EQ3-peak 2 register contents
    ///
    /// See [`read_eq3peak2`](crate::Codec::read_eq3peak2),
    /// [`write_eq3peak2`](crate::Codec::write_eq3peak2) and
    /// [`modify_eq3peak2`](crate::Codec::modify_eq3peak2)
    pub struct EQ3Peak2(u16);
    impl Debug;
    u8;
    /// Equalizer Band 3 bandwidth selection
    ///
    /// * `false` = narrow band characteristic (default)
    /// * `true` = wide band characteristic
    pub eq3bw, eq3bw_set: 8;
    /// Equalizer Band 3 center frequency selection
    ///
    /// * `0` = 650 Hz
    /// * `1` = 850 Hz (default)
    /// * `2` = 1.1 kHz
    /// * `3` = 1.4 kHz
    pub eq3cf, eq3cf_set: 6, 5;
    /// EQ Band 3 digital gain control.
    ///
    /// Expressed as a gain or attenuation in 1 dB steps. `12` = 0.0 dB default
    /// unity gain value.
    ///
    /// * `0` = +12 dB
    /// * `1` = +11 dB
    /// * *all intermediate 1.0 dB step values through minimum gain*
    /// * `24` = -12 dB
    /// * `25` and larger values are reserved
    pub eq3gc, eq3gc_set: 4, 0;
}

bitfield! {
    /// EQ4-peak 3 register contents
    ///
    /// See [`read_eq4peak3`](crate::Codec::read_eq4peak3),
    /// [`write_eq4peak3`](crate::Codec::write_eq4peak3) and
    /// [`modify_eq4peak3`](crate::Codec::modify_eq4peak3)
    pub struct EQ4Peak3(u16);
    impl Debug;
    u8;
    /// Equalizer Band 4 bandwidth selection
    ///
    /// * `false` = narrow band characteristic (default)
    /// * `true` = wide band characteristic
    pub eq4bw, eq4bw_set: 8;
    /// Equalizer Band 4 center frequency selection
    ///
    /// * `0` = 1.8 kHz
    /// * `1` = 2.4 kHz (default)
    /// * `2` = 3.2 kHz
    /// * `3` = 4.1 kHz
    pub eq4cf, eq4cf_set: 6, 5;
    /// EQ Band 4 digital gain control.
    ///
    /// Expressed as a gain or attenuation in 1 dB steps. `12` = 0.0 dB default
    /// unity gain value.
    ///
    /// * `0` = +12 dB
    /// * `1` = +11 dB
    /// * *all intermediate 1.0 dB step values through minimum gain*
    /// * `24` = -12 dB
    /// * `25` and larger values are reserved
    pub eq4gc, eq4gc_set: 4, 0;
}

bitfield! {
    /// EQ5-low cutoff register contents
    ///
    /// See [`read_eq5lowcutoff`](crate::Codec::read_eq5lowcutoff),
    /// [`write_eq5lowcutoff`](crate::Codec::write_eq5lowcutoff) and
    /// [`modify_eq5lowcutoff`](crate::Codec::modify_eq5lowcutoff)
    pub struct EQ5LowCutoff(u16);
    impl Debug;
    u8;
    /// Equalizer Band 5 center frequency selection
    ///
    /// * `0` = 5.3 kHz
    /// * `1` = 6.9 kHz (default)
    /// * `2` = 9.0 kHz
    /// * `3` = 11.7 kHz
    pub eq5cf, eq5cf_set: 6, 5;
    /// EQ Band 5 digital gain control.
    ///
    /// Expressed as a gain or attenuation in 1 dB steps. `12` = 0.0 dB default
    /// unity gain value.
    ///
    /// * `0` = +12 dB
    /// * `1` = +11 dB
    /// * *all intermediate 1.0 dB step values through minimum gain*
    /// * `24` = -12 dB
    /// * `25` and larger values are reserved
    pub eq5gc, eq5gc_set: 4, 0;
}

bitfield! {
    /// DAC Limiter 1 register contents
    ///
    /// See [`read_daclimiter1`](crate::Codec::read_daclimiter1),
    /// [`write_daclimiter1`](crate::Codec::write_daclimiter1) and
    /// [`modify_daclimiter1`](crate::Codec::modify_daclimiter1)
    pub struct DACLimiter1(u16);
    impl Debug;
    u8;
    /// DAC digital limiter control bit
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub daclimen, daclimen_set: 8;
    /// DAC limiter decay time. Proportional to actual DAC sample rate. Duration
    /// doubles with each binary bit value. Values given here are for 44.1 kHz
    /// sample rate.
    ///
    /// * `0` = 0.544 ms
    /// * `1` = 1.09 ms
    /// * `2` = 2.18 ms
    /// * `3` = 4.36 ms (default)
    /// * `4` = 8.72 ms
    /// * `5` = 17.4 ms
    /// * `6` = 34.8 ms
    /// * `7` = 69.6 ms
    /// * `8` = 139 ms
    /// * `9` = 278 ms
    /// * `10` = 566 ms
    /// * `11` through `15` = 1130 ms
    pub daclimdcy, daclimdcy_set: 7, 4;
    /// DAC limiter attack time. Proportional to actual DAC sample rate.
    /// Duration doubles with each binary bit value. Values given here are for
    /// 44.1 kHz sample rate.
    ///
    /// * `0` = 68.0 µs
    /// * `1` = 136 µs
    /// * `2` = 272 µs (default)
    /// * `3` = 544 µs
    /// * `4` = 1.09 ms
    /// * `5` = 2.18 ms
    /// * `6` = 4.36 ms
    /// * `7` = 8.72 ms
    /// * `8` = 17.4 ms
    /// * `9` = 34.8 ms
    /// * `10` = 69.6 ms
    /// * `11` through `15` = 139 ms
    pub daclimatk, daclimatk_set: 4, 0;
}

bitfield! {
    /// DAC Limiter 2 register contents
    ///
    /// See [`read_daclimiter2`](crate::Codec::read_daclimiter2),
    /// [`write_daclimiter2`](crate::Codec::write_daclimiter2) and
    /// [`modify_daclimiter2`](crate::Codec::modify_daclimiter2)
    pub struct DACLimiter2(u16);
    impl Debug;
    u8;
    /// DAC limiter threshold in relation to full scale output level (0.0 dB =
    /// full scale)
    ///
    /// * `0` = -1.0 dB
    /// * `1` = -2.0 dB
    /// * `2` = -3.0 dB
    /// * `3` = -4.0 dB
    /// * `4` = -5.0 dB
    /// * `5` through `7` = -6.0 dB
    pub daclimthl, daclimthl_set: 6, 4;
    /// DAC limiter maximum automatic gain boost in limiter mode.
    ///
    /// If R24 (`DACLimiter1`) limiter mode is disabled, specified gain value
    /// will be applied in addition to other gain values in the signal path.
    ///
    /// * `0` = 0.0 dB (default)
    /// * `1` = +1.0 dB
    /// * *Gain value increases in 1.0 dB steps for each binary value*
    /// * `12` = +12 dB (maximum allowed boost value)
    /// * `13` through `15` = reserved
    pub daclimbst, daclimbst_set: 3, 0;
}

bitfield! {
    /// Notch Filter 1 register contents
    ///
    /// See [`read_notchfilter1`](crate::Codec::read_notchfilter1),
    /// [`write_notchfilter1`](crate::Codec::write_notchfilter1) and
    /// [`modify_notchfilter1`](crate::Codec::modify_notchfilter1)
    pub struct NotchFilter1(u16);
    impl Debug;
    u8;
    /// Update bit feature for simultaneous change of all notch filter
    /// parameters. Write-only bit.
    ///
    /// * Setting `true` on [`NotchFilter1`] register write operation causes new
    /// [`NotchFilter1`] value and any pending value in [`NotchFilter2`],
    /// [`NotchFilter3`], or [`NotchFilter4`] to go into effect.
    /// * Setting `false` on [`NotchFilter1`] register write causes new value to
    /// be pending an update bit event on [`NotchFilter1`], [`NotchFilter2`],
    /// [`NotchFilter3`], or [`NotchFilter4`].
    pub _, nfcu1_set: 8;
    /// Notch filter control bit
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub nfcen, nfcen_set: 7;
    /// Notch filter A0 coefficient most significant bits.
    ///
    /// See datasheet for details.
    pub nfca0high, nfca0high_set: 6, 0;
}

bitfield! {
    /// Notch Filter 2 register contents
    ///
    /// See [`read_notchfilter2`](crate::Codec::read_notchfilter2),
    /// [`write_notchfilter2`](crate::Codec::write_notchfilter2) and
    /// [`modify_notchfilter2`](crate::Codec::modify_notchfilter2)
    pub struct NotchFilter2(u16);
    impl Debug;
    u8;
    /// Update bit feature for simultaneous change of all notch filter
    /// parameters. Write-only bit.
    ///
    /// * Setting `true` on [`NotchFilter2`] register write operation causes new
    /// [`NotchFilter2`] value and any pending value in [`NotchFilter1`],
    /// [`NotchFilter3`], or [`NotchFilter4`] to go into effect.
    /// * Setting `false` on [`NotchFilter2`] register write causes new value to
    /// be pending an update bit event on [`NotchFilter1`], [`NotchFilter2`],
    /// [`NotchFilter3`], or [`NotchFilter4`].
    pub _, nfcu2_set: 8;
    /// Notch filter A0 coefficient least significant bits
    ///
    /// See datasheet for details.
    pub nfcaolow, nfcaolow_set: 6, 0;
}

bitfield! {
    /// Notch Filter 3 register contents
    ///
    /// See [`read_notchfilter3`](crate::Codec::read_notchfilter3),
    /// [`write_notchfilter3`](crate::Codec::write_notchfilter3) and
    /// [`modify_notchfilter3`](crate::Codec::modify_notchfilter3)
    pub struct NotchFilter3(u16);
    impl Debug;
    u8;
    /// Update bit feature for simultaneous change of all notch filter
    /// parameters. Write-only bit.
    ///
    /// * Setting `true` on [`NotchFilter3`] register write operation causes new
    /// [`NotchFilter3`] value and any pending value in [`NotchFilter1`],
    /// [`NotchFilter2`], or [`NotchFilter4`] to go into effect.
    /// * Setting `false` on [`NotchFilter3`] register write causes new value to
    /// be pending an update bit event on [`NotchFilter1`], [`NotchFilter2`],
    /// [`NotchFilter3`], or [`NotchFilter4`].
    pub _, nfcu3_set: 8;
    /// Notch filter A1 coefficient most significant bits
    ///
    /// See datasheet for details.
    pub nfca1high, nfca1high_set: 6, 0;
}

bitfield! {
    /// Notch Filter 4 register contents
    ///
    /// See [`read_notchfilter4`](crate::Codec::read_notchfilter4),
    /// [`write_notchfilter4`](crate::Codec::write_notchfilter4) and
    /// [`modify_notchfilter4`](crate::Codec::modify_notchfilter4)
    pub struct NotchFilter4(u16);
    impl Debug;
    u8;
    /// Update bit feature for simultaneous change of all notch filter
    /// parameters. Write-only bit.
    ///
    /// * Setting `true` on [`NotchFilter4`] register write operation causes new
    /// [`NotchFilter4`] value and any pending value in [`NotchFilter1`],
    /// [`NotchFilter2`], or [`NotchFilter3`] to go into effect.
    /// * Setting `false` on [`NotchFilter4`] register write causes new value to
    /// be pending an update bit event on [`NotchFilter1`], [`NotchFilter2`],
    /// [`NotchFilter3`], or [`NotchFilter4`].
    pub _, nfcu4_set: 8;
    /// Notch filter A1 coefficient least significant bits
    ///
    /// See datasheet for details.
    pub nfca1low, nfca1low_set: 6, 0;
}

bitfield! {
    /// ALC Control 1 register contents
    ///
    /// See [`read_alccontrol1`](crate::Codec::read_alccontrol1),
    /// [`write_alccontrol1`](crate::Codec::write_alccontrol1) and
    /// [`modify_alccontrol1`](crate::Codec::modify_alccontrol1)
    pub struct ALCControl1(u16);
    impl Debug;
    u8;
    /// Automatic Level Control function control bits
    ///
    /// * `0` = right and left ALCs disabled
    /// * `1` = only right channel ALC enabled
    /// * `2` = only left channel ALC enabled
    /// * `3` = both right and left channel ALCs enabled
    pub alcen, alcen_set: 8, 7;
    /// Set maximum gain limit for PGA volume setting changes under ALC control
    ///
    /// * `7` = +35.25 dB (default)
    /// * `6` = +29.25 dB
    /// * `5` = +23.25 dB
    /// * `4` = +17.25 dB
    /// * `3` = +11.25 dB
    /// * `2` = +5.25 dB
    /// * `1` = -0.75 dB
    /// * `0` = -6.75 dB
    pub alcmxgain, alcmxgain_set: 5, 3;
    /// Set minimum gain value limit for PGA volume setting changes under ALC
    /// control
    ///
    /// * `0` = -12 dB (default)
    /// * `1` = -6.0 dB
    /// * `2` = 0.0 dB
    /// * `3` = +6.0 dB
    /// * `4` = +12 dB
    /// * `5` = +18 dB
    /// * `6` = +24 dB
    /// * `7` = +30 dB
    pub alcmngain, alcmngain_set: 2, 0;
}

bitfield! {
    /// ALC Control 2 register contents
    ///
    /// See [`read_alccontrol2`](crate::Codec::read_alccontrol2),
    /// [`write_alccontrol2`](crate::Codec::write_alccontrol2) and
    /// [`modify_alccontrol2`](crate::Codec::modify_alccontrol2)
    pub struct ALCControl2(u16);
    impl Debug;
    u8;
    /// Hold time before ALC automated gain increase
    ///
    /// * `0` = 0.00 ms (default)
    /// * `1` = 2.00 ms
    /// * `2` = 4.00 ms
    /// * *time value doubles with each bit value increment*
    /// * `9` = 512 ms
    /// * `10` through `15` = 1000 ms
    pub alcht, alcht_set: 7, 4;
    /// ALC target level at ADC output
    ///
    /// * `15` = -1.5 dB below full scale (FS)
    /// * `14` = -1.5 dB FS (same value as `15`)
    /// * `13` = -3.0 dB FS
    /// * `12` = -4.5 dB FS
    /// * `11` = -6.0 dB FS (default)
    /// * *target level varies 1.5 dB per binary step throughout control range*
    /// * `1` = -21.0 dB FS
    /// * `0` = -22.5 dB FS (lowest possible target signal level)
    pub alcsl, alcsl_set: 3, 0;
}

bitfield! {
    /// ALC Control 3 register contents
    ///
    /// See [`read_alccontrol3`](crate::Codec::read_alccontrol3),
    /// [`write_alccontrol3`](crate::Codec::write_alccontrol3) and
    /// [`modify_alccontrol3`](crate::Codec::modify_alccontrol3)
    pub struct ALCControl3(u16);
    impl Debug;
    u8;
    /// ALC mode control setting
    ///
    /// * `false` = normal ALC operation
    /// * `true` = Limiter Mode operation
    pub alcm, alcm_set: 8;
    /// ALC decay time duration per step of gain change for gain increase of
    /// 0.75 dB of PGA gain. Total response time can be estimated by the total
    /// number of steps necessary to compensate for a given magnitude change in
    /// the signal. For example, a 6 dB decrease in the signal would require
    /// eight ALC steps to compensate.
    ///
    /// Step size for each mode is given by:
    ///
    /// | Normal Mode                | Limiter Mode               |
    /// | -------------------------- | -------------------------- |
    /// | `0` = 500 µs               | `0` = 125 µs               |
    /// | `1` = 1.0 ms               | `1` = 250 µs               |
    /// | `2` = 2.0 ms (default)     | `2` = 500 µs (default)     |
    /// | ...                        | ...                        |
    /// | `8` = 128 ms               | `8` = 32 ms                |
    /// | `9` = 256 ms               | `9` = 64 ms                |
    /// | `10` through `15` = 512 ms | `10` through `15` = 128 ms |
    pub alcdcy, alcdcy_set: 7, 4;
    /// ALC attack time duration per step of gain change for gain decrease of
    /// 0.75 dB of PGA gain. Total response time can be estimated by the total
    /// number of steps necessary to compensate for a given magnitude change in
    /// the signal. For example, a 6 dB increase in the signal would require
    /// eight ALC steps to compensate.
    ///
    /// Step size for each mode is given by:
    ///
    /// | Normal Mode                | Limiter Mode                |
    /// | -------------------------- | --------------------------- |
    /// | `0` = 125 µs               | `0` = 31 µs                 |
    /// | `1` = 250 µs               | `1` = 62 µs                 |
    /// | `2` = 500 µs (default)     | `2` = 124 µs (default)      |
    /// | ...                        | ...                         |
    /// | `8` = 26.5 ms              | `8` = 7.95 ms               |
    /// | `9` = 53.0 ms              | `9` = 15.9 ms               |
    /// | `10` through `15` = 128 ms | `10` through `15` = 31.7 ms |
    pub alcatk, alcatk_set: 3, 0;
}

bitfield! {
    /// Noise Gate register contents
    ///
    /// See [`read_noisegate`](crate::Codec::read_noisegate),
    /// [`write_noisegate`](crate::Codec::write_noisegate) and
    /// [`modify_noisegate`](crate::Codec::modify_noisegate)
    pub struct NoiseGate(u16);
    impl Debug;
    u8;
    /// ALC noise gate function control bit
    ///
    /// * `false` = disabled
    /// * `true` = enabled
    pub alcnen, alcnen_set: 3;
    /// ALC noise gate threshold level
    ///
    /// * `0` = -39 dB (default)
    /// * `1` = -45 dB
    /// * `2` = -51 dB
    /// * `3` = -57 dB
    /// * `4` = -63 dB
    /// * `5` = -69 dB
    /// * `6` = -75 dB
    /// * `7` = -81 dB
    pub alcnth, alcnth_set: 2, 0;


}

bitfield! {
    /// PLL N register contents
    ///
    /// See [`read_plln`](crate::Codec::read_plln),
    /// [`write_plln`](crate::Codec::write_plln) and
    /// [`modify_plln`](crate::Codec::modify_plln)
    pub struct PllN(u16);
    impl Debug;
    u8;
    /// Control bit for divide by 2 pre-scale of MCLK path to PLL clock input
    ///
    /// * `false` = MCLK divide by 1 (default)
    /// * `true` = MCLK divide by 2
    pub pllmclk, pllmclk_set: 4;
    /// Integer portion of PLL input/output frequency ratio divider. Decimal
    /// value should be constrained to 6, 7, 8, 9, 10, 11, or 12. Default
    /// decimal value is 8.
    ///
    /// See datasheet for details.
    pub plln, plln_set: 3, 0;
}

bitfield! {
    /// PLL K 1 register contents
    ///
    /// See [`read_pllk1`](crate::Codec::read_pllk1),
    /// [`write_pllk1`](crate::Codec::write_pllk1) and
    /// [`modify_pllk1`](crate::Codec::modify_pllk1)
    pub struct PllK1(u16);
    impl Debug;
    u8;
    /// High order bits of fractional portion of PLL input/output frequency
    /// ratio divider.
    ///
    /// See datasheet for details.
    pub pllkhigh, pllkhigh_set: 5, 0;
}

bitfield! {
    /// PLL K 2 register contents
    ///
    /// See [`read_pllk2`](crate::Codec::read_pllk2),
    /// [`write_pllk2`](crate::Codec::write_pllk2) and
    /// [`modify_pllk2`](crate::Codec::modify_pllk2)
    pub struct PllK2(u16);
    impl Debug;
    u16;
    /// Middle order bits of fractional portion of PLL input/output frequency
    /// ratio divider.
    ///
    /// See datasheet for details.
    pub pllkmedium, pllkmedium_set: 8, 0;
}

bitfield! {
    /// PLL K 3 register contents
    ///
    /// See [`read_pllk3`](crate::Codec::read_pllk3),
    /// [`write_pllk3`](crate::Codec::write_pllk3) and
    /// [`modify_pllk3`](crate::Codec::modify_pllk3)
    pub struct PllK3(u16);
    impl Debug;
    u16;
    /// Low order bits of fractional portion of PLL input/output frequency ratio
    /// divider.
    ///
    /// See datasheet for details.
    pub pllklow, pllklow_set: 8, 0;
}

bitfield! {
    /// 3D control register contents
    ///
    /// See [`read_threedcontrol`](crate::Codec::read_threedcontrol),
    /// [`write_threedcontrol`](crate::Codec::write_threedcontrol) and
    /// [`modify_threedcontrol`](crate::Codec::modify_threedcontrol)
    pub struct ThreeDControl(u16);
    impl Debug;
    u8;
    /// 3D Stereo Enhancement effect depth control
    ///
    /// * `0` = 0.0% effect (disabled, default)
    /// * `1` = 6.67% effect
    /// * `2` = 13.3% effect
    /// * *3 depth varies by 6.67% per binary bit value*
    /// * `4` = 93.3% effect
    /// * `5` = 100% effect (maximum effect)
    pub threeddepth, threeddepth_set: 3, 0;
}

bitfield! {
    /// Right Speaker Submix register contents
    ///
    /// See [`read_rightspeakersubmix`](crate::Codec::read_rightspeakersubmix),
    /// [`write_rightspeakersubmix`](crate::Codec::write_rightspeakersubmix) and
    /// [`modify_rightspeakersubmix`](crate::Codec::modify_rightspeakersubmix)
    pub struct RightSpeakerSubmix(u16);
    impl Debug;
    u8;
    /// Mutes the RMIX speaker signal gain stage output in the right speaker
    /// submixer
    ///
    /// * `false` = gain stage output enabled
    /// * `true` = gain stage output muted
    pub rmixmut, rmixmut_set: 5;
    /// Right speaker submixer bypass control
    ///
    /// * `false` = right speaker amplifier directly connected to RMIX speaker
    ///   signal gain stage
    /// * `true` = right speaker amplifier connected to submixer output (inverts
    ///   RMIX for BTL)
    pub rsubbyp, rsubbyp_set: 4;
    /// RAUXIN to Right Speaker Submixer input gain control
    ///
    /// * `0` = -15 dB (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub rauxrsubg, rauxrsubg_set: 3, 1;
    /// RAUXIN to Right Speaker Submixer mute control
    ///
    /// * `false` = RAUXIN path to submixer is muted
    /// * `true` = RAUXIN path to submixer is enabled
    pub rauxsmut, rauxsmut_set: 0;
}

bitfield! {
    /// Input Control register contents
    ///
    /// See [`read_inputcontrol`](crate::Codec::read_inputcontrol),
    /// [`write_inputcontrol`](crate::Codec::write_inputcontrol) and
    /// [`modify_inputcontrol`](crate::Codec::modify_inputcontrol)
    pub struct InputControl(u16);
    impl Debug;
    u8;
    /// Microphone bias voltage selection control. Values change slightly with
    /// MICBIAS mode selection control. Open circuit voltage on MICBIAS pin#32
    /// is shown as follows as a fraction of the VDDA pin#31 supply voltage.
    ///
    /// | Normal Mode | Low Noise Mode |
    /// | ----------- | -------------- |
    /// | `0` = 0.9x  | `0` = 0.85x    |
    /// | `1` = 0.65x | `1` = 0.60x    |
    /// | `2` = 0.75x | `2` = 0.70x    |
    /// | `3` = 0.50x | `3` = 0.50x    |
    pub micbiasv, micbiasv_set: 8, 7;
    /// RLIN right line input path control to right PGA positive input
    ///
    /// * `false` = RLIN not connected to PGA positive input (default)
    /// * `true` = RLIN connected to PGA positive input
    pub rlinrpga, rlinrpga_set: 6;
    /// RMICN right microphone negative input to right PGA negative input path
    /// control
    ///
    /// * `false` = RMICN not connected to PGA negative input (default)
    /// * `true` = RMICN connected to PGA negative input
    pub rmicnrpga, rmicnrpga_set: 5;
    /// RMICP right microphone positive input to right PGA positive input enable
    ///
    /// * `false` = RMICP not connected to PGA positive input (default)
    /// * `true` = RMICP connected to PGA positive input
    pub rmicprpga, rmicprpga_set: 4;
    /// LLIN right line input path control to left PGA positive input
    ///
    /// * `false` = LLIN not connected to PGA positive input (default)
    /// * `true` = LLIN connected to PGA positive input
    pub llinlpga, llinlpga_set: 2;
    /// LMICN left microphone negative input to left PGA negative input path
    /// control
    ///
    /// * `false` = LMICN not connected to PGA negative input (default)
    /// * `true` = LMICN connected to PGA negative input
    pub lmicnlpga, lmicnlpga_set: 1;
    /// LMICP left microphone positive input to left PGA positive input enable
    ///
    /// * `false` = LMICP not connected to PGA positive input (default)
    /// * `true` = LMICP connected to PGA positive input
    pub lmicplpga, lmicplpga_set: 0;
}

bitfield! {
    /// Left Input PGA Gain register contents
    ///
    /// See [`read_leftinputpgagain`](crate::Codec::read_leftinputpgagain),
    /// [`write_leftinputpgagain`](crate::Codec::write_leftinputpgagain) and
    /// [`modify_leftinputpgagain`](crate::Codec::modify_leftinputpgagain)
    pub struct LeftInputPGAGain(u16);
    impl Debug;
    u8;
    /// PGA volume update bit feature. Write-only bit for synchronized L/R PGA
    /// changes
    ///
    /// * If `false` on [`LeftInputPGAGain`] write, new [`LeftInputPGAGain`]
    ///   value stored in temporary register If
    /// * If `true` on [`LeftInputPGAGain`] write, new [`LeftInputPGAGain`]
    ///   and pending [`RightInputPGAGain`] values become active
    pub _, lpgau_set: 8;
    /// Left channel input zero cross detection enable
    ///
    /// * `false` = gain changes to PGA register happen immediately (default)
    /// * `true` = gain changes to PGA happen pending zero crossing logic
    pub lpgazc, lpgazc_set: 7;
    /// Left channel mute PGA mute control
    ///
    /// * `false` = PGA not muted, normal operation (default)
    /// * `true` = PGA in muted condition not connected to LADC Mix/Boost stage
    pub lpgamt, lpgamt_set: 6;
    /// Left channel input PGA volume control setting. Setting becomes active
    /// when allowed by zero crossing and/or update bit features. `16` = 0.0 dB
    /// default setting
    ///
    /// * `0` = -12 dB
    /// * `1` = -11.25 dB
    /// * *volume changes in 0.75 dB steps per binary bit value*
    /// * `62` = +34.50 dB
    /// * `63` = +35.25 dB
    pub lpgagain, lpgagain_set: 5, 0;
}

bitfield! {
    /// Right Input PGA Gain register contents
    ///
    /// See [`read_rightinputpgagain`](crate::Codec::read_rightinputpgagain),
    /// [`write_rightinputpgagain`](crate::Codec::write_rightinputpgagain) and
    /// [`modify_rightinputpgagain`](crate::Codec::modify_rightinputpgagain)
    pub struct RightInputPGAGain(u16);
    impl Debug;
    u8;
    /// PGA volume update bit feature. Write-only bit for synchronized L/R PGA
    /// changes
    ///
    /// * If `false` on [`RightInputPGAGain`] write, new [`RightInputPGAGain`]
    ///   value stored in temporary register
    /// * If `true` on [`RightInputPGAGain`] write, new [`RightInputPGAGain`]
    /// * and pending [`LeftInputPGAGain`] values become active
    pub _, rpgau_set: 8;
    /// Right channel input zero cross detection enable
    ///
    /// * `false` = gain changes to PGA register happen immediately
    /// * `true` = gain changes to PGA happen pending zero crossing logic
    pub rpgazc, rpgazc_set: 7;
    /// Right channel mute PGA mute control
    ///
    /// * `false` = PGA not muted, normal operation (default)
    /// * `true` = PGA in muted condition not connected to RADC Mix/Boost stage
    pub rpgamt, rpgamt_set: 6;
    /// Right channel input PGA volume control setting. Setting becomes active
    /// when allowed by zero crossing and/or update bit features. `16` = 0.0 dB
    /// default setting.
    ///
    /// * `0` = -12 dB
    /// * `1` = -11.25 dB
    /// * *volume changes in 0.75 dB steps per binary bit value*
    /// * `62` = +34.50 dB
    /// * `63` = +35.25 dB
    pub rpgagain, rpgagain_set: 5, 0;
}

bitfield! {
    /// Left ADC Boost register contents
    ///
    /// See [`read_leftadcboost`](crate::Codec::read_leftadcboost),
    /// [`write_leftadcboost`](crate::Codec::write_leftadcboost) and
    /// [`modify_leftadcboost`](crate::Codec::modify_leftadcboost)
    pub struct LeftADCBoost(u16);
    impl Debug;
    u8;
    /// Left channel PGA boost control
    ///
    /// * `false` = no gain between PGA output and LPGA Mix/Boost stage input
    /// * `true` = +20 dB gain between PGA output and LPGA Mix/Boost stage input
    pub lpgabst, lpgabst_set: 8;
    /// Gain value between LLIN line input and LPGA Mix/Boost stage input
    ///
    /// * `0` = path disconnected (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub lpgabstgain, lpgabstgain_set: 6, 4;
    /// Gain value between LAUXIN auxiliary input and LPGA Mix/Boost stage input
    ///
    /// * `0` = path disconnected (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub lauxbstgain, lauxbstgain_set: 3, 0;
}

bitfield! {
    /// Right ADC Boost register contents
    ///
    /// See [`read_rightadcboost`](crate::Codec::read_rightadcboost),
    /// [`write_rightadcboost`](crate::Codec::write_rightadcboost) and
    /// [`modify_rightadcboost`](crate::Codec::modify_rightadcboost)
    pub struct RightADCBoost(u16);
    impl Debug;
    u8;
    /// Right channel PGA boost control
    ///
    /// * `false` = no gain between PGA output and RPGA Mix/Boost stage input
    /// * `true` = +20 dB gain between PGA output and RPGA Mix/Boost stage input
    pub rpgabst, rpgabst_set: 8;
    /// Gain value between RLIN line input and RPGA Mix/Boost stage input
    ///
    /// * `0` = path disconnected (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub rpgabstgain, rpgabstgain_set: 6, 5;
    /// Gain value between RAUXIN auxiliary input and RPGA Mix/Boost stage input
    ///
    /// * `0` = path disconnected (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub rauxbstgain, rauxbstgain_set: 3, 0;
}

bitfield! {
    /// Output Control register contents
    ///
    /// See [`read_outputcontrol`](crate::Codec::read_outputcontrol),
    /// [`write_outputcontrol`](crate::Codec::write_outputcontrol) and
    /// [`modify_outputcontrol`](crate::Codec::modify_outputcontrol)
    pub struct OutputControl(u16);
    impl Debug;
    u8;
    /// Left DAC output to RMIX right output mixer cross-coupling path control
    ///
    /// * `false` = path disconnected (default)
    /// * `true` = path connected
    pub ldacrmx, ldacrmx_set: 6;
    /// Right DAC output to LMIX left output mixer cross-coupling path control
    ///
    /// * `false` = path disconnected (default)
    /// * `true` = path connected
    pub rdaclmx, rdaclmx_set: 5;
    /// AUXOUT1 gain boost control
    ///
    /// * `false` = preferred setting for 3.6V and lower operation, -1.0x gain
    ///   (default)
    /// * `true` = required setting for greater than 3.6V operation, +1.5x gain
    pub aux1bst, aux1bst_set: 4;
    /// AUXOUT2 gain boost control
    ///
    /// * `false` = preferred setting for 3.6V and lower operation, -1.0x gain
    ///   (default)
    /// * `true` = required setting for greater than 3.6V operation, +1.5x gain
    pub aux2bst, aux2bst_set: 3;
    /// LSPKOUT and RSPKOUT speaker amplifier gain boost control
    ///
    /// * `false` = preferred setting for 3.6V and lower operation, -1.0x gain
    ///   (default)
    /// * `true` = required setting for greater than 3.6V operation, +1.5x gain
    pub spkbst, spkbst_set: 2;
    /// Thermal shutdown enable protects chip from thermal destruction on
    /// overload
    ///
    /// * `false` = disable thermal shutdown (engineering purposes, only)
    /// * `true` = enable (default) strongly recommended for normal operation
    pub tsen, tsen_set: 1;
    /// Output resistance control option for tie-off of unused or disabled
    /// outputs. Unused outputs tie to internal voltage reference for reduced
    /// pops and clicks.
    ///
    /// * `false` = nominal tie-off impedance value of 1 kΩ (default)
    /// * `true` = nominal tie-off impedance value of 30 kΩ
    pub aoutimp, aoutimp_set: 0;
}

bitfield! {
    /// Left Mixer register contents
    ///
    /// See [`read_leftmixer`](crate::Codec::read_leftmixer),
    /// [`write_leftmixer`](crate::Codec::write_leftmixer) and
    /// [`modify_leftmixer`](crate::Codec::modify_leftmixer)
    pub struct LeftMixer(u16);
    impl Debug;
    u8;
    /// Gain value between LAUXIN auxiliary input and input to LMAIN left output
    /// mixer
    ///
    /// * `0` = -15 dB (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub lauxmxgain, lauxmxgain_set: 8, 6;
    /// LAUXIN input to LMAIN left output mixer path control
    ///
    /// * `false` = LAUXIN not connected to LMAIN left output mixer (default)
    /// * `true` = LAUXIN connected to LMAIN left output mixer
    pub lauxlmx, lauxlmx_set: 5;
    /// Gain value for bypass from LADC Mix/Boost output to LMAIN left output
    /// mixer.
    ///
    /// * `0` = -15 dB (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub lbypmxgain, lbypmxgain_set: 4, 2;
    /// Left bypass path control from LADC Mix/Boost output to LMAIN left output
    /// mixer
    ///
    /// * `false` = path not connected
    /// * `true` = bypass path connected
    pub lbyplmx, lbyplmx_set: 1;
    /// Left DAC output to LMIX left output mixer path control
    ///
    /// * `false` = path disconnected (default)
    /// * `true` = path connected
    pub ldaclmx, ldaclmx_set: 0;
}

bitfield! {
    /// Right Mixer register contents
    ///
    /// See [`read_rightmixer`](crate::Codec::read_rightmixer),
    /// [`write_rightmixer`](crate::Codec::write_rightmixer) and
    /// [`modify_rightmixer`](crate::Codec::modify_rightmixer)
    pub struct RightMixer(u16);
    impl Debug;
    u8;
    /// Gain value between RAUXIN auxiliary input and input to RMAIN left output
    /// mixer
    ///
    /// * `0` = -15 dB (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub rauxmxgain, rauxmxgain_set: 8, 6;
    /// RAUXIN input to RMAIN right output mixer path control
    ///
    /// * `false` = RAUXIN not connected to RMAIN right output mixer (default)
    /// * `true` = RAUXIN connected to RMAIN right output mixer
    pub rauxrmx, rauxrmx_set: 5;
    /// Gain value for bypass from RADC Mix/Boost output to RMAIN left output
    /// mixer.
    ///
    /// * `0` = -15 dB (default)
    /// * `1` = -12 dB
    /// * `2` = -9.0 dB
    /// * `3` = -6.0 dB
    /// * `4` = -3.0 dB
    /// * `5` = 0.0 dB
    /// * `6` = +3.0 dB
    /// * `7` = +6.0 dB
    pub rbyprmxgain, rbyprmxgain_set: 4, 2;
    /// Right bypass path control from RADC Mix/Boost output to RMAIN r output
    /// mixer
    ///
    /// * `false` = path not connected
    /// * `true` = bypass path connected
    pub rbyprmx, rbyprmx_set: 1;
    /// Right DAC output to RMIX right output mixer path control
    ///
    /// * `false` = path disconnected (default)
    /// * `true` = path connected
    pub rdacrmx, rdacrmx_set: 0;
}

bitfield! {
    /// LHP Volume register contents
    ///
    /// See [`read_lhpvolume`](crate::Codec::read_lhpvolume),
    /// [`write_lhpvolume`](crate::Codec::write_lhpvolume) and
    /// [`modify_lhpvolume`](crate::Codec::modify_lhpvolume)
    pub struct LHPVolume(u16);
    impl Debug;
    u8;
    /// Headphone output volume update bit feature. Write-only bit for
    /// synchronized changes of left and right headphone amplifier output
    /// settings
    ///
    /// * If `false` on [`LHPVolume`] write, new [`LHPVolume`] value stored in
    ///   temporary register
    /// * If `true` on [`LHPVolume`] write, new [`LHPVolume`] and pending
    ///   [`RHPVolume`] values become active
    pub lhpvu, lhpvu_set: 8;
    /// Left channel input zero cross detection enable
    ///
    /// * `false` = gain changes to left headphone happen immediately (default)
    /// * `true` = gain changes to left headphone happen pending zero crossing
    ///   logic
    pub lhpzc, lhpzc_set: 7;
    /// Left headphone output mute control
    ///
    /// * `false` = headphone output not muted, normal operation (default)
    /// * `true` = headphone in muted condition not connected to LMIX output
    ///   stage
    pub lhpmute, lhpmute_set: 6;
    /// Left channel headphone output volume control setting. Setting becomes
    /// active when allowed by zero crossing and/or update bit features. `57` =
    /// 0.0 dB default setting.
    ///
    /// * `0` = -57 dB
    /// * `1` = -56 dB
    /// * *volume changes in 1.0 dB steps per binary bit value*
    /// * `62` = +5.0 dB
    /// * `63` = +6.0 dB
    pub lhpgain, lhpgain_set: 5, 0;
}

bitfield! {
    /// RHP Volume register contents
    ///
    /// See [`read_rhpvolume`](crate::Codec::read_rhpvolume),
    /// [`write_rhpvolume`](crate::Codec::write_rhpvolume) and
    /// [`modify_rhpvolume`](crate::Codec::modify_rhpvolume)
    pub struct RHPVolume(u16);
    impl Debug;
    u8;
    /// Headphone output volume update bit feature. Write-only bit for
    /// synchronized changes of left and right headphone amplifier output
    /// settings
    ///
    /// * If `false` on [`RHPVolume`] write, new [`RHPVolume`] value stored in
    ///   temporary register
    /// * If `true` on [`RHPVolume`] write, new [`RHPVolume`] and pending
    ///   [`LHPVolume`] values become active
    pub rhpvu, rhpvu_set: 8;
    /// Right channel input zero cross detection enable
    ///
    /// * `false` = gain changes to right headphone happen immediately (default)
    /// * `true` = gain changes to right headphone happen pending zero crossing
    ///   logic
    pub rhpzc, rhpzc_set: 7;
    /// Right headphone output mute control
    ///
    /// * `false` = headphone output not muted, normal operation (default)
    /// * `true` = headphone in muted condition not connected to RMIX output
    ///   stage
    pub rhpmute, rhpmute_set: 6;
    /// Right channel headphone output volume control setting. Setting becomes
    /// active when allowed by zero crossing and/or update bit features. `57` =
    /// 0.0 dB default setting.
    ///
    /// * `0` = -57 dB
    /// * `1` = -56 dB
    /// * *volume changes in 1.0 dB steps per binary bit value*
    /// * `62` = +5.0 dB
    /// * `63` = +6.0 dB
    pub rhpgain, rhpgain_set: 5, 0;
}

bitfield! {
    /// LSPKOUT Volume register contents
    ///
    /// See [`read_lspkoutvolume`](crate::Codec::read_lspkoutvolume),
    /// [`write_lspkoutvolume`](crate::Codec::write_lspkoutvolume) and
    /// [`modify_lspkoutvolume`](crate::Codec::modify_lspkoutvolume)
    pub struct LSPKOUTVolume(u16);
    impl Debug;
    u8;
    /// Loudspeaker output volume update bit feature. Write-only bit for
    /// synchronized changes of left and right headphone amplifier output
    /// settings
    ///
    /// * If `false` on [`LSPKOUTVolume`] write, new [`LSPKOUTVolume`] value
    ///   stored in temporary register
    /// * If `true` on [`LSPKOUTVolume`] write, new [`LSPKOUTVolume`] and
    ///   pending [`RSPKOUTVolume`] values become active
    pub lspkvu, lspkvu_set: 8;
    /// Left loudspeaker LSPKOUT output zero cross detection enable
    ///
    /// * `false` = gain changes to left loudspeaker happen immediately
    ///   (default)
    /// * `true` = gain changes to left loudspeaker happen pending zero crossing
    ///   logic
    pub lspkzc, lspkzc_set: 7;
    /// Right loudspeaker LSPKOUT output mute control
    ///
    /// * `false` = loudspeaker output not muted, normal operation (default)
    /// * `true` = loudspeaker in muted condition
    pub lspkmute, lspkmute_set: 6;
    /// Left loudspeaker output volume control setting. Setting becomes active
    /// when allowed by zero crossing and/or update bit features. `57` = 0.0 dB
    /// default setting.
    ///
    /// * `0` = -57 dB
    /// * `1` = -56 dB
    /// * *volume changes in 1.0 dB steps per binary bit value*
    /// * `62` = +5.0 dB
    /// * `63` = +6.0 dB
    pub lspkgain, lspkgain_set: 5, 0;


}

bitfield! {
    /// RSPKOUT Volume register contents
    ///
    /// See [`read_rspkoutvolume`](crate::Codec::read_rspkoutvolume),
    /// [`write_rspkoutvolume`](crate::Codec::write_rspkoutvolume) and
    /// [`modify_rspkoutvolume`](crate::Codec::modify_rspkoutvolume)
    pub struct RSPKOUTVolume(u16);
    impl Debug;
    u8;
    /// Loudspeaker output volume update bit feature. Write-only bit for
    /// synchronized changes of left and right headphone amplifier output
    /// settings
    ///
    /// * If `false` on [`RSPKOUTVolume`] write, new [`RSPKOUTVolume`] value
    ///   stored in temporary register
    /// * If `true` on [`RSPKOUTVolume`] write, new [`RSPKOUTVolume`] and
    ///   pending [`LSPKOUTVolume`] values become active
    pub rspkvu, rspkvu_set: 8;
    /// Right loudspeaker RSPKOUT output zero cross detection enable
    ///
    /// * `false` = gain changes to right loudspeaker happen immediately
    ///   (default)
    /// * `true` = gain changes to right loudspeaker happen pending zero
    ///   crossing logic
    pub rspkzc, rspkzc_set: 7;
    /// Right loudspeaker RSPKOUT output mute control
    ///
    /// * `false` = loudspeaker output not muted, normal operation (default)
    /// * `true` = loudspeaker in muted condition
    pub rspkmute, rspkmute_set: 6;
    /// Right loudspeaker output volume control setting. Setting becomes active
    /// when allowed by zero crossing and/or update bit features. `57` = 0.0 dB
    /// default setting.
    ///
    /// * `0` = -57 dB
    /// * `1` = -56 dB
    /// * *volume changes in 1.0 dB steps per binary bit value*
    /// * `62` = +5.0 dB
    /// * `63` = +6.0 dB
    pub rspkgain, rspkgain_set: 5, 0;
}

bitfield! {
    /// AUX2 Mixer register contents
    ///
    /// See [`read_aux2mixer`](crate::Codec::read_aux2mixer),
    /// [`write_aux2mixer`](crate::Codec::write_aux2mixer) and
    /// [`modify_aux2mixer`](crate::Codec::modify_aux2mixer)
    pub struct AUX2Mixer(u16);
    impl Debug;
    u8;
    /// AUXOUT2 output mute control
    ///
    /// * `false` = output not muted, normal operation (default)
    /// * `true` = output in muted condition
    pub auxout2mt, auxout2mt_set: 6;
    /// AUX1 Mixer output to AUX2 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub aux1mix2, aux1mix2_set: 3;
    /// Left LADC Mix/Boost output LINMIX path control to AUX2 MIXER input
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub ladcaux2, ladcaux2_set: 2;
    /// Left LMAIN MIXER output to AUX2 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub lmixaux2, lmixaux2_set: 1;
    /// Left DAC output to AUX2 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub ldacaux2, ldacaux2_set: 0;
}

bitfield! {
    /// AUX1 Mixer register contents
    ///
    /// See [`read_aux1mixer`](crate::Codec::read_aux1mixer),
    /// [`write_aux1mixer`](crate::Codec::write_aux1mixer) and
    /// [`modify_aux1mixer`](crate::Codec::modify_aux1mixer)
    pub struct AUX1Mixer(u16);
    impl Debug;
    u8;
    /// AUXOUT1 output mute control
    ///
    /// * `false` = output not muted, normal operation (default)
    /// * `true` = output in muted condition
    pub auxout1mt, auxout1mt_set: 6;
    /// AUXOUT1 6 dB attenuation enable
    ///
    /// * `false` = output signal at normal gain value (default)
    /// * `true` = output signal attenuated by 6.0 dB
    pub aux1half, aux1half_set: 5;
    /// Left LMAIN MIXER output to AUX1 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub lmixaux1, lmixaux1_set: 4;
    /// Left DAC output to AUX1 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub ldacaux1, ldacaux1_set: 3;
    /// Right RADC Mix/Boost output RINMIX path control to AUX1 MIXER input
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub radcaux1, radcaux1_set: 2;
    /// Right RMIX output to AUX1 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub rmixaux1, rmixaux1_set: 1;
    /// Right DAC output to AUX1 MIXER input path control
    ///
    /// * `false` = path not connected
    /// * `true` = path connected
    pub rdacaux1, rdacaux1_set: 0;
}

bitfield! {
    /// Power Management register contents
    ///
    /// See [`read_powermanagement`](crate::Codec::read_powermanagement),
    /// [`write_powermanagement`](crate::Codec::write_powermanagement) and
    /// [`modify_powermanagement`](crate::Codec::modify_powermanagement)
    pub struct PowerManagement(u16);
    impl Debug;
    u8;
    /// Reduce DAC supply current 50% in low power operating mode
    ///
    /// * `false` = normal supply current operation (default)
    /// * `true` = 50% reduced supply current mode
    pub lpdac, lpdac_set: 8;
    ///
    /// Reduce ADC Mix/Boost amplifier supply current 50% in low power operating
    /// mode
    ///
    /// * `false` = normal supply current operation (default)
    /// * `true` = 50% reduced supply current mode
    pub lpipbst, lpipbst_set: 7;
    /// Reduce ADC supply current 50% in low power operating mode
    ///
    /// * `false` = normal supply current operation (default)
    /// * `true` = 50% reduced supply current mode
    pub lpadc, lpadc_set: 6;
    /// Reduce loudspeaker amplifier supply current 50% in low power operating
    /// mode
    ///
    /// * `false` = normal supply current operation (default)
    /// * `true` = 50% reduced supply current mode
    pub lpspkd, lpspkd_set: 5;
    /// Microphone bias optional low noise mode configuration control
    ///
    /// * `false` = normal configuration with low-Z micbias output impedance
    /// * `true` = low noise configuration with 200 Ω micbias output impedance
    pub micbiasm, micbiasm_set: 4;
    /// Regulator voltage control power reduction options
    ///
    /// * `0` = normal 1.80Vdc operation (default)
    /// * `1` = 1.61Vdc operation
    /// * `2` = 1.40 Vdc operation
    /// * `3` = 1.218 Vdc operation
    pub regvolt, regvolt_set: 3, 2;
    /// Master bias current power reduction options
    ///
    /// * `0` = normal operation (default)
    /// * `1` = 25% reduced bias current from default
    /// * `2` = 14% reduced bias current from default
    /// * `3` = 25% reduced bias current from default
    pub ibadj, ibadj_set: 1, 0;
}

bitfield! {
    /// Left Time Slot register contents
    ///
    /// See [`read_lefttimeslot`](crate::Codec::read_lefttimeslot),
    /// [`write_lefttimeslot`](crate::Codec::write_lefttimeslot) and
    /// [`modify_lefttimeslot`](crate::Codec::modify_lefttimeslot)
    pub struct LeftTimeSlot(u16);
    impl Debug;
    u16;
    /// Left channel PCM time slot start count.
    ///
    /// LSB portion of total number of bit times to wait from frame sync before
    /// clocking audio channel data. LSB portion is combined with MSB from
    /// [`Misc`] to get total number of bit times to wait.
    pub ltslot, ltslot_set: 8, 0;
}

bitfield! {
    /// Misc register contents
    ///
    /// See [`read_misc`](crate::Codec::read_misc),
    /// [`write_misc`](crate::Codec::write_misc) and
    /// [`modify_misc`](crate::Codec::modify_misc)
    pub struct Misc(u16);
    impl Debug;
    u8;
    /// Time slot function enable for PCM mode.
    pub pcmtsen, pcmtsen_set: 8;
    /// Tri state ADC out after second half of LSB enable
    pub tri, tri_set: 7;
    /// 8-bit word length enable
    pub pcm8bit, pcm8bit_set: 6;
    /// ADCOUT output driver
    ///
    /// * `true` = enabled (default)
    /// * `false` = disabled (driver in high-z state)
    pub puden, puden_set: 5;
    /// ADCOUT passive resistor pull-up or passive pull-down enable
    ///
    /// * `false` = no passive pull-up or pull-down on ADCOUT pin
    /// * `true` = passive pull-up resistor on ADCOUT pin if PUDPS = 1
    /// * `true` = passive pull-down resistor on ADCOUT pin if PUDPS = 0
    pub pudpe, pudpe_set: 4;
    /// ADCOUT passive resistor pull-up or pull-down selection
    ///
    /// * `false` = passive pull-down resistor applied to ADCOUT pin if PUDPE =
    ///   1
    /// * `true` = passive pull-down resistor applied to ADCOUT pin if PUDPE = 1
    pub pudps, pudps_set: 3;
    /// Right channel PCM time slot start count.
    ///
    /// MSB portion of total number of bit times to wait from frame sync before
    /// clocking audio channel data. MSB is combined with LSB portion from R61
    /// ([`RightTimeSlot`]) to get total number of bit times to wait.
    pub rtslot9, rtslot9_set: 1;
    /// Left channel PCM time slot start count.
    ///
    /// MSB portion of total number of bit times to wait from frame sync before
    /// clocking audio channel data. MSB is combined with LSB portion from R59
    /// ([`LeftTimeSlot`]) to get total number of bit times to wait.
    pub ltslot9, ltslot9_set: 0;
}

bitfield! {
    /// Right Time Slot register contents
    ///
    /// See [`read_righttimeslot`](crate::Codec::read_righttimeslot),
    /// [`write_righttimeslot`](crate::Codec::write_righttimeslot) and
    /// [`modify_righttimeslot`](crate::Codec::modify_righttimeslot)
    pub struct RightTimeSlot(u16);
    impl Debug;
    u16;
    /// Right channel PCM time slot start count.
    ///
    /// LSB portion of total number of bit times to wait from frame sync before
    /// clocking audio channel data. LSB portion is combined with MSB from
    /// [`Misc`] to get total number of bit times to wait.
    pub rtslot, rtslot_set: 8, 0;
}

bitfield! {
    /// Device Revision register contents
    ///
    /// See [`read_devicerevisionno`](crate::Codec::read_devicerevisionno),
    /// [`write_devicerevisionno`](crate::Codec::write_devicerevisionno) and
    /// [`modify_devicerevisionno`](crate::Codec::modify_devicerevisionno)
    pub struct DeviceRevisionNo(u16);
    impl Debug;
    u8;
    /// Chip Revision.
    ///
    /// * Will be `0x7F` for Revision A.
    pub revision, _: 7, 0;
}

bitfield! {
    /// Device ID register contents
    ///
    /// See [`read_deviceid`](crate::Codec::read_deviceid),
    /// [`write_deviceid`](crate::Codec::write_deviceid) and
    /// [`modify_deviceid`](crate::Codec::modify_deviceid)
    pub struct DeviceId(u16);
    impl Debug;
    u16;
    /// Device ID.
    ///
    /// Will be `0x01A`
    pub device_id, _: 8, 0;
}

bitfield! {
    /// DAC Dither
    ///
    /// See [`read_dacdither`](crate::Codec::read_dacdither),
    /// [`write_dacdither`](crate::Codec::write_dacdither) and
    /// [`modify_dacdither`](crate::Codec::modify_dacdither)
    pub struct DacDither(u16);
    impl Debug;
    u8;
    /// Dither added to DAC modulator to eliminate all non-random noise
    ///
    /// * `0` = dither off
    /// * `17` = nominal optimal dither
    /// * `31` = maximum dither
    pub mod_dither, mod_dither_set: 8, 4;
    /// Dither added to DAC analog output to eliminate all non-random noise
    ///
    /// * `0` = dither off
    /// * `8` = nominal optimal dither
    /// * `15` = maximum dither
    pub analog_dither, analog_dither_set: 3, 0;
}

bitfield! {
    /// ALC Enhancements 1 register contents
    ///
    /// See [`read_alcenhancements1`](crate::Codec::read_alcenhancements1),
    /// [`write_alcenhancements1`](crate::Codec::write_alcenhancements1) and
    /// [`modify_alcenhancements1`](crate::Codec::modify_alcenhancements1)
    pub struct AlcEnhancements1(u16);
    impl Debug;
    u8;
    /// Selects one of two tables used to set the target level for the ALC
    ///
    /// * `false` = default recommended target level table spanning -1.5 dB
    /// through -22.5 dB FS
    /// * `true` = optional ALC target level table spanning -6.0 dB through
    ///   -28.5 dB FS
    pub alctblsel, alctblsel_set: 8;
    /// Choose peak or peak-to-peak value for ALC threshold logic
    ///
    /// * `false` = use rectified peak detector output value
    /// * `true` = use peak-to-peak detector output value
    pub alcpksel, alcpksel_set: 7;
    /// Choose peak or peak-to-peak value for Noise Gate threshold logic
    ///
    /// * `false` = use rectified peak detector output value
    /// * `true` = use peak-to-peak detector output value
    pub alcngsel, alcngsel_set: 6;
    /// Real time readout of instantaneous gain value used by left channel PGA
    pub alcgainl, _: 5, 0;
}

bitfield! {
    /// ALC Enhancements 2 register contents
    ///
    /// See [`read_alcenhancements2`](crate::Codec::read_alcenhancements2),
    /// [`write_alcenhancements2`](crate::Codec::write_alcenhancements2) and
    /// [`modify_alcenhancements2`](crate::Codec::modify_alcenhancements2)
    pub struct AlcEnhancements2(u16);
    impl Debug;
    u8;
    /// Enable control for ALC fast peak limiter function
    ///
    /// * `false` = enabled (default)
    /// * `true` = disabled
    pub pklimena, pklimena_set: 8;
    /// Real time readout of instantaneous gain value used by right channel PGA
    pub alcgainl, _: 5, 0;
}

bitfield! {
    /// Misc Controls register contents
    ///
    /// See [`read_misccontrols`](crate::Codec::read_misccontrols),
    /// [`write_misccontrols`](crate::Codec::write_misccontrols) and
    /// [`modify_misccontrols`](crate::Codec::modify_misccontrols)
    pub struct MiscControls(u16);
    impl Debug;
    u8;
    /// Set SPI control bus mode regardless of state of Mode pin
    ///
    /// * `false` = normal operation (default)
    /// * `true` = force SPI 4-wire mode regardless of state of Mode pin
    pub fwspiena, fwspiena_set: 8;
    /// Short frame sync detection period value
    ///
    /// * `0` = trigger if frame time less than 255 MCLK edges
    /// * `1` = trigger if frame time less than 253 MCLK edges
    /// * `2` = trigger if frame time less than 254 MCLK edges
    /// * `3` = trigger if frame time less than 255 MCLK edges
    pub fserrval, fserrval_set: 7, 6;
    /// Enable DSP state flush on short frame sync event
    ///
    /// * `false` = ignore short frame sync events (default)
    /// * `true` = set DSP state to initial conditions on short frame sync event
    pub fserflsh, fserflsh_set: 5;
    /// Enable control for short frame cycle detection logic
    ///
    /// * `false` = short frame cycle detection logic enabled
    /// * `true` = short frame cycle detection logic disabled
    pub fserrena, fserrena_set: 4;
    /// Enable control to delay use of notch filter output when filter is
    /// enabled
    ///
    /// * `false` = delay using notch filter output 512 sample times after notch
    ///   enabled (default)
    /// * `true` = use notch filter output immediately after notch filter is
    ///   enabled
    pub notchdly, notchdly_set: 3;
    /// Enable control to mute DAC limiter output when softmute is enabled
    ///
    /// * `false` = DAC limiter output may not move to exactly zero during
    ///   Softmute (default)
    /// * `true` = DAC limiter output muted to exactly zero during softmute
    pub dacinmute, dacinmute_set: 2;
    /// Enable control to use PLL output when PLL is not in phase locked
    /// condition
    ///
    /// * `false` = PLL VCO output disabled when PLL is in unlocked condition
    ///   (default)
    /// * `true` = PLL VCO output used as-is when PLL is in unlocked condition
    pub plllockbp, plllockbp_set: 1;
    /// Set DAC to use 256x oversampling rate (best at lower sample rates)
    ///
    /// * `false` = Use oversampling rate as determined by Register 0x0A[3]
    ///   (default)
    /// * `true` = Set DAC to 256x oversampling rate regardless of Register
    ///   0x0A[3]
    pub dacosr256, dacosr256_set: 0;
}

bitfield! {
    /// Tie-Off Overrides register contents
    ///
    /// See [`read_tieoffoverrides`](crate::Codec::read_tieoffoverrides),
    /// [`write_tieoffoverrides`](crate::Codec::write_tieoffoverrides) and
    /// [`modify_tieoffoverrides`](crate::Codec::modify_tieoffoverrides)
    pub struct TieOffOverrides(u16);
    impl Debug;
    u8;
    /// Enable direct control over input tie-off resistor switching
    ///
    /// * `false` = ignore Register 0x4A bits to control input tie-off resistor
    ///   switching
    /// * `true` = use Register 0x4A bits to override automatic tie-off resistor
    ///   switching
    pub maninena, maninena_set: 8;
    /// If MANUINEN = 1, use this bit to control right aux input tie-off
    /// resistor switch
    ///
    /// * `false` = Tie-off resistor switch for RAUXIN input is forced open
    /// * `true` = Tie-off resistor switch for RAUXIN input is forced closed
    pub manraux, manraux_set: 7;
    /// If MANUINEN = 1, use this bit to control right line input tie-off
    /// resistor switch
    ///
    /// * `false` = Tie-off resistor switch for RLIN input is forced open
    /// * `true` = Tie-off resistor switch for RLIN input is forced closed
    pub manrlin, manrlin_set: 6;
    /// If MANUINEN = 1, use this bit to control right PGA inverting input
    /// tie-off switch
    ///
    /// * `false` = Tie-off resistor switch for RMICN input is forced open
    /// * `true` = Tie-off resistor switch for RMICN input is forced closed
    pub manrmicn, manrmicn_set: 5;
    /// If MANUINEN =1, use this bit to control right PGA non-inverting input
    /// tie-off switch
    ///
    /// * `false` = Tie-off resistor switch for RMICP input is forced open
    /// * `true` = Tie-off resistor switch for RMICP input is forced closed
    pub manrmicp, manrmicp_set: 4;
    /// If MANUINEN = 1, use this bit to control left aux input tie-off resistor
    /// switch
    ///
    /// * `false` = Tie-off resistor switch for LAUXIN input is forced open
    /// * `true` = Tie-off resistor switch for RAUXIN input is forced closed
    pub manlaux, manlaux_set: 3;
    /// If MANUINEN = 1, use this bit to control left line input tie-off
    /// resistor switch
    ///
    /// * `false` = Tie-off resistor switch for LLIN input is forced open
    /// * `true` = Tie-off resistor switch for LLIN input is forced closed
    pub manllin, manllin_set: 2;
    /// If MANUINEN = 1, use this bit to control left PGA inverting input
    /// tie-off switch
    ///
    /// * `false` = Tie-off resistor switch for LMICN input is forced open
    /// * `true` = Tie-off resistor switch for LMINN input is forced closed
    pub manlmicn, manlmicn_set: 1;
    /// If MANUINEN = 1, use this bit to control left PGA non-inverting input
    /// tie-off switch
    ///
    /// * `false` = Tie-off resistor switch for LMICP input is forced open
    /// * `true` = Tie-off resistor switch for LMICP input is forced closed
    pub manlmicp, manlmicp_set: 0;
}

bitfield! {
    /// Power/Tie-off Ctrl register contents
    ///
    /// See [`read_powertieoffctrl`](crate::Codec::read_powertieoffctrl),
    /// [`write_powertieoffctrl`](crate::Codec::write_powertieoffctrl) and
    /// [`modify_powertieoffctrl`](crate::Codec::modify_powertieoffctrl)
    pub struct PowerTieOffCtrl(u16);
    impl Debug;
    u8;
    /// Reduce bias current to left and right input MIX/BOOST stage
    ///
    /// * `false` = normal bias current
    /// * `true` = bias current reduced by 50% for reduced power and bandwidth
    pub ibthalfi, ibthalfi_set: 8;
    /// Increase bias current to left and right input MIX/BOOST stage
    ///
    /// * `false` = normal bias current
    /// * `true` = bias current increased by 500 µA
    pub ibt500up, ibt500up_set: 6;
    /// Decrease bias current to left and right input MIX/BOOST stage
    ///
    /// * `false` = normal bias current
    /// * `true` = bias current reduced by 250 µA
    pub ibt250dn, ibt250dn_set: 5;
    /// Direct manual control to turn on bypass switch around input tie-off
    /// buffer amplifier
    ///
    /// * `false` = normal automatic operation of bypass switch
    /// * `true` = bypass switch in closed position when input buffer amplifier
    ///   is disabled
    pub maninbbp, maninbbp_set: 4;
    /// Direct manual control to turn on switch to ground at input tie-off
    /// buffer amp output
    ///
    /// * `false` = normal automatic operation of switch to ground
    /// * `true` = switch to ground in in closed position when input buffer
    ///   amplifier is disabled
    pub maninpad, maninpad_set: 3;
    /// Direct manual control of switch for Vref 600 kΩ resistor to ground
    ///
    /// * `false` = switch to ground controlled by Register 0x01 setting
    /// * `true` = switch to ground in the closed position
    pub manvrefh, manvrefh_set: 2;
    /// Direct manual control for switch for Vref 160 kΩ resistor to ground
    ///
    /// * `false` = switch to ground controlled by Register 0x01 setting
    /// * `true` = switch to ground in the closed position
    pub manvrefm, manvrefm_set: 1;
    /// Direct manual control for switch for Vref 6 kΩ resistor to ground
    ///
    /// * `false` = switch to ground controlled by Register 0x01 setting
    /// * `true` = switch to ground in the closed position
    pub manvrefl, manvrefl_set: 0;
}

bitfield! {
    /// P2P Detector Read register contents
    ///
    /// See [`read_p2pdetectorread`](crate::Codec::read_p2pdetectorread),
    /// [`write_p2pdetectorread`](crate::Codec::write_p2pdetectorread) and
    /// [`modify_p2pdetectorread`](crate::Codec::modify_p2pdetectorread)
    pub struct P2PDetectorRead(u16);
    impl Debug;
    u16;
    /// Read-only register which outputs the instantaneous value contained in
    /// the peak-to-peak amplitude register used by the ALC for signal level
    /// dependent logic. Value is highest of left or right input when both
    /// inputs are under ALC control.
    pub p2pval, _: 8, 0;
}

bitfield! {
    /// Peak Detector Read register contents
    ///
    /// See [`read_peakdetectorread`](crate::Codec::read_peakdetectorread),
    /// [`write_peakdetectorread`](crate::Codec::write_peakdetectorread) and
    /// [`modify_peakdetectorread`](crate::Codec::modify_peakdetectorread)
    pub struct PeakDetectorRead(u16);
    impl Debug;
    u8;
    /// Read-only register which outputs the instantaneous value contained in
    /// the peak detector amplitude register used by the ALC for signal level
    /// dependent logic. Value is highest of left or right input when both
    /// inputs are under ALC control.
    pub peakval, _: 8, 0;
}

bitfield! {
    /// Control and Status register contents
    ///
    /// See [`read_controlandstatus`](crate::Codec::read_controlandstatus),
    /// [`write_controlandstatus`](crate::Codec::write_controlandstatus) and
    /// [`modify_controlandstatus`](crate::Codec::modify_controlandstatus)
    pub struct ControlAndStatus(u16);
    impl Debug;
    u8;
    /// Select observation point used by DAC output automute feature
    ///
    /// * `false` = automute operates on data at the input to the DAC digital
    ///   attenuator (default)
    /// * `true` = automute operates on data at the DACIN input pin
    pub amutctrl, amutctrl_set: 5;
    /// Read-only status bit of high voltage detection circuit monitoring VDDSPK
    /// voltage
    ///
    /// * `false` = voltage on VDDSPK pin measured at approximately 4.0Vdc or
    ///   less
    /// * `true` = voltage on VDDSPK pin measured at approximately 4.0Vdc or
    ///   greater
    pub hvdet, _: 4;
    /// Read-only status bit of logic controlling the noise gate function
    ///
    /// * `false` = signal is greater than the noise gate threshold and ALC gain
    ///   can change
    /// * `true` = signal is less than the noise gate threshold and ALC gain is
    ///   held constant
    pub nsgate, _: 3;
    /// Read-only status bit of analog mute function applied to DAC channels
    ///
    /// * `false` = not in the automute condition
    /// * `true` = in automute condition
    pub anamute, _: 2;
    /// Read-only status bit of digital mute function of the left channel DAC
    ///
    /// * `false` = digital gain value is greater than zero
    /// * `true` = digital gain is zero either by direct setting or operation of
    ///   softmute function
    pub digmutel, _: 1;
    /// Read-only status bit of digital mute function of the left channel DAC
    ///
    /// * `false` = digital gain value is greater than zero
    /// * `true` = digital gain is zero either by direct setting or operation of
    ///   softmute function
    pub digmuter, _: 0;
}

bitfield! {
    /// Output tie-off control register contents
    ///
    /// See
    /// [`read_outputtieoffcontrol`](crate::Codec::read_outputtieoffcontrol),
    /// [`write_outputtieoffcontrol`](crate::Codec::write_outputtieoffcontrol)
    /// and
    /// [`modify_outputtieoffcontrol`](crate::Codec::modify_outputtieoffcontrol)
    pub struct OutputTieOffControl(u16);
    impl Debug;
    u8;
    /// Enable direct control over output tie-off resistor switching
    ///
    /// * `false` = ignore Register 0x4F bits to control input tie-off
    ///   resistor/buffer switching
    /// * `true` = use Register 0x4F bits to override automatic tie-off
    ///   resistor/buffer switching
    pub manouten, manouten_set: 8;
    /// If `manouten` = `true`, use this bit to control bypass switch around
    /// 1.5x boosted output tie-off buffer amplifier
    ///
    /// * `false` = normal automatic operation of bypass switch
    /// * `true` = bypass switch in closed position when output buffer amplifier
    ///   is disabled
    pub shrtbufh, shrtbufh_set: 7;
    /// If `manouten` = `true`, use this bit to control bypass switch around
    /// 1.0x non-boosted output tie-off buffer amplifier
    ///
    /// * `false` = normal automatic operation of bypass switch
    /// * `true` = bypass switch in closed position when output buffer amplifier
    ///   is disabled
    pub shrtbufl, shrtbufl_set: 6;
    /// If `manouten` = `true`, use this bit to control left speaker output
    /// tie-off resistor switch
    ///
    /// * `false` = tie-off resistor switch for LSPKOUT speaker output is forced
    ///   open
    /// * `true` = tie-off resistor switch for LSPKOUT speaker output is forced
    ///   closed
    pub shrtlspk, shrtlspk_set: 5;
    /// If `manouten` = `true`, use this bit to control left speaker output
    /// tie-off resistor switch
    ///
    /// * `false` = tie-off resistor switch for RSPKOUT speaker output is forced
    ///   open
    /// * `true` = tie-off resistor switch for RSPKOUT speaker output is forced
    ///   closed
    pub shrtrspk, shrtrspk_set: 4;
    /// If `manouten` = `true`, use this bit to control Auxout1 output tie-off
    /// resistor switch
    ///
    /// * `false` = tie-off resistor switch for AUXOUT1 output is forced open
    /// * `true` = tie-off resistor switch for AUXOUT1 output is forced closed
    pub shrtaux1, shrtaux1_set: 3;
    /// If `manouten` = `true`, use this bit to control Auxout2 output tie-off
    /// resistor switch
    ///
    /// * `false` = tie-off resistor switch for AUXOUT2 output is forced open
    /// * `true` = tie-off resistor switch for AUXOUT2 output is forced closed
    pub shrtaux2, shrtaux2_set: 2;
    /// If `manouten` = `true`, use this bit to control left headphone output
    /// tie-off switch
    ///
    /// * `false` = tie-off resistor switch for LHP output is forced open
    /// * `true` = tie-off resistor switch for LHP output is forced closed
    pub shrtlhp, shrtlhp_set: 1;
    /// If `manouten` = `true`, use this bit to control right headphone output
    /// tie-off switch
    ///
    /// * `false` = tie-off resistor switch for RHP output is forced open
    /// * `true` = tie-off resistor switch for RHP output is forced closed
    pub shrtrhp, shrtrhp_set: 0;
}

// End of file
