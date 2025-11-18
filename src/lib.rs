//! DHT22 humidity and temperature sensor driver for the RP2040 microcontroller.
//! Based on rp2040_hal and embedded_hal
//!
//! Communicates though a single data wire which requires special pin handling
//! for bi-directional communication.
//!
//! Reference:
//! <https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf>
//!
//!
//! Example:
//!
//! ```rust
//! 
//! use lh_dht22_rp as dht22;
//! ...
//!
//! // Hal Boilerplate ...
//! ...
//!
//! let dht_pin = pins.gpio1;
//! let mut dht = dht22::DHT22::new(dht_pin, timer);
//!
//! match dht.read() {
//!     Ok((humidity, temperature)) => {
//!            println!("Humidity   : {:.1} %RH", humidity);
//!            println!("Temperature: {:.1} C", temperature); },
//!     Err(e) => println!("Err: {e}"),
//!     }
//! ```

#![no_std]

use rp2040_hal::gpio;
use rp2040_hal::timer::{Instant, Timer};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_0_2::blocking::delay::DelayUs;

// —————————————————————————————————————————————————————————————————————————————————————————————————
//                                             Globals
// —————————————————————————————————————————————————————————————————————————————————————————————————

const TIMEOUT_MS: u64 = 2 * 1000;
const INIT_STATE_CHECK_DELAY_US: i32 = 100;
const START_SIGNAL_US: u32 = 5_000;
const SETUP_TIME_US: u32 = 20;
const HIGH: u8 = 1;
const LOW: u8 = 0;

type Output = gpio::Pin<gpio::DynPinId, gpio::FunctionSio<gpio::SioOutput>, gpio::PullUp>;
type Input = gpio::Pin<gpio::DynPinId, gpio::FunctionSio<gpio::SioInput>, gpio::PullNone>;

pub type Result<T> = core::result::Result<T, DhtError>;

// —————————————————————————————————————————————————————————————————————————————————————————————————
//                                              DHT22
// —————————————————————————————————————————————————————————————————————————————————————————————————

pub struct DHT22 {
    pin:        Output,
    timer:      Timer,
    start_time: Instant,
}

impl DHT22 {
    /// Creates a new DHT22 sensor instance.
    /// Requires the Pin connected to the DHT22 Data line, and a copy of the mcu timer.
    pub fn new(pin: impl gpio::AnyPin, timer: Timer) -> Self {
        let mut pin = pin.into_output();
        pin.set_high();

        let start_time = timer.get_counter();

        Self { pin, timer, start_time }
    }

    #[inline]
    /// Internal check for time out
    fn not_timed_out(&self) -> Result<()> {
        let elapsed = self
            .timer
            .get_counter()
            .checked_duration_since(self.start_time)
            .unwrap()
            .to_millis();

        if elapsed > TIMEOUT_MS {
            return Err(DhtError::Timeout);
        }
        Ok(())
    }

    #[inline]
    /// Waits until the desired state is read. Errors on timeout
    fn wait_for_state(&mut self, state: u8, pin: &mut Input) -> Result<()> {
        loop {
            if get_input_state(pin) == state {
                return Ok(());
            }

            if self.not_timed_out().is_err() {
                return Err(DhtError::Timeout);
            }
        }
    }

    /// Reads the data from the sensor.
    /// Returns Ok((humidity, temperature)) or Err(DhtError)
    pub fn read(&mut self) -> Result<(f32, f32)> {
        const PACKET_SIZE: usize = 40;
        //
        self.start_time = self.timer.get_counter();

        // DTH22 sends a 16b + 16b + 8b package
        let mut buffer = [0u8; PACKET_SIZE / 8];

        // Requesting Data
        let mut pin = self.pin.into_output();
        pin.set_low();
        self.timer.delay_us(START_SIGNAL_US);
        pin.set_high();
        self.timer.delay_us(SETUP_TIME_US);

        // Switching pin into Input type
        let mut pin = pin.into_input();

        // Critical Section Interrupt Free - for time sensitive ops
        let transaction_result = critical_section::with(|_cs| {
            // Receiving Prelude - Expecting the pin to be HIGH at this time
            self.timer.delay_us(INIT_STATE_CHECK_DELAY_US); // This may vary so it could be improved
            if get_input_state(&mut pin) == LOW {
                return Err(DhtError::Communication);
            }

            // Waiting for data transmission to start
            self.wait_for_state(LOW, &mut pin)?;

            // Reading Data
            for i in 0..PACKET_SIZE {
                // Waiting for Bit tx signaled by HIGH state
                self.wait_for_state(HIGH, &mut pin)?;

                // Reading bit value
                self.timer.delay_us(35);
                let state = get_input_state(&mut pin);

                // Adding bit to buffer
                let byte_index = i / 8;
                let bit_index = 7 - (i % 8);
                if state == 1 {
                    buffer[byte_index] |= 1 << bit_index;
                }

                // Wait until bit finished sending
                if state == HIGH {
                    self.wait_for_state(LOW, &mut pin)?;
                }
            }

            Ok(())
        });

        // Resetting pin state
        let mut pin = self.pin.into_output();
        pin.set_high();

        // Evaluating transaction result
        transaction_result?;

        // Compute Checksum
        let checksum = buffer[4];
        let checksum_truth = buffer[0]
            .wrapping_add(buffer[1])
            .wrapping_add(buffer[2])
            .wrapping_add(buffer[3]);

        // If all received bits are 1
        if checksum_truth == 252 {
            return Err(DhtError::Connection);
        }

        if checksum != checksum_truth {
            return Err(DhtError::Checksum);
        }

        // Compute Humidity
        let humidity = u16::from_be_bytes([buffer[0], buffer[1]]);
        let humidity = humidity as f32 * 0.1;

        // Compute Temperature
        let temperature = u16::from_be_bytes([buffer[2], buffer[3]]);

        // Negative if highest bit is 1
        let temperature = if temperature >> 15 == 1 {
            (temperature & !(1 << 15)) as f32 * -0.1
        }
        else {
            temperature as f32 * 0.1
        };

        Ok((humidity, temperature))
    }
}

// —————————————————————————————————————————————————————————————————————————————————————————————————
//                                              Error
// —————————————————————————————————————————————————————————————————————————————————————————————————

#[derive(Debug, Clone, Eq, PartialEq)]
pub enum DhtError {
    Timeout,
    Checksum,
    Communication,
    Connection,
}

impl core::fmt::Display for DhtError {
    fn fmt(
        &self,
        fmt: &mut core::fmt::Formatter<'_>,
    ) -> core::result::Result<(), core::fmt::Error> {
        match self {
            DhtError::Timeout => write!(fmt, "timeout"),
            DhtError::Checksum => write!(fmt, "invalid data"),
            DhtError::Communication => write!(fmt, "communication error"),
            DhtError::Connection => write!(fmt, "connection error"),
        }
    }
}

// —————————————————————————————————————————————————————————————————————————————————————————————————
//                                             Traits
// —————————————————————————————————————————————————————————————————————————————————————————————————

/// Trait for constructing a dynamic input or output pin from scratch
#[allow(clippy::wrong_self_convention)]
pub trait ReconstructPin {
    fn into_output(&self) -> Output;
    fn into_input(&self) -> Input;
}

impl<T: gpio::AnyPin> ReconstructPin for T {
    #[inline]
    /// Returns a dynamic output pin
    fn into_output(&self) -> Output {
        let id = self.borrow().id().num;
        // HAL is pedantic with self-creating pins, but we have ownership over the original pin
        unsafe {
            let pin = gpio::new_pin(gpio::DynPinId {
                bank: gpio::DynBankId::Bank0,
                num:  id,
            });

            pin.try_into_function::<gpio::FunctionSio<gpio::SioOutput>>()
                .expect("Pin into Output")
                .into_pull_type::<gpio::PullUp>()
        }
    }

    #[inline]
    /// Returns a dynamic input pin
    fn into_input(&self) -> Input {
        let id = self.borrow().id().num;
        // HAL is pedantic with self-creating pins, but we have ownership over the original pin
        unsafe {
            let pin = gpio::new_pin(gpio::DynPinId {
                bank: gpio::DynBankId::Bank0,
                num:  id,
            });

            pin.try_into_function::<gpio::FunctionSio<gpio::SioInput>>()
                .expect("Pin into Input")
                .into_pull_type::<gpio::PullNone>()
        }
    }
}

// —————————————————————————————————————————————————————————————————————————————————————————————————
//                                         Functions
// —————————————————————————————————————————————————————————————————————————————————————————————————

#[inline]
fn get_input_state(pin: &mut Input) -> u8 {
    if pin.is_high().unwrap() {
        return HIGH;
    }
    LOW
}
