use thiserror::Error;
use rusb;

#[derive(Error, Debug)]
pub enum Error {
    #[error("USB error: {0}")]
    Usb(#[from] rusb::Error),

    #[error("I2C error: address {addr:02x} failed")]
    I2c { addr: u8 },

    #[error("Tuner error: {0}")]
    Tuner(String),

    #[error("Unsupported tuner: {0}")]
    UnsupportedTuner(String),

    #[error("Invalid frequency: {0} Hz")]
    InvalidFrequency(u64),

    #[error("Timeout occurred during operation")]
    Timeout,

    #[error("Device not found")]
    NotFound,
}

pub type Result<T> = std::result::Result<T, Error>;
