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

    #[error("Invalid sample rate: {0} Hz (Valid range: 225 kSPS to 3.2 MSPS)")]
    InvalidSampleRate(u32),

    #[error("Device not initialized. Call new() first.")]
    NotInitialized,

    #[error("Device not found")]
    NotFound,

    #[error("Buffer accessed after drop")]
    BufferEmpty,

    #[error("Channel is full")]
    ChannelFull,

    #[error("Channel closed unexpectedly")]
    ChannelClosed,

    #[error("Mutex poisoned: {0}")]
    MutexPoisoned(String),

    #[error("Invalid gain value: {0}")]
    InvalidGain(i32),

    #[error("Hardware command failed: {0}")]
    HardwareCommand(String),
}

pub type Result<T> = std::result::Result<T, Error>;
// The following variants are used by stream.rs, r828d.rs, and websdr server.
