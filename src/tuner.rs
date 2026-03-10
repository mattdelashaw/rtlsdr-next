use crate::error::Result;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TunerType {
    R820T,
    R828D,
    E4000,
    FC0012,
    FC0013,
    Unknown(u8),
}

pub struct FilterRange {
    pub start_hz: u64,
    pub end_hz: u64,
}

pub trait Tuner: Send + Sync {
    /// Initialize the tuner chip with default registers.
    fn initialize(&self) -> Result<()>;
    
    /// Set the center frequency in Hz. Returns the actual frequency set.
    fn set_frequency(&self, hz: u64) -> Result<u64>;
    
    /// Set the gain in dB. Returns the actual gain set.
    fn set_gain(&self, db: f32) -> Result<f32>;

    /// Get the current gain in dB.
    fn get_gain(&self) -> Result<f32>;
    
    /// Get the supported filter ranges (useful for V4 Triplexer).
    fn get_filters(&self) -> Vec<FilterRange>;
    
    /// Toggle the Bias-T power.
    fn set_bias_t(&self, on: bool) -> Result<()>;

    /// Set the crystal frequency correction in Parts Per Million (PPM).
    fn set_ppm(&self, ppm: i32) -> Result<()>;
}