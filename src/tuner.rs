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
    pub end_hz:   u64,
}

/// Board-level configuration injected into the `Driver` orchestrator.
/// The tuner chip itself (`R828D`, `R820T`, etc.) never sees this — it stays
/// as a pure chip driver. The `Driver` uses `BoardConfig` to decide which
/// GPIO and notch-filter operations to perform after the chip is tuned.
#[derive(Debug, Clone)]
pub enum BoardConfig {
    /// Generic RTL-SDR dongle — no special GPIO or triplexer logic.
    Generic,
    /// RTL-SDR Blog V4 — R828D chip + external triplexer on GPIO 5
    /// + dynamic notch filters for FM/AM bands.
    BlogV4,
}

impl BoardConfig {
    /// Returns true if the given frequency falls within a notch-filtered band
    /// on this board. Used by the orchestrator to set `open_d` in `set_mux`.
    pub fn in_notch_band(&self, freq_hz: u64) -> bool {
        match self {
            BoardConfig::Generic => false,
            BoardConfig::BlogV4 => {
                freq_hz <= 2_200_000
                    || (freq_hz >= 85_000_000  && freq_hz <= 112_000_000)
                    || (freq_hz >= 172_000_000 && freq_hz <= 242_000_000)
            }
        }
    }

    /// Which triplexer input path to select on the V4, returned as
    /// `(gpio5_high, cable2_active, cable1_active, air_in_active)`.
    /// Returns `None` for Generic boards (no GPIO needed).
    pub fn input_path(&self, freq_hz: u64) -> Option<InputPath> {
        match self {
            BoardConfig::Generic => None,
            BoardConfig::BlogV4 => {
                if freq_hz < 28_800_000 {
                    Some(InputPath::Hf)   // HF: cable 2, GPIO5 low
                } else if freq_hz < 250_000_000 {
                    Some(InputPath::Vhf)  // VHF: cable 1, GPIO5 high
                } else {
                    Some(InputPath::Uhf)  // UHF: air in, GPIO5 high
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum InputPath { Hf, Vhf, Uhf }

pub trait Tuner: Send + Sync {
    /// Initialize the tuner chip with default registers.
    fn initialize(&self) -> Result<()>;

    /// Set the center frequency in Hz. Returns the actual frequency set.
    /// The chip driver handles PLL and MUX only — no GPIO, no board logic.
    fn set_frequency(&self, hz: u64) -> Result<u64>;

    /// Set the gain in dB. Returns the actual gain set.
    fn set_gain(&self, db: f32) -> Result<f32>;

    /// Get the current gain in dB.
    fn get_gain(&self) -> Result<f32>;

    /// Get the supported filter ranges.
    fn get_filters(&self) -> Vec<FilterRange>;

    /// Set the IF frequency in Hz.
    fn set_if_freq(&self, hz: u64) -> Result<()>;

    /// Get the current IF frequency in Hz.
    fn get_if_freq(&self) -> u64;

    /// Set the crystal PPM correction.
    fn set_ppm(&self, ppm: i32) -> Result<()>;

    /// Set the internal input mux path (e.g. for V4 triplexer).
    /// Default: no-op.
    fn set_input_path(&self, _path: InputPath) -> Result<()> { Ok(()) }

    /// Apply board-level notch filter hint. Called by the orchestrator
    /// after `set_frequency` so the chip can adjust `open_d` in `set_mux`
    /// without knowing anything about the board it's sitting on.
    /// Default: no-op (Generic boards, future tuners).
    fn apply_notch(&self, _in_notch_band: bool) -> Result<()> { Ok(()) }
}
