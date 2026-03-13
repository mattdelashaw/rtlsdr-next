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

/// A calculated plan for tuning to a specific frequency on a specific board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TuningPlan {
    /// The actual frequency to request from the tuner chip.
    pub tuner_hz: u64,
    /// Whether the spectrum needs to be inverted (e.g., for V4 HF path).
    pub spectral_inv: bool,
    /// The triplexer input path to select (if any).
    pub input_path: Option<InputPath>,
    /// Whether the frequency falls within a board-level notch filter band.
    pub in_notch: bool,
}

/// Board-level configuration and logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardConfig {
    /// Generic RTL-SDR dongle — no special GPIO or triplexer logic.
    Generic,
    /// RTL-SDR Blog V4 — R828D chip + external triplexer on GPIO 5
    /// + dynamic notch filters for FM/AM bands.
    BlogV4,
}

pub trait BoardOrchestrator: Send + Sync {
    /// Calculate the tuning plan for a requested frequency.
    fn plan_tuning(&self, hz: u64) -> TuningPlan;
}

pub struct GenericOrchestrator;
impl BoardOrchestrator for GenericOrchestrator {
    fn plan_tuning(&self, hz: u64) -> TuningPlan {
        TuningPlan {
            tuner_hz: hz,
            spectral_inv: false,
            input_path: None,
            in_notch: false,
        }
    }
}

pub struct V4Orchestrator;
impl BoardOrchestrator for V4Orchestrator {
    fn plan_tuning(&self, hz: u64) -> TuningPlan {
        let mut tuner_hz = hz;
        let mut spectral_inv = false;

        // V4 HF Upconverter (28.8 MHz offset)
        if hz < 28_800_000 {
            tuner_hz += 28_800_000;
            spectral_inv = true;
        }

        let input_path = if hz < 28_800_000 {
            Some(InputPath::Hf) // HF: cable 2, GPIO5 low
        } else if hz < 250_000_000 {
            Some(InputPath::Vhf) // VHF: cable 1, GPIO5 high
        } else {
            Some(InputPath::Uhf) // UHF: air in, GPIO5 high
        };

        let in_notch = hz <= 2_200_000
            || (85_000_000..=112_000_000).contains(&hz)
            || (172_000_000..=242_000_000).contains(&hz);

        TuningPlan {
            tuner_hz,
            spectral_inv,
            input_path,
            in_notch,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_v4_orchestrator_hf() {
        let orch = V4Orchestrator;
        // 7 MHz HF
        let plan = orch.plan_tuning(7_000_000);
        assert_eq!(plan.tuner_hz, 35_800_000);
        assert!(plan.spectral_inv);
        assert_eq!(plan.input_path, Some(InputPath::Hf));
        assert!(plan.in_notch); // 7 MHz is outside AM notch but in_notch handles <= 2.2MHz
    }

    #[test]
    fn test_v4_orchestrator_vhf() {
        let orch = V4Orchestrator;
        // 100 MHz FM
        let plan = orch.plan_tuning(100_000_000);
        assert_eq!(plan.tuner_hz, 100_000_000);
        assert!(!plan.spectral_inv);
        assert_eq!(plan.input_path, Some(InputPath::Vhf));
        assert!(plan.in_notch); // 100 MHz is in FM notch band
    }

    #[test]
    fn test_v4_orchestrator_uhf() {
        let orch = V4Orchestrator;
        // 900 MHz UHF
        let plan = orch.plan_tuning(900_000_000);
        assert_eq!(plan.tuner_hz, 900_000_000);
        assert!(!plan.spectral_inv);
        assert_eq!(plan.input_path, Some(InputPath::Uhf));
        assert!(!plan.in_notch);
    }

    #[test]
    fn test_generic_orchestrator() {
        let orch = GenericOrchestrator;
        let plan = orch.plan_tuning(100_000_000);
        assert_eq!(plan.tuner_hz, 100_000_000);
        assert!(!plan.spectral_inv);
        assert_eq!(plan.input_path, None);
        assert!(!plan.in_notch);
    }
}

impl BoardConfig {
    pub fn orchestrator(&self) -> Box<dyn BoardOrchestrator> {
        match self {
            BoardConfig::Generic => Box::new(GenericOrchestrator),
            BoardConfig::BlogV4 => Box::new(V4Orchestrator),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputPath {
    Hf,
    Vhf,
    Uhf,
}

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
    fn set_input_path(&self, _path: InputPath) -> Result<()> {
        Ok(())
    }

    /// Apply board-level notch filter hint. Called by the orchestrator
    /// after `set_frequency` so the chip can adjust `open_d` in `set_mux`
    /// without knowing anything about the board it's sitting on.
    /// Default: no-op (Generic boards, future tuners).
    fn apply_notch(&self, _in_notch_band: bool) -> Result<()> {
        Ok(())
    }
}
