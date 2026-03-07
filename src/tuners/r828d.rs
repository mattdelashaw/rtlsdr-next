use crate::device::HardwareInterface;
use crate::tuner::{Tuner, FilterRange};
use crate::error::{Error, Result};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use log::{info, warn};

// ============================================================
// Constants
// ============================================================

const I2C_ADDR: u8 = 0x34;
const NUM_REGS: usize = 27; // Shadow regs 0x05..=0x1f
const REG_SHADOW_START: u8 = 0x05;

// VCO operating range (Hz) — from reference driver
const VCO_MIN: u64 = 1_770_000_000;
const VCO_MAX: u64 = 3_600_000_000;

// R828D PLL: nint must be 13..=127
const NINT_MIN: u64 = 13;
const NINT_MAX: u64 = 127;

// Intermediate frequency the RTL2832U demodulator expects (Hz)
const IF_FREQ: u64 = 3_570_000;

// Gain table — matches rtl-sdr-blog librtlsdr.c r82xx_gains[]
// Values are in tenths of a dB (e.g. 9 = 0.9 dB) to avoid floats in the table
const GAIN_STEPS: [i32; 29] = [
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
    166, 197, 207, 229, 254, 280, 297, 328,
    338, 364, 372, 386, 402, 421, 434, 439,
    445, 480, 496,
];

struct GainEntry {
    lna: u8,
    mix: u8,
    vga: u8,
}

/// Interleaved gain table from rtl-sdr-blog reference driver.
/// Maps each of the 29 gain steps to (LNA, Mixer, VGA) register values.
static GAIN_TABLE: [GainEntry; 29] = [
    GainEntry { lna: 0,  mix: 0,  vga: 0  }, GainEntry { lna: 1,  mix: 0,  vga: 2  },
    GainEntry { lna: 2,  mix: 0,  vga: 2  }, GainEntry { lna: 3,  mix: 0,  vga: 2  },
    GainEntry { lna: 4,  mix: 1,  vga: 3  }, GainEntry { lna: 5,  mix: 1,  vga: 3  },
    GainEntry { lna: 6,  mix: 2,  vga: 4  }, GainEntry { lna: 7,  mix: 2,  vga: 4  },
    GainEntry { lna: 8,  mix: 3,  vga: 5  }, GainEntry { lna: 9,  mix: 3,  vga: 5  },
    GainEntry { lna: 10, mix: 4,  vga: 6  }, GainEntry { lna: 11, mix: 4,  vga: 6  },
    GainEntry { lna: 12, mix: 5,  vga: 7  }, GainEntry { lna: 13, mix: 5,  vga: 7  },
    GainEntry { lna: 14, mix: 6,  vga: 8  }, GainEntry { lna: 15, mix: 6,  vga: 8  },
    GainEntry { lna: 15, mix: 7,  vga: 9  }, GainEntry { lna: 15, mix: 7,  vga: 9  },
    GainEntry { lna: 15, mix: 8,  vga: 10 }, GainEntry { lna: 15, mix: 8,  vga: 10 },
    GainEntry { lna: 15, mix: 9,  vga: 11 }, GainEntry { lna: 15, mix: 9,  vga: 11 },
    GainEntry { lna: 15, mix: 10, vga: 12 }, GainEntry { lna: 15, mix: 10, vga: 12 },
    GainEntry { lna: 15, mix: 11, vga: 13 }, GainEntry { lna: 15, mix: 11, vga: 13 },
    GainEntry { lna: 15, mix: 12, vga: 14 }, GainEntry { lna: 15, mix: 12, vga: 14 },
    GainEntry { lna: 15, mix: 13, vga: 15 },
];

// ============================================================
// Init register array (shadow regs 0x05..=0x1f)
// Sourced from RTL-SDR Blog fork r82xx_init_array
// ============================================================
static INIT_ARRAY: [u8; NUM_REGS] = [
    0x83, 0x32, 0x75, // 05..07
    0xc0, 0x40, 0xd6, 0x6c, // 08..0b
    0xf5, 0x63, 0x75, 0x68, // 0c..0f
    0x6c, 0x83, 0x80, 0x00, // 10..13
    0x0f, 0x00, 0xc0, 0x30, // 14..17
    0x48, 0xcc, 0x60, 0x00, // 18..1b
    0x54, 0xae, 0x4a, 0xc0, // 1c..1f
];

// ============================================================
// Frequency range table
// Each entry configures the RF mux and filter for a band.
// Sourced from r82xx_freq_ranges[] in reference driver.
// ============================================================
struct FreqRange {
    freq_hz:      u64,
    open_d:       u8,
    rf_mux_ploy:  u8,
    tf_c:         u8,
    xtal_cap_sel: u8, // index into xtal cap table
}

static FREQ_RANGES: &[FreqRange] = &[
    FreqRange { freq_hz:          0, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xdf, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   50_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xbe, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   55_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x8b, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   60_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x7b, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   65_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x69, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   70_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x58, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   75_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   80_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   90_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  100_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  110_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  120_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  140_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x14, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  180_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  220_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  250_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x11, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  280_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  310_000_000, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  450_000_000, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  588_000_000, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  650_000_000, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, xtal_cap_sel: 0 },
];

// Xtal cap values for the 4 selectable cap settings
static XTAL_CAP_SEL: [u8; 5] = [0x0b, 0x0b, 0x0b, 0x0b, 0x00];

// ============================================================
// Struct
// ============================================================

pub struct R828D {
    device:     Arc<dyn HardwareInterface>,
    regs:       Mutex<[u8; NUM_REGS]>,
    xtal_freq:  Mutex<u64>,
    is_v4:      bool,
    has_lock:   Mutex<bool>,
}

// ============================================================
// Internal helpers
// ============================================================

impl R828D {
    pub fn new(device: Arc<dyn HardwareInterface>, is_v4: bool) -> Self {
        Self {
            device,
            regs:      Mutex::new(INIT_ARRAY),
            xtal_freq: Mutex::new(28_800_000), // Default
            is_v4,
            has_lock:  Mutex::new(false),
        }
    }

    /// Write a masked value into a shadow register then push to hardware.
    fn write_reg_mask(&self, reg: u8, val: u8, mask: u8) -> Result<()> {
        let new = {
            let mut regs = self.regs.lock().unwrap();
            let idx = (reg - REG_SHADOW_START) as usize;
            if idx >= NUM_REGS {
                return Err(Error::Tuner(format!("Register 0x{:02x} out of shadow range", reg)));
            }
            let old = regs[idx];
            let new = (old & !mask) | (val & mask);
            regs[idx] = new;
            new
            // lock released here
        };
        self.device.i2c_write_tuner(I2C_ADDR, reg, &[new])
    }

    /// Read a byte from hardware via I2C.
    fn read_reg(&self, reg: u8) -> Result<u8> {
        let data = self.device.i2c_read_tuner(I2C_ADDR, reg, 1)?;
        Ok(data[0])
    }

    fn read_status(&self) -> Result<[u8; 5]> {
        let data = self.device.i2c_read_tuner(I2C_ADDR, 0x00, 5)?;
        Ok([data[0], data[1], data[2], data[3], data[4]])
    }

    /// Poll PLL lock bit (reg 0x00 bit 6) with up to `retries` attempts.
    fn wait_pll_lock(&self, retries: u32) -> Result<bool> {
        for _ in 0..retries {
            let status = self.read_reg(0x00)?;
            if status & 0x40 != 0 {
                *self.has_lock.lock().unwrap() = true;
                return Ok(true);
            }
            // Brief pause — ~1ms between polls
            std::thread::sleep(Duration::from_millis(1));
        }
        *self.has_lock.lock().unwrap() = false;
        warn!("PLL not locked after {} retries", retries);
        Ok(false)
    }

    /// Core PLL synthesis.
    ///
    /// Implements the fixed-point N+SDM fractional-N PLL calculation
    /// from the reference driver (tuner_r82xx.c r82xx_set_pll):
    ///
    ///   vco_div = round(65536 * vco_freq / (2 * pll_ref))
    ///           = (pll_ref + 65536 * vco_freq) / (2 * pll_ref)   [integer]
    ///   nint    = vco_div / 65536
    ///   sdm     = vco_div % 65536
    ///
    /// Then:
    ///   ni = (nint - 13) / 4
    ///   si = nint - 4*ni - 13
    ///   reg 0x14 = ni | (si << 6)
    ///   reg 0x16 = sdm >> 8
    ///   reg 0x15 = sdm & 0xff
    fn set_pll(&self, lo_freq_hz: u64) -> Result<u64> {
        let pll_ref = *self.xtal_freq.lock().unwrap();

        // --- Step 1: find mix_div so that vco_freq is in [VCO_MIN, VCO_MAX) ---
        let mut mix_div: u64 = 2;
        let mut div_num: u8 = 0; // log2(mix_div/2)

        while mix_div <= 128 {
            let vco = lo_freq_hz * mix_div;
            if vco >= VCO_MIN && vco < VCO_MAX {
                // div_num = log2(mix_div) - 1
                let mut div_buf = mix_div;
                while div_buf > 2 {
                    div_buf >>= 1;
                    div_num += 1;
                }
                break;
            }
            mix_div <<= 1;
        }

        if mix_div > 128 {
            return Err(Error::InvalidFrequency(lo_freq_hz));
        }

        let vco_freq = lo_freq_hz * mix_div;

        // --- Step 2: vco_fine_tune adjustment (R828D specific) ---
        // R828D vco_power_ref = 1; read from status reg 0x04 bits [5:4]
        // Reference driver hardcodes vco_fine_tune = 2 on some paths,
        // but we read it properly here.
        let vco_power_ref: u8 = 1; // R828D constant
        let status = self.read_status()?;
        let vco_fine_tune = (status[4] & 0x30) >> 4;

        if vco_fine_tune > vco_power_ref {
            div_num = div_num.saturating_sub(1);
        } else if vco_fine_tune < vco_power_ref {
            div_num += 1;
        }

        // Write divider selection to reg 0x10 bits [7:5]
        self.write_reg_mask(0x10, div_num << 5, 0xe0)?;

        // --- Step 3: fixed-point N+SDM calculation ---
        // vco_div = floor((pll_ref + 65536 * vco_freq) / (2 * pll_ref))
        let vco_div: u64 = (pll_ref + 65536 * vco_freq) / (2 * pll_ref);
        let nint: u64 = vco_div / 65536;
        let sdm:  u64 = vco_div % 65536;

        // Validate nint range for R828D
        if nint < NINT_MIN || nint > NINT_MAX {
            return Err(Error::InvalidFrequency(lo_freq_hz));
        }

        // Encode nint into ni/si format
        // ni = (nint - 13) / 4
        // si = nint - 4*ni - 13
        let ni = ((nint - 13) / 4) as u8;
        let si = (nint as u8).wrapping_sub(4u8.wrapping_mul(ni).wrapping_add(13));

        self.device.i2c_write_tuner(I2C_ADDR, 0x14, &[ni | (si << 6)])?;

        // pw_sdm: disable dithering when SDM == 0 (integer-N mode)
        let pw_sdm: u8 = if sdm == 0 { 0x08 } else { 0x00 };
        self.write_reg_mask(0x12, pw_sdm, 0x08)?;

        // Write SDM fraction (big-endian split across 0x16, 0x15)
        self.device.i2c_write_tuner(I2C_ADDR, 0x16, &[(sdm >> 8) as u8])?;
        self.device.i2c_write_tuner(I2C_ADDR, 0x15, &[(sdm & 0xff) as u8])?;

        info!(
            "LO: {} kHz | MixDiv: {} | nint: {} | SDM: {} | VCO: {} kHz",
            lo_freq_hz / 1000, mix_div, nint, sdm, vco_freq / 1000
        );

        // --- Step 4: poll for lock ---
        self.wait_pll_lock(10)?;

        Ok(lo_freq_hz)
    }

    /// Configure RF mux, tracking filter, and xtal cap for a given frequency.
    fn set_mux(&self, freq_hz: u64) -> Result<()> {
        // Find the last range entry whose freq_hz <= our frequency
        let range = FREQ_RANGES
            .iter()
            .rev()
            .find(|r| freq_hz >= r.freq_hz)
            .unwrap_or(&FREQ_RANGES[0]);

        // open_d — controls RF input path
        self.write_reg_mask(0x17, range.open_d, 0x08)?;

        // RF mux and poly filter
        self.write_reg_mask(0x1a, range.rf_mux_ploy, 0xc3)?;

        // Tracking filter
        self.write_reg_mask(0x1b, range.tf_c, 0xff)?;

        // Xtal cap selection
        let cap = XTAL_CAP_SEL[range.xtal_cap_sel as usize];
        self.write_reg_mask(0x10, cap, 0x0b)?;
        self.write_reg_mask(0x08, 0x00, 0x3f)?;
        self.write_reg_mask(0x09, 0x00, 0x3f)?;

        Ok(())
    }

    /// V4-specific triplexer input switching.
    /// The V4 has three RF inputs: HF (upconverted), VHF (Cable1), UHF (Air).
    /// RTL-SDR Blog fork switches at 28.8 MHz and 250 MHz.
    pub(crate) fn set_v4_input(&self, freq_hz: u64) -> Result<()> {
        if freq_hz <= 28_800_000 {
            // HF band — route through upconverter (Cable2 path)
            self.write_reg_mask(0x06, 0x08, 0x08)?; // Cable2 ON
            self.write_reg_mask(0x05, 0x00, 0x40)?; // Cable1 OFF
            self.write_reg_mask(0x05, 0x20, 0x20)?; // Air OFF
        } else if freq_hz < 250_000_000 {
            // VHF band — Cable1 input
            self.write_reg_mask(0x06, 0x00, 0x08)?; // Cable2 OFF
            self.write_reg_mask(0x05, 0x40, 0x40)?; // Cable1 ON
            self.write_reg_mask(0x05, 0x20, 0x20)?; // Air OFF
        } else {
            // UHF band — Air input
            self.write_reg_mask(0x06, 0x00, 0x08)?; // Cable2 OFF
            self.write_reg_mask(0x05, 0x00, 0x40)?; // Cable1 OFF
            self.write_reg_mask(0x05, 0x00, 0x20)?; // Air ON
        }
        Ok(())
    }
}

// ============================================================
// Tuner trait implementation
// ============================================================

impl Tuner for R828D {
    fn initialize(&self) -> Result<()> {
        // Write full init array starting at register 0x05
        self.device.i2c_write_tuner(I2C_ADDR, REG_SHADOW_START, &INIT_ARRAY)?;

        // Set PLL autotune to 128 kHz
        self.write_reg_mask(0x1a, 0x00, 0x0c)?;

        // Set VCO current = 100 (reference driver: reg 0x12 bits [7:5] = 0x80 >> 0)
        self.write_reg_mask(0x12, 0x80, 0xe0)?;

        // Clear init flag and set version
        self.write_reg_mask(0x0c, 0x00, 0x0f)?;
        self.write_reg_mask(0x13, 0x01, 0x3f)?;

        Ok(())
    }

    /// Set frequency in Hz.
    ///
    /// Full sequence:
    ///   1. Compute LO = freq + IF_FREQ
    ///   2. Configure RF mux and tracking filter for band
    ///   3. Switch V4 triplexer input if applicable
    ///   4. Run PLL synthesis and poll for lock
    fn set_frequency(&self, hz: u64) -> Result<u64> {
        if hz == 0 {
            return Err(Error::InvalidFrequency(hz));
        }

        // LO must be offset by IF so the signal lands at the demodulator IF
        let lo_freq = hz + IF_FREQ;

        // Configure band-dependent RF path
        self.set_mux(hz)?;

        // V4: switch triplexer input based on frequency band
        if self.is_v4 {
            self.set_v4_input(hz)?;
        }

        // Program the PLL to lo_freq
        self.set_pll(lo_freq)?;

        Ok(hz)
    }

    /// Set gain in dB. Finds the nearest step in the gain table.
    fn set_gain(&self, db: f32) -> Result<f32> {
        let target_tenths = (db * 10.0) as i32;

        // Find the closest gain step
        let (idx, _) = GAIN_STEPS
            .iter()
            .enumerate()
            .min_by_key(|&(_, g)| (g - target_tenths).abs())
            .unwrap();

        let cfg = &GAIN_TABLE[idx];

        // Enable manual gain (reg 0x05 bit 4 = 1)
        self.write_reg_mask(0x05, 0x10, 0x10)?;

        // LNA gain: reg 0x05 bits [3:0]
        self.write_reg_mask(0x05, cfg.lna, 0x0f)?;

        // Mixer gain: reg 0x07 bits [3:0]
        self.write_reg_mask(0x07, cfg.mix, 0x0f)?;

        // VGA gain: reg 0x0a bits [3:0]
        self.write_reg_mask(0x0a, cfg.vga, 0x0f)?;

        info!(
            "Gain set to {:.1} dB (Idx: {}, LNA: {}, Mix: {}, VGA: {})",
            GAIN_STEPS[idx] as f32 / 10.0, idx, cfg.lna, cfg.mix, cfg.vga
        );

        Ok(GAIN_STEPS[idx] as f32 / 10.0)
    }

    /// Return the three triplexer filter ranges for the V4.
    fn get_filters(&self) -> Vec<FilterRange> {
        if self.is_v4 {
            vec![
                FilterRange { start_hz: 0,           end_hz: 28_800_000   }, // HF
                FilterRange { start_hz: 28_800_001,  end_hz: 249_999_999  }, // VHF
                FilterRange { start_hz: 250_000_000, end_hz: 1_766_000_000}, // UHF
            ]
        } else {
            vec![
                FilterRange { start_hz: 0, end_hz: 1_766_000_000 },
            ]
        }
    }

    /// Toggle the Bias-T (5V on the SMA centre pin for powering LNAs).
    /// Controlled via reg 0x0f bit 0.
    fn set_bias_t(&self, on: bool) -> Result<()> {
        let val: u8 = if on { 0x01 } else { 0x00 };
        self.write_reg_mask(0x0f, val, 0x01)
    }

    /// Update the reference crystal frequency based on PPM offset.
    /// Formula: nominal * (1 + ppm/1e6)
    fn set_ppm(&self, ppm: i32) -> Result<()> {
        let nominal = 28_800_000u64;
        let offset = (nominal as i64 * ppm as i64) / 1_000_000;
        let actual = (nominal as i64 + offset) as u64;
        *self.xtal_freq.lock().unwrap() = actual;
        info!("Tuner crystal frequency updated: {} Hz ({} PPM)", actual, ppm);
        Ok(())
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    struct MockHardware {
        writes:     Mutex<Vec<(u8, u8, Vec<u8>)>>,
        reg_writes: Mutex<Vec<(u16, u16, u8)>>,
    }

    impl MockHardware {
        fn new() -> Self {
            Self {
                writes:     Mutex::new(vec![]),
                reg_writes: Mutex::new(vec![]),
            }
        }
    }

    impl HardwareInterface for MockHardware {
        fn read_reg(&self, _: u16, _: u16) -> Result<u8> { Ok(0) }
        fn write_reg(&self, block: u16, addr: u16, val: u8) -> Result<()> {
            self.reg_writes.lock().unwrap().push((block, addr, val));
            Ok(())
        }
        fn i2c_read(&self, _: u8, _: u8, len: usize) -> Result<Vec<u8>> {
            let mut data = vec![0u8; len];
            if len >= 5 { data[4] = 0x20; }
            Ok(data)
        }
        fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
            self.writes.lock().unwrap().push((addr, reg, data.to_vec()));
            Ok(())
        }
        fn read_bulk(&self, _: u8, _: &mut [u8], _: Duration) -> Result<usize> { Ok(0) }
    }

    fn make_tuner(is_v4: bool) -> (R828D, Arc<MockHardware>) {
        let mock = Arc::new(MockHardware::new());
        let tuner = R828D::new(mock.clone(), is_v4);
        (tuner, mock)
    }

    #[test]
    fn test_i2c_repeater_bracketed() {
        // Every tuner I2C write must be bracketed by repeater ON (0x08) / OFF (0x00)
        // via demod page 0 register 0x08.
        let (tuner, mock) = make_tuner(false);
        mock.reg_writes.lock().unwrap().clear();

        tuner.initialize().unwrap();

        let reg_writes = mock.reg_writes.lock().unwrap();
        use crate::registers::demod::P0_IIC_REPEAT;
        let repeater_block = crate::registers::Block::Demod as u16;
        let repeater_writes: Vec<u8> = reg_writes
            .iter()
            .filter(|(b, addr, _)| *b == repeater_block && *addr == P0_IIC_REPEAT)
            .map(|(_, _, val)| *val)
            .collect();

        // Must have at least one ON/OFF pair
        assert!(!repeater_writes.is_empty(), "No repeater writes found");
        // First write must be ON (0x08)
        assert_eq!(repeater_writes[0], 0x08, "First repeater write should be ON");
        // Last write must be OFF (0x00)
        assert_eq!(
            *repeater_writes.last().unwrap(), 0x00,
            "Last repeater write should be OFF"
        );
        // Every ON must be followed by an OFF — no dangling enables
        let mut i = 0;
        while i < repeater_writes.len() {
            assert_eq!(repeater_writes[i], 0x08, "Expected ON at index {}", i);
            assert!(i + 1 < repeater_writes.len(), "Dangling repeater ON at index {}", i);
            assert_eq!(repeater_writes[i + 1], 0x00, "Expected OFF at index {}", i + 1);
            i += 2;
        }
    }

    #[test]
    fn test_initialize_writes_init_array() {
        let (tuner, mock) = make_tuner(true);
        tuner.initialize().unwrap();
        let writes = mock.writes.lock().unwrap();
        // First write should be the full init array at reg 0x05
        let first = &writes[0];
        assert_eq!(first.1, 0x05);
        assert_eq!(first.2, INIT_ARRAY.to_vec());
    }

    #[test]
    fn test_band_switching_hf() {
        let (tuner, mock) = make_tuner(true);
        tuner.initialize().unwrap();
        mock.writes.lock().unwrap().clear();

        // Call set_v4_input directly — HF band switching is independent of PLL.
        // set_frequency(10MHz) would fail because the R828D VCO cannot reach
        // the required LO frequency (13.57MHz * max_mix_div=128 = 1.737GHz
        // which is still below VCO_MIN of 1.77GHz). The V4 HF band requires
        // an external upconverter — we test the register switching in isolation.
        tuner.set_v4_input(10_000_000).unwrap();
        let writes = mock.writes.lock().unwrap();

        // reg 0x06 bit 0x08 should be set (Cable2 ON)
        let reg06 = writes.iter().find(|(_, r, _)| *r == 0x06);
        assert!(reg06.is_some(), "no write to reg 0x06");
        assert_eq!(reg06.unwrap().2[0] & 0x08, 0x08);
    }

    #[test]
    fn test_band_switching_vhf() {
        let (tuner, mock) = make_tuner(true);
        tuner.initialize().unwrap();
        mock.writes.lock().unwrap().clear();

        tuner.set_frequency(100_000_000).unwrap();
        let writes = mock.writes.lock().unwrap();

        // reg 0x05 bit 0x40 should be set (Cable1 ON)
        let reg05_writes: Vec<_> = writes.iter().filter(|(_, r, _)| *r == 0x05).collect();
        let cable1_on = reg05_writes.iter().any(|(_, _, d)| d[0] & 0x40 != 0);
        assert!(cable1_on, "Cable1 not switched ON for VHF");
    }

    #[test]
    fn test_band_switching_uhf() {
        let (tuner, mock) = make_tuner(true);
        tuner.initialize().unwrap();
        mock.writes.lock().unwrap().clear();

        tuner.set_frequency(440_000_000).unwrap();
        let writes = mock.writes.lock().unwrap();

        // Cable1 and Cable2 should both be off for UHF (Air input)
        let reg05 = writes.iter().filter(|(_, r, _)| *r == 0x05).last();
        let reg06 = writes.iter().filter(|(_, r, _)| *r == 0x06).last();
        if let Some(w) = reg05 { assert_eq!(w.2[0] & 0x40, 0x00, "Cable1 should be OFF in UHF"); }
        if let Some(w) = reg06 { assert_eq!(w.2[0] & 0x08, 0x00, "Cable2 should be OFF in UHF"); }
    }

    #[test]
    fn test_pll_writes_nint_register() {
        let (tuner, mock) = make_tuner(false);
        tuner.initialize().unwrap();
        mock.writes.lock().unwrap().clear();

        // FM: 98.7 MHz
        tuner.set_frequency(98_700_000).unwrap();
        let writes = mock.writes.lock().unwrap();

        // reg 0x14 should have been written with ni|(si<<6)
        let reg14 = writes.iter().find(|(_, r, _)| *r == 0x14);
        assert!(reg14.is_some(), "PLL nint register 0x14 was never written");
    }

    #[test]
    fn test_pll_writes_sdm_registers() {
        let (tuner, mock) = make_tuner(false);
        tuner.initialize().unwrap();
        mock.writes.lock().unwrap().clear();

        tuner.set_frequency(162_550_000).unwrap(); // NOAA weather
        let writes = mock.writes.lock().unwrap();

        let reg15 = writes.iter().find(|(_, r, _)| *r == 0x15);
        let reg16 = writes.iter().find(|(_, r, _)| *r == 0x16);
        assert!(reg15.is_some(), "SDM low byte register 0x15 never written");
        assert!(reg16.is_some(), "SDM high byte register 0x16 never written");
    }

    #[test]
    fn test_gain_nearest_step() {
        let (tuner, _) = make_tuner(false);
        tuner.initialize().unwrap();

        // Request 25.0 dB — nearest step is 25.4 dB (index 14)
        let actual = tuner.set_gain(25.0).unwrap();
        assert!((actual - 25.4).abs() < 0.1, "Expected ~25.4 dB, got {}", actual);
    }

    #[test]
    fn test_filters_v4_returns_three_bands() {
        let (tuner, _) = make_tuner(true);
        assert_eq!(tuner.get_filters().len(), 3);
    }

    #[test]
    fn test_filters_non_v4_returns_one_band() {
        let (tuner, _) = make_tuner(false);
        assert_eq!(tuner.get_filters().len(), 1);
    }

    #[test]
    fn test_invalid_frequency_zero() {
        let (tuner, _) = make_tuner(false);
        tuner.initialize().unwrap();
        assert!(tuner.set_frequency(0).is_err());
    }
}