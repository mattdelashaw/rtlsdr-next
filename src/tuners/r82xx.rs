//! R828D / R820T chip driver — pure chip implementation.
//!
//! This module knows only about the R828D silicon: PLL math, register maps,
//! gain tables, IF bandwidth. It has no knowledge of which board it is
//! mounted on. All GPIO, triplexer, and notch-filter logic lives in the
//! `Driver` orchestrator (`lib.rs`) via `BoardConfig`.

use crate::device::HardwareInterface;
use crate::error::{Error, Result};
use crate::tuner::{FilterRange, Tuner, TunerType};
use log::warn;
use parking_lot::Mutex;
use std::sync::Arc;
use std::time::Duration;

const NUM_REGS: usize = 27;
const REG_SHADOW_START: u8 = 0x05;

const VCO_MIN: u64 = 1_770_000_000;
const VCO_MAX: u64 = 3_600_000_000;

const IF_FREQ_NARROW: u64 = 2_300_000;

const GAIN_STEPS: [i32; 29] = [
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338, 364, 372,
    386, 402, 421, 434, 439, 445, 480, 496,
];

struct GainEntry {
    lna: u8,
    mix: u8,
    vga: u8,
}

#[rustfmt::skip]
static GAIN_TABLE: [GainEntry; 29] = [
    GainEntry { lna:  0, mix:  0, vga:  0 }, GainEntry { lna:  1, mix:  0, vga:  2 },
    GainEntry { lna:  2, mix:  0, vga:  2 }, GainEntry { lna:  3, mix:  0, vga:  2 },
    GainEntry { lna:  4, mix:  1, vga:  3 }, GainEntry { lna:  5, mix:  1, vga:  3 },
    GainEntry { lna:  6, mix:  2, vga:  4 }, GainEntry { lna:  7, mix:  2, vga:  4 },
    GainEntry { lna:  8, mix:  3, vga:  5 }, GainEntry { lna:  9, mix:  3, vga:  5 },
    GainEntry { lna: 10, mix:  4, vga:  6 }, GainEntry { lna: 11, mix:  4, vga:  6 },
    GainEntry { lna: 12, mix:  5, vga:  7 }, GainEntry { lna: 13, mix:  5, vga:  7 },
    GainEntry { lna: 14, mix:  6, vga:  8 }, GainEntry { lna: 15, mix:  6, vga:  8 },
    GainEntry { lna: 15, mix:  7, vga:  9 }, GainEntry { lna: 15, mix:  7, vga:  9 },
    GainEntry { lna: 15, mix:  8, vga: 10 }, GainEntry { lna: 15, mix:  8, vga: 10 },
    GainEntry { lna: 15, mix:  9, vga: 11 }, GainEntry { lna: 15, mix:  9, vga: 11 },
    GainEntry { lna: 15, mix: 10, vga: 12 }, GainEntry { lna: 15, mix: 10, vga: 12 },
    GainEntry { lna: 15, mix: 11, vga: 13 }, GainEntry { lna: 15, mix: 11, vga: 13 },
    GainEntry { lna: 15, mix: 12, vga: 14 }, GainEntry { lna: 15, mix: 12, vga: 14 },
    GainEntry { lna: 15, mix: 13, vga: 15 },
];

static INIT_ARRAY: [u8; NUM_REGS] = [
    0x83, 0x30, 0x75, 0xc0, 0x40, 0xd6, 0x6c, 0xf5, 0x63, 0x75, 0x68, 0x6c, 0x83, 0x80, 0x00, 0x0f,
    0x00, 0xc0, 0x30, 0x48, 0xcc, 0x60, 0x00, 0x54, 0xae, 0x4a, 0xc0,
];

struct FreqRange {
    freq_hz: u64,
    open_d: u8,
    rf_mux_ploy: u8,
    tf_c: u8,
    xtal_cap_sel: u8,
}

#[rustfmt::skip]
static FREQ_RANGES: &[FreqRange] = &[
    FreqRange { freq_hz:           0, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xdf, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  50_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xbe, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  55_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x8b, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  60_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x7b, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  65_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x69, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  70_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x58, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  75_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  80_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  90_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 100_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 110_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 120_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 140_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x14, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 180_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 220_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 250_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x11, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 280_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 310_000_000, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 450_000_000, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 588_000_000, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz: 650_000_000, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, xtal_cap_sel: 0 },
];

static XTAL_CAP_SEL: [u8; 5] = [0x0b, 0x0b, 0x0b, 0x0b, 0x00];

// ── parking_lot::Mutex: no poisoning, no Result unwrap, never blocks the
//    Tokio executor even when locked from sync context. ──────────────────
pub struct R82xx {
    device: Arc<dyn HardwareInterface>,
    tuner_type: TunerType,
    i2c_addr: u8,
    regs: Mutex<[u8; NUM_REGS]>,
    nominal_xtal: u64,
    xtal_freq: Mutex<u64>,
    has_lock: Mutex<bool>,
    current_gain: Mutex<f32>,
    current_if: Mutex<u64>,
    /// Notch state set by the orchestrator via `apply_notch`.
    /// `true`  → frequency is inside a notch band, suppress open_d.
    /// `false` → normal operation, use table value.
    in_notch: Mutex<bool>,
}

impl R82xx {
    /// `xtal_hz`: 28_800_000 for R828D/V4, 16_000_000 for R820T.
    pub fn new(
        device: Arc<dyn HardwareInterface>,
        tuner_type: TunerType,
        i2c_addr: u8,
        xtal_hz: u64,
    ) -> Self {
        Self {
            device,
            tuner_type,
            i2c_addr,
            regs: Mutex::new(INIT_ARRAY),
            nominal_xtal: xtal_hz,
            xtal_freq: Mutex::new(xtal_hz),
            has_lock: Mutex::new(false),
            current_gain: Mutex::new(0.0),
            current_if: Mutex::new(IF_FREQ_NARROW),
            in_notch: Mutex::new(false),
        }
    }

    fn update_shadow(&self, reg: u8, val: u8, mask: u8) -> Result<u8> {
        let mut regs = self.regs.lock();
        let idx = (reg - REG_SHADOW_START) as usize;
        if idx >= NUM_REGS {
            return Err(Error::Tuner(format!("Register 0x{:02x} out of range", reg)));
        }
        let new = (regs[idx] & !mask) | (val & mask);
        regs[idx] = new;
        Ok(new)
    }

    /// Write without repeater toggle — must be called inside `with_repeater`.
    fn write_reg_mask_raw(&self, reg: u8, val: u8, mask: u8) -> Result<()> {
        let new = self.update_shadow(reg, val, mask)?;
        self.device.i2c_write_raw(self.i2c_addr, &[reg, new])
    }

    fn write_reg_mask(&self, reg: u8, val: u8, mask: u8) -> Result<()> {
        self.with_repeater(|| self.write_reg_mask_raw(reg, val, mask))
    }

    /// Hold the I2C repeater open for the duration of a closure.
    /// Repeater is always closed on exit, even on error.
    fn with_repeater<F, T>(&self, f: F) -> Result<T>
    where
        F: FnOnce() -> Result<T>,
    {
        self.device.set_i2c_repeater(true)?;
        let result = f();
        let _ = self.device.set_i2c_repeater(false);
        result
    }

    /// Raw read — must be called inside `with_repeater`.
    fn read_status_raw(&self) -> Result<[u8; 5]> {
        self.device.i2c_write_raw(self.i2c_addr, &[0x00])?;
        let mut data = self.device.i2c_read_raw(self.i2c_addr, 5)?;
        for byte in data.iter_mut() {
            *byte = bit_reverse(*byte);
        }
        Ok([data[0], data[1], data[2], data[3], data[4]])
    }

    /// Must be called inside `with_repeater`.
    fn wait_pll_lock_raw(&self, retries: u32) -> Result<bool> {
        let t = std::time::Instant::now();
        for i in 0..retries {
            self.device.i2c_write_raw(self.i2c_addr, &[0x00])?;
            let mut status = self.device.i2c_read_raw(self.i2c_addr, 3)?;
            for byte in status.iter_mut() {
                *byte = bit_reverse(*byte);
            }
            if status[2] & 0x40 != 0 {
                *self.has_lock.lock() = true;
                log::debug!(
                    "PLL locked after {} attempt(s), {}µs",
                    i + 1,
                    t.elapsed().as_micros()
                );
                return Ok(true);
            }
            if i == 0 {
                self.write_reg_mask_raw(0x12, 0x06, 0xff)?;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        *self.has_lock.lock() = false;
        warn!(
            "PLL not locked after {} retries ({}ms)",
            retries,
            t.elapsed().as_millis()
        );
        Ok(false)
    }
    /// Must be called inside `with_repeater`.
    fn set_pll_raw(&self, lo_freq_hz: u64) -> Result<u64> {
        let pll_ref = *self.xtal_freq.lock();
        let pll_ref_khz = pll_ref / 1000;

        let mut mix_div: u64 = 2;
        let mut div_num: u8 = 0;
        while mix_div <= 64 {
            let vco = lo_freq_hz * mix_div;
            if (VCO_MIN..VCO_MAX).contains(&vco) {
                let mut div_buf = mix_div;
                while div_buf > 2 {
                    div_buf >>= 1;
                    div_num += 1;
                }
                break;
            }
            mix_div <<= 1;
        }
        if mix_div > 64 {
            return Err(Error::InvalidFrequency(lo_freq_hz));
        }

        let vco_freq = lo_freq_hz * mix_div;

        // R828D VCO power ref = 1, R820T = 2.
        let vco_power_ref: u64 = match self.tuner_type {
            TunerType::R828D => 1,
            TunerType::R820T => 2,
            _ => 1,
        };

        let nint: u64 = vco_freq / (2 * pll_ref);
        let mut vco_fra: u64 = (vco_freq - 2 * pll_ref * nint) / 1000;
        if nint > (128 / vco_power_ref) - 1 {
            return Err(Error::InvalidFrequency(lo_freq_hz));
        }

        let ni = ((nint - 13) / 4) as u8;
        let si = (nint as u8).wrapping_sub(4u8.wrapping_mul(ni).wrapping_add(13));

        let pw_sdm: u8 = if vco_fra == 0 { 0x08 } else { 0x00 };
        let mut sdm: u32 = 0;
        let mut n_sdm: u32 = 2;
        while vco_fra > 1 {
            if vco_fra > (2 * pll_ref_khz / n_sdm as u64) {
                sdm += 32768 / (n_sdm / 2);
                vco_fra -= 2 * pll_ref_khz / n_sdm as u64;
                if n_sdm >= 0x8000 {
                    break;
                }
            }
            n_sdm <<= 1;
        }

        // ── Pass 1: Apply initial registers ───────────────────────────────
        self.write_reg_mask_raw(0x10, div_num << 5, 0xe0)?;
        self.device
            .i2c_write_raw(self.i2c_addr, &[0x14, ni | (si << 6)])?;
        self.device
            .i2c_write_raw(self.i2c_addr, &[0x16, (sdm >> 8) as u8])?;
        self.device
            .i2c_write_raw(self.i2c_addr, &[0x15, (sdm & 0xff) as u8])?;
        self.write_reg_mask_raw(0x12, pw_sdm, 0x08)?;

        // ── Settle & Probe ────────────────────────────────────────────────
        // Give the VCO a moment to settle so status is valid for the NEW freq.
        std::thread::sleep(Duration::from_millis(2));
        let status = self.read_status_raw()?;
        let vco_fine_tune = (status[4] & 0x30) >> 4;

        // ── Pass 2: Adjust divisor if VCO band is off ─────────────────────
        if vco_fine_tune > vco_power_ref as u8 {
            div_num = div_num.saturating_sub(1);
            self.write_reg_mask_raw(0x10, div_num << 5, 0xe0)?;
        } else if (vco_fine_tune as u64) < vco_power_ref {
            div_num += 1;
            self.write_reg_mask_raw(0x10, div_num << 5, 0xe0)?;
        }

        if self.wait_pll_lock_raw(10)? {
            self.write_reg_mask_raw(0x1a, 0x08, 0x08)?;
        }
        Ok(lo_freq_hz)
    }

    fn set_if_bandwidth(&self, if_hz: u64) -> Result<()> {
        let (reg_0a, reg_0b) = if if_hz >= 3_500_000 {
            (0x10, 0x6b)
        } else {
            (0x00, 0x80)
        };
        self.with_repeater(|| {
            self.write_reg_mask_raw(0x0a, reg_0a, 0x70)?;
            self.write_reg_mask_raw(0x0b, reg_0b, 0xef)
        })
    }

    /// Must be called inside `with_repeater`.
    fn set_mux_raw(&self, freq_hz: u64) -> Result<()> {
        let range = FREQ_RANGES
            .iter()
            .rev()
            .find(|r| freq_hz >= r.freq_hz)
            .unwrap_or(&FREQ_RANGES[0]);

        self.write_reg_mask_raw(0x17, 0xa0, 0x30)?;
        let open_d = if *self.in_notch.lock() {
            0x00
        } else {
            range.open_d
        };
        self.write_reg_mask_raw(0x17, open_d, 0x08)?;
        self.write_reg_mask_raw(0x1a, range.rf_mux_ploy, 0xc3)?;
        self.write_reg_mask_raw(0x1b, range.tf_c, 0xff)?;
        let cap = XTAL_CAP_SEL[range.xtal_cap_sel as usize];
        self.write_reg_mask_raw(0x10, cap, 0x0b)?;
        self.write_reg_mask_raw(0x08, 0x00, 0x3f)?;
        self.write_reg_mask_raw(0x09, 0x00, 0x3f)?;
        self.write_reg_mask_raw(0x1d, 0x18, 0x38)?;
        self.write_reg_mask_raw(0x1c, 0x24, 0x04)?;
        self.write_reg_mask_raw(0x1e, 14, 0x1f)?;
        self.write_reg_mask_raw(0x1a, 0x20, 0x30)?;
        Ok(())
    }
}

impl Tuner for R82xx {
    fn initialize(&self) -> Result<()> {
        let mid = 16;
        self.device
            .i2c_write_tuner(self.i2c_addr, REG_SHADOW_START, &INIT_ARRAY[..mid])?;
        self.device.i2c_write_tuner(
            self.i2c_addr,
            REG_SHADOW_START + mid as u8,
            &INIT_ARRAY[mid..],
        )?;
        self.with_repeater(|| {
            self.write_reg_mask_raw(0x1a, 0x00, 0x0c)?;
            self.write_reg_mask_raw(0x12, 0x06, 0xff)?;
            self.write_reg_mask_raw(0x0c, 0x00, 0x0f)?;
            self.write_reg_mask_raw(0x13, 0x01, 0x3f)
        })?;
        self.set_if_bandwidth(IF_FREQ_NARROW)?;
        Ok(())
    }

    fn set_frequency(&self, hz: u64) -> Result<u64> {
        if hz == 0 {
            return Err(Error::InvalidFrequency(hz));
        }
        let current_if = *self.current_if.lock();
        let lo_freq = hz + current_if;
        // Single repeater bracket for entire mux + pll. Before: ~270ms. After: ~45ms.
        self.with_repeater(|| {
            let t_mux = std::time::Instant::now();
            self.set_mux_raw(hz)?;
            log::debug!("set_mux: {}µs", t_mux.elapsed().as_micros());
            let t_pll = std::time::Instant::now();
            self.set_pll_raw(lo_freq)?;
            log::debug!("set_pll total: {}µs", t_pll.elapsed().as_micros());
            Ok(())
        })?;
        Ok(hz)
    }

    fn set_gain(&self, db: f32) -> Result<f32> {
        let target_tenths = (db * 10.0) as i32;
        let (idx, _) = GAIN_STEPS
            .iter()
            .enumerate()
            .min_by_key(|&(_, g)| (g - target_tenths).abs())
            .ok_or_else(|| Error::Tuner("Empty gain table".into()))?;
        let cfg = &GAIN_TABLE[idx];

        // 0x05: bits 0-3 are LNA gain. bits 5-6 are V4 antenna mux.
        // Mask 0x0f preserves the antenna mux bits.
        self.with_repeater(|| {
            self.write_reg_mask_raw(0x05, 0x10, 0x10)?;
            self.write_reg_mask_raw(0x07, 0x00, 0x10)?;
            self.write_reg_mask_raw(0x05, cfg.lna, 0x0f)?;
            self.write_reg_mask_raw(0x07, cfg.mix, 0x0f)?;
            self.write_reg_mask_raw(0x0c, cfg.vga, 0x0f)
        })?;

        let actual = GAIN_STEPS[idx] as f32 / 10.0;
        *self.current_gain.lock() = actual;
        Ok(actual)
    }

    fn get_gain(&self) -> Result<f32> {
        Ok(*self.current_gain.lock())
    }

    fn get_filters(&self) -> Vec<FilterRange> {
        // Generic: the full chip range. Board-specific ranges (V4 triplexer
        // bands) are reported by the Driver orchestrator if needed.
        vec![FilterRange {
            start_hz: 0,
            end_hz: 1_766_000_000,
        }]
    }

    fn set_if_freq(&self, hz: u64) -> Result<()> {
        *self.current_if.lock() = hz;
        self.set_if_bandwidth(hz)?;
        Ok(())
    }

    fn get_if_freq(&self) -> u64 {
        *self.current_if.lock()
    }

    fn set_bandwidth(&self, hz: u32) -> Result<()> {
        // Standard R820T/R828D analog filter bandwidths:
        // <= 6 MHz: 0x08 (binary 1000)
        // <= 7 MHz: 0x06 (binary 0110)
        // <= 8 MHz: 0x00 (binary 0000)
        let val = if hz <= 6_000_000 {
            0x08
        } else if hz <= 7_000_000 {
            0x06
        } else {
            0x00
        };
        // Bits 0-3 of register 0x0a
        self.write_reg_mask(0x0a, val, 0x0f)
    }

    fn set_ppm(&self, ppm: i32) -> Result<()> {
        let nominal = self.nominal_xtal;
        let offset = (nominal as i64 * ppm as i64) / 1_000_000;
        *self.xtal_freq.lock() = (nominal as i64 + offset) as u64;
        Ok(())
    }

    /// Called by the Driver orchestrator after computing `BoardConfig::in_notch_band`.
    /// Stores the flag so `set_mux` can apply it on the next tune.
    fn apply_notch(&self, in_notch_band: bool) -> Result<()> {
        *self.in_notch.lock() = in_notch_band;
        Ok(())
    }

    fn set_gain_by_index(&self, idx: usize) -> Result<f32> {
        let clamped = idx.min(GAIN_STEPS.len() - 1);
        self.set_gain(GAIN_STEPS[clamped] as f32 / 10.0)
    }

    fn get_gain_table(&self) -> Vec<i32> {
        GAIN_STEPS.to_vec()
    }

    fn set_input_path(&self, path: crate::tuner::InputPath) -> Result<()> {
        // R820T only has one input path, so this is a no-op for it.
        if self.tuner_type == TunerType::R820T {
            return Ok(());
        }

        use crate::tuner::InputPath::*;
        self.with_repeater(|| match path {
            Hf => {
                self.write_reg_mask_raw(0x06, 0x08, 0x08)?; // cable 2 active
                self.write_reg_mask_raw(0x05, 0x00, 0x60) // cable 1 / air in off
            }
            Vhf => {
                self.write_reg_mask_raw(0x05, 0x40, 0x60)?; // cable 1 active
                self.write_reg_mask_raw(0x06, 0x00, 0x08) // cable 2 off
            }
            Uhf => {
                self.write_reg_mask_raw(0x05, 0x20, 0x60)?; // air in active
                self.write_reg_mask_raw(0x06, 0x00, 0x08) // cable 2 off
            }
        })
    }
}

fn bit_reverse(mut b: u8) -> u8 {
    b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
    b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
    b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
    b
}
