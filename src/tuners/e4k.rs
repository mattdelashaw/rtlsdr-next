//! Elonics E4000 chip driver.
//!
//! The E4000 is a Zero-IF tuner known for its wide frequency range
//! (typically 52 MHz to 2.2 GHz) and low power consumption.

use crate::device::HardwareInterface;
use crate::error::{Error, Result};
use crate::tuner::{FilterRange, Tuner};
use log::debug;
use parking_lot::Mutex;
use std::sync::Arc;

const E4K_I2C_ADDR: u8 = 0xc8; // 0x64 << 1

// Registers
const E4K_REG_MASTER1: u8 = 0x00;
const E4K_REG_CLK_INP: u8 = 0x05;
const E4K_REG_REF_CLK: u8 = 0x06;
const E4K_REG_SYNTH1: u8 = 0x07;
const E4K_REG_SYNTH3: u8 = 0x09;
const E4K_REG_SYNTH4: u8 = 0x0a;
const E4K_REG_SYNTH5: u8 = 0x0b;
const E4K_REG_SYNTH7: u8 = 0x0d;
const E4K_REG_FILT1: u8 = 0x10;
const E4K_REG_FILT2: u8 = 0x11;
const E4K_REG_FILT3: u8 = 0x12;
const E4K_REG_GAIN1: u8 = 0x14;
const E4K_REG_GAIN2: u8 = 0x15;
const E4K_REG_GAIN3: u8 = 0x16;
const E4K_REG_GAIN4: u8 = 0x17;
const E4K_REG_DC1: u8 = 0x1e;

const E4K_VCO_MIN: u64 = 2_600_000_000;
const E4K_VCO_MAX: u64 = 3_900_000_000;

// Gain steps in 0.1 dB
static E4K_GAIN_STEPS: [i32; 18] = [
    -10, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420, 430, 450, 470, 490,
];

struct PllVar {
    freq: u64,
    div: u8,
    mul: u8,
}

#[rustfmt::skip]
static PLL_VARS: [PllVar; 9] = [
    PllVar { freq: 724_000_000, div: 4, mul: 0x00 },
    PllVar { freq: 482_000_000, div: 6, mul: 0x01 },
    PllVar { freq: 362_000_000, div: 8, mul: 0x02 },
    PllVar { freq: 241_000_000, div: 12, mul: 0x03 },
    PllVar { freq: 181_000_000, div: 16, mul: 0x04 },
    PllVar { freq: 120_000_000, div: 24, mul: 0x05 },
    PllVar { freq: 90_000_000, div: 32, mul: 0x06 },
    PllVar { freq: 60_000_000, div: 48, mul: 0x07 },
    PllVar { freq: 0, div: 48, mul: 0x07 }, // Default
];

pub struct E4k {
    device: Arc<dyn HardwareInterface>,
    nominal_xtal: u64,
    xtal_freq: Mutex<u64>,
    current_gain: Mutex<f32>,
}

impl E4k {
    pub fn new(device: Arc<dyn HardwareInterface>, xtal_hz: u64) -> Self {
        Self {
            device,
            nominal_xtal: xtal_hz,
            xtal_freq: Mutex::new(xtal_hz),
            current_gain: Mutex::new(0.0),
        }
    }

    fn write_reg(&self, reg: u8, val: u8) -> Result<()> {
        self.device.i2c_write_tuner(E4K_I2C_ADDR, reg, &[val])
    }

    fn set_lna_gain(&self, db: i32) -> Result<()> {
        let val = match db {
            d if d < 5 => 0,
            d if d < 10 => 1,
            d if d < 15 => 2,
            d if d < 20 => 3,
            d if d < 25 => 4,
            d if d < 30 => 5,
            _ => 6,
        };
        self.write_reg(E4K_REG_GAIN1, val)
    }

    fn set_mixer_gain(&self, db: i32) -> Result<()> {
        let val = if db < 8 { 0 } else { 1 };
        self.write_reg(E4K_REG_GAIN2, val)
    }

    fn set_if_gain(&self, stage: u8, db: i32) -> Result<()> {
        // E4000 has complex IF gain stages with varying bit widths and step sizes.
        match stage {
            1..=4 => {
                let reg = E4K_REG_GAIN3;
                let current = self.device.i2c_read_tuner(E4K_I2C_ADDR, reg, 1)?[0];
                let (mask, shift, val) = match stage {
                    1 => (0x01, 0, (db / 6).clamp(0, 1) as u8), // -3 or 6dB
                    2 => (0x06, 1, (db / 3).clamp(0, 3) as u8), // 0, 3, 6, 9dB
                    3 => (0x18, 3, (db / 3).clamp(0, 3) as u8), // 0, 3, 6, 9dB
                    4 => (0x60, 5, db.clamp(0, 3) as u8),       // 0, 1, 2dB
                    _ => unreachable!(),
                };
                self.write_reg(reg, (current & !mask) | (val << shift))
            }
            5..=6 => {
                let reg = E4K_REG_GAIN4;
                let current = self.device.i2c_read_tuner(E4K_I2C_ADDR, reg, 1)?[0];
                let (mask, shift) = if stage == 5 { (0x07, 0) } else { (0x38, 3) };
                let val = (db / 3).clamp(0, 7) as u8; // 3 to 15 dB (3dB steps)
                self.write_reg(reg, (current & !mask) | (val << shift))
            }
            _ => Ok(()),
        }
    }
}

impl Tuner for E4k {
    fn initialize(&self) -> Result<()> {
        debug!("Initializing E4000 tuner...");

        // 1. Reset
        self.write_reg(E4K_REG_MASTER1, 0x01)?;

        // 2. Clock config
        self.write_reg(E4K_REG_CLK_INP, 0x00)?;
        self.write_reg(E4K_REG_REF_CLK, 0x00)?; // Use reference clock directly

        // 3. Magic values (from librtlsdr)
        self.write_reg(0x7e, 0x01)?;
        self.write_reg(0x7f, 0xfe)?;
        self.write_reg(0x82, 0x00)?;
        self.write_reg(0x86, 0x50)?;
        self.write_reg(0x87, 0x2c)?;
        self.write_reg(0x88, 0x01)?;
        self.write_reg(0x9f, 0x7f)?;
        self.write_reg(0xa0, 0x07)?;

        // 4. Filters & Gains
        self.write_reg(E4K_REG_FILT1, 0x03)?; // Default RF filter
        self.write_reg(E4K_REG_FILT2, 0x04)?; // Default Mixer filter
        self.write_reg(E4K_REG_FILT3, 0x01)?; // Enable channel filter

        self.set_gain(20.0)?; // Default gain

        // 5. Trigger DC offset calibration
        self.write_reg(E4K_REG_DC1, 0x01)?;

        Ok(())
    }

    fn set_frequency(&self, hz: u64) -> Result<u64> {
        let f_osc = *self.xtal_freq.lock();

        // Find the first divider whose range ceiling is greater than our frequency.
        // The table is ordered by frequency ceilings.
        let var = PLL_VARS
            .iter()
            .find(|v| hz < v.freq)
            .unwrap_or(&PLL_VARS[PLL_VARS.len() - 1]);

        let f_vco = hz * var.div as u64;
        if !(E4K_VCO_MIN..=E4K_VCO_MAX).contains(&f_vco) {
            return Err(Error::InvalidFrequency(hz));
        }

        let z = f_vco / f_osc;
        let x = ((f_vco % f_osc) * 65536) / f_osc;

        // Band selection (Synth1) - RF filter bank selection.
        // Note: Breakpoints are approximate based on common Osmocom/librtlsdr implementations.
        let band = if hz <= 140_000_000 {
            0
        } else if hz <= 350_000_000 {
            1
        } else if hz <= 467_000_000 {
            2
        } else if hz <= 657_000_000 {
            3
        } else if hz <= 930_000_000 {
            4
        } else if hz <= 1_260_000_000 {
            5
        } else if hz <= 1_610_000_000 {
            6
        } else {
            7
        };

        self.write_reg(E4K_REG_SYNTH1, band << 1)?;

        // PLL Parameters
        if z > 255 {
            return Err(Error::InvalidFrequency(hz));
        }
        self.write_reg(E4K_REG_SYNTH3, z as u8)?;
        self.write_reg(E4K_REG_SYNTH4, (x & 0xff) as u8)?;
        self.write_reg(E4K_REG_SYNTH5, ((x >> 8) & 0xff) as u8)?;

        // Divider (Synth7)
        let mut synth7 = var.mul;
        if hz < 350_000_000 {
            synth7 |= 0x08; // 3-phase mixing
        }
        self.write_reg(E4K_REG_SYNTH7, synth7)?;

        Ok(hz)
    }

    fn set_gain(&self, db: f32) -> Result<f32> {
        let target = (db * 10.0) as i32;
        let (_idx, &actual_tenths) = E4K_GAIN_STEPS
            .iter()
            .enumerate()
            .min_by_key(|&(_, &g)| (g - target).abs())
            .ok_or_else(|| Error::Tuner("Empty gain table".into()))?;

        // Simplified gain distribution:
        // LNA (30dB) + Mixer (12dB) + IF stages (rest)
        let total_db = actual_tenths / 10;
        self.set_lna_gain(total_db.min(30))?;
        let mut rem = total_db.saturating_sub(30);
        self.set_mixer_gain(rem.min(12))?;
        rem = rem.saturating_sub(12);

        // Distribute remaining gain across 6 IF stages (up to 3dB each in this model)
        for stage in 1..=6 {
            let stage_gain = rem.min(3);
            self.set_if_gain(stage, stage_gain)?;
            rem = rem.saturating_sub(stage_gain);
        }

        let actual = actual_tenths as f32 / 10.0;
        *self.current_gain.lock() = actual;
        Ok(actual)
    }

    fn get_gain(&self) -> Result<f32> {
        Ok(*self.current_gain.lock())
    }

    fn get_filters(&self) -> Vec<FilterRange> {
        vec![FilterRange {
            start_hz: 64_000_000,
            end_hz: 1_700_000_000, // Official range
        }]
    }

    fn set_if_freq(&self, _hz: u64) -> Result<()> {
        // E4000 is Zero-IF, so IF is effectively 0
        Ok(())
    }

    fn get_if_freq(&self) -> u64 {
        0
    }

    fn set_bandwidth(&self, _hz: u32) -> Result<()> {
        Ok(())
    }

    fn set_ppm(&self, ppm: i32) -> Result<()> {
        let nominal = self.nominal_xtal;
        let offset = (nominal as i64 * ppm as i64) / 1_000_000;
        *self.xtal_freq.lock() = (nominal as i64 + offset) as u64;
        Ok(())
    }
}
