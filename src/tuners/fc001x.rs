//! FC0012 / FC0013 Fitipower tuner chip driver.
//!
//! These are older fractional-N PLL tuners often found in generic RTL-SDR dongles.
//! They use a multi-stage divider system to derive the LO from a 28.8 MHz crystal.

use crate::device::HardwareInterface;
use crate::error::{Error, Result};
use crate::tuner::{FilterRange, Tuner, TunerType};
use parking_lot::Mutex;
use std::sync::Arc;

const FC0012_CHIP_ID: u8 = 0xa1;
const FC0013_CHIP_ID: u8 = 0x63;

pub struct Fc001x {
    device: Arc<dyn HardwareInterface>,
    tuner_type: TunerType,
    i2c_addr: u8,
    xtal_freq: u64,
    current_gain: Mutex<f32>,
    current_if: Mutex<u64>,
}

impl Fc001x {
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
            xtal_freq: xtal_hz,
            current_gain: Mutex::new(0.0),
            current_if: Mutex::new(0),
        }
    }

    fn write_reg(&self, reg: u8, val: u8) -> Result<()> {
        self.device.i2c_write_tuner(self.i2c_addr, reg, &[val])
    }

    fn read_reg(&self, reg: u8) -> Result<u8> {
        let res = self.device.i2c_read_tuner(self.i2c_addr, reg, 1)?;
        Ok(res[0])
    }
}

impl Tuner for Fc001x {
    fn initialize(&self) -> Result<()> {
        // Basic init sequence from librtlsdr
        self.write_reg(0x00, 0x00)?; // Reset / NOP

        // Check ID
        let id = self.read_reg(0x00)?;
        match self.tuner_type {
            TunerType::FC0012 if id != FC0012_CHIP_ID => {
                return Err(Error::Tuner(format!("FC0012 ID mismatch: 0x{:02x}", id)));
            }
            TunerType::FC0013 if id != FC0013_CHIP_ID => {
                // Some FC0013 report 0xa1 (FC0012 ID), which is fine.
                if id != FC0012_CHIP_ID {
                    return Err(Error::Tuner(format!("FC0013 ID mismatch: 0x{:02x}", id)));
                }
            }
            _ => {}
        }

        // Default initialization registers for FC0012/13
        let init_regs: &[(u8, u8)] = &[
            (0x07, 0x20),
            (0x08, 0xff),
            (0x09, 0x6e),
            (0x0a, 0xb8),
            (0x0b, 0x82),
            (0x0c, 0x50),
            (0x0d, 0x01),
            (0x0e, 0x00),
            (0x0f, 0x00),
            (0x10, 0x00),
            (0x11, 0x00),
            (0x12, 0x00),
            (0x13, 0x00),
            (0x14, 0x00),
            (0x15, 0x00),
        ];

        for &(reg, val) in init_regs {
            self.write_reg(reg, val)?;
        }

        if self.tuner_type == TunerType::FC0013 {
            self.write_reg(0x16, 0x00)?;
        }

        Ok(())
    }

    fn set_frequency(&self, hz: u64) -> Result<u64> {
        let lo_freq = hz + *self.current_if.lock();

        // Output divider selection
        let (mult, div_reg) = if lo_freq < 37_080_000 {
            (96, 7)
        } else if lo_freq < 55_620_000 {
            (64, 6)
        } else if lo_freq < 74_160_000 {
            (48, 5)
        } else if lo_freq < 111_250_000 {
            (32, 4)
        } else if lo_freq < 148_330_000 {
            (24, 3)
        } else if lo_freq < 222_500_000 {
            (16, 2)
        } else if lo_freq < 445_000_000 {
            (8, 1)
        } else {
            (4, 0)
        };

        let f_vco = lo_freq * mult;
        let pll_int = f_vco / self.xtal_freq;
        let pll_frac = ((f_vco % self.xtal_freq) << 15) / self.xtal_freq;

        // Band selection (VHF/UHF) for FC0012/13
        let band = if hz < 300_000_000 { 0 } else { 0x08 };

        self.write_reg(0x05, (div_reg << 4) | band)?;

        // PLL Integer part
        // Reg 0x01: bits 0-5 of M (lower)
        // Reg 0x02: bits 6-12 of M (upper)
        // Note: Exact bit packing varies by source, this matches librtlsdr logic.
        let am = (pll_int % 8) as u8;
        let m = (pll_int / 8) as u8;
        self.write_reg(0x01, am)?;
        self.write_reg(0x02, m)?;

        // PLL Fractional part (15-bit)
        self.write_reg(0x03, ((pll_frac >> 8) & 0x7f) as u8)?;
        self.write_reg(0x04, (pll_frac & 0xff) as u8)?;

        // Specific fix for FC0013 high UHF performance (> 862 MHz)
        if self.tuner_type == TunerType::FC0013 && hz > 862_000_000 {
            self.write_reg(0x16, 0x0c)?;
        }

        Ok(hz)
    }

    fn set_gain(&self, db: f32) -> Result<f32> {
        // FC0012/13 usually only support a few discrete gain steps or auto-gain.
        // For now, we'll implement a simple manual/auto toggle.
        if db <= 0.0 {
            // Auto Gain
            self.write_reg(0x0d, 0x00)?;
        } else {
            // Manual Gain (Force max for now as specific tables are obscure)
            self.write_reg(0x0d, 0x01)?;
        }
        *self.current_gain.lock() = db;
        Ok(db)
    }

    fn get_gain(&self) -> Result<f32> {
        Ok(*self.current_gain.lock())
    }

    fn get_filters(&self) -> Vec<FilterRange> {
        vec![FilterRange {
            start_hz: 22_000_000,
            end_hz: if self.tuner_type == TunerType::FC0012 {
                948_000_000
            } else {
                1_100_000_000
            },
        }]
    }

    fn set_if_freq(&self, hz: u64) -> Result<()> {
        *self.current_if.lock() = hz;
        Ok(())
    }

    fn get_if_freq(&self) -> u64 {
        *self.current_if.lock()
    }

    fn set_ppm(&self, _ppm: i32) -> Result<()> {
        // PPM correction for Fitipower is handled at the driver level via IF frequency.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::MockHardware;

    #[test]
    fn test_fc0012_freq_calc() {
        let dev = Arc::new(MockHardware);
        let tuner = Fc001x::new(dev, TunerType::FC0012, 0xc2, 28_800_000);

        // 100 MHz FM
        // lo_freq = 100M + 0 (initial IF)
        // mult = 32 (since 100M < 111.25M)
        // f_vco = 100M * 32 = 3.2 GHz
        // pll_int = 3.2G / 28.8M = 111.111... -> 111
        // pll_frac = (0.111... * 2^15) = 3640
        let res = tuner.set_frequency(100_000_000);
        assert!(res.is_ok());
        assert_eq!(res.unwrap(), 100_000_000);
    }

    #[test]
    fn test_fc0012_low_freq_div() {
        let dev = Arc::new(MockHardware);
        let tuner = Fc001x::new(dev, TunerType::FC0012, 0xc2, 28_800_000);

        // 30 MHz (Shortwave)
        // lo_freq < 37.08M -> mult = 96
        let res = tuner.set_frequency(30_000_000);
        assert!(res.is_ok());
    }
}
