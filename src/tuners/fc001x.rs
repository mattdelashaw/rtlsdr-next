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

        // FC001x uses multipliers of 2 or 4 only, derived from xtal/2.
        // Divider and reg[5] values from Osmocom tuner_fc001x.c reference.
        let (multi, reg5) = match self.tuner_type {
            TunerType::FC0012 => {
                if lo_freq < 300_000_000      { (32u64, 0x08u8) }
                else if lo_freq < 862_000_000  { (16,    0x00) }
                else                           { ( 4,    0x0a) }
            }
            _ /* FC0013 */ => {
                if lo_freq < 300_000_000      { (32u64, 0x08u8) }
                else if lo_freq < 862_000_000  { (16,    0x00) }
                else if lo_freq < 948_600_000  { ( 4,    0x12) }
                else                           { ( 2,    0x0a) }
            }
        };

        // reg[6]: VCO select + integer/fractional mode
        let f_vco = lo_freq * multi;
        let mut reg6: u8 = if (multi % 3) == 0 { 0x00 } else { 0x02 };
        if f_vco >= 3_060_000_000 {
            reg6 |= 0x08; // high VCO select
        }

        // xdiv = f_vco / (xtal / 2), rounded
        let xtal_div2 = self.xtal_freq / 2;
        let mut xdiv = (f_vco / xtal_div2) as u16;
        if (f_vco - xdiv as u64 * xtal_div2) >= (xtal_div2 / 2) {
            xdiv += 1; // round up
        }

        // am = xdiv % 8, pm = xdiv / 8
        let mut pm = (xdiv / 8) as u8;
        let mut am = (xdiv - 8 * pm as u16) as u8;

        // am must be >= 2; if not, borrow from pm
        if am < 2 {
            am += 8;
            pm = pm.saturating_sub(1);
        }

        // pm overflow: fold excess into am
        let (reg1, reg2) = if pm > 31 {
            (am + 8 * (pm - 31), 31u8)
        } else {
            (am, pm)
        };

        // Validity check — matches Osmocom reference
        if reg1 > 15 || reg2 < 0x0b {
            return Err(Error::InvalidFrequency(hz));
        }

        self.write_reg(0x05, reg5)?;
        self.write_reg(0x06, reg6)?;
        self.write_reg(0x01, reg1)?;
        self.write_reg(0x02, reg2)?;

        // FC0013 high-UHF tweak (> 862 MHz)
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
    fn test_fc0012_vhf() {
        let dev = Arc::new(MockHardware);
        let tuner = Fc001x::new(dev, TunerType::FC0012, 0xc2, 28_800_000);
        // 100 MHz FM: multi=32, f_vco=3.2GHz, xdiv=3200M/14.4M=222
        // pm=27, am=6 — valid (am>=2, pm<=31)
        let res = tuner.set_frequency(100_000_000);
        assert!(res.is_ok());
        assert_eq!(res.unwrap(), 100_000_000);
    }

    #[test]
    fn test_fc0012_uhf() {
        let dev = Arc::new(MockHardware);
        let tuner = Fc001x::new(dev, TunerType::FC0012, 0xc2, 28_800_000);
        // 434 MHz: multi=16, f_vco=6.944GHz — should be ok
        let res = tuner.set_frequency(434_000_000);
        assert!(res.is_ok());
    }

    #[test]
    fn test_fc0013_high_uhf() {
        let dev = Arc::new(MockHardware);
        let tuner = Fc001x::new(dev, TunerType::FC0013, 0xc6, 28_800_000);
        // 900 MHz: FC0013-specific path
        let res = tuner.set_frequency(900_000_000);
        assert!(res.is_ok());
    }
}
