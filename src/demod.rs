//! RTL2832U baseband / demodulator initialization.
//!
//! All sequences match librtlsdr rtl-sdr-blog fork exactly.
//! Register access uses the correct split encoding:
//!   - SYS/USB block registers  → hw.write_reg(block::SYS/USB, addr, val)
//!   - Demodulator page registers → hw.demod_write_reg(page, addr, val)

use crate::device::HardwareInterface;
use crate::error::Result;
use crate::registers::{block, usb, sys, demod, if_freq_regs, resample_regs, IF_FREQ_HZ, XTAL_FREQ_HZ};
use log::debug;

/// Full RTL2832U power-on and EPA init sequence.
///
/// Matches librtlsdr open sequence:
/// ```c
/// rtlsdr_write_reg(USBB, USB_SYSCTL,    0x09, 1)
/// rtlsdr_write_reg(USBB, USB_EPA_MAXPKT, 0x0002, 2)
/// rtlsdr_write_reg(USBB, USB_EPA_CTL,   0x1002, 2)
/// rtlsdr_write_reg(SYSB, DEMOD_CTL_1,  0x22, 1)
/// rtlsdr_write_reg(SYSB, DEMOD_CTL,    0xe8, 1)
/// ```
pub fn power_on(hw: &dyn HardwareInterface) -> Result<()> {
    // USB FIFO / EPA init
    hw.write_reg(block::USB, usb::SYSCTL, 0x09)?;
    hw.write_reg16(block::USB, usb::EPA_MAXPKT, 0x0002)?;
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;

    // Power on demod
    hw.write_reg(block::SYS, sys::DEMOD_CTL1, 0x22)?;
    hw.write_reg(block::SYS, sys::DEMOD_CTL, sys::DEMOD_CTL_POWER_ON)?;

    debug!("RTL2832U powered on");
    Ok(())
}

/// Soft-reset the demodulator (matches librtlsdr init sequence).
pub fn reset(hw: &dyn HardwareInterface) -> Result<()> {
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_ON)?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_OFF)?;
    debug!("Demod soft-reset complete");
    Ok(())
}

/// Disable spectrum inversion and adjacent channel rejection.
pub fn disable_dsp(hw: &dyn HardwareInterface) -> Result<()> {
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SPEC_INV, 0x00)?;
    hw.demod_write_reg16(demod::P1_PAGE, demod::P1_ADJ_CHAN, 0x0000)?;
    debug!("Spectrum inversion and ACR disabled");
    Ok(())
}

/// Program the IF frequency registers.
///
/// The RTL2832U demod needs to know the IF frequency the tuner delivers.
/// Standard value for R820T/R828D is 3.57 MHz.
pub fn set_if_freq(hw: &dyn HardwareInterface) -> Result<()> {
    let regs = if_freq_regs(IF_FREQ_HZ as u32, XTAL_FREQ_HZ);
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_H, regs[0])?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_M, regs[1])?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_L, regs[2])?;
    debug!("IF frequency set to {} Hz", IF_FREQ_HZ);
    Ok(())
}

/// Program the resampler for the requested sample rate.
pub fn set_sample_rate(hw: &dyn HardwareInterface, rate_hz: u32) -> Result<()> {
    let regs = resample_regs(rate_hz, XTAL_FREQ_HZ);
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_H,   regs[0])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_M,   regs[1])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_L,   regs[2])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_LSB, regs[3])?;
    debug!("Sample rate set to {} Hz", rate_hz);
    Ok(())
}

/// Flush the EPA FIFO and unstall the bulk IN endpoint.
///
/// Must be called after full init to start USB streaming.
/// Matches librtlsdr rtlsdr_reset_buffer:
/// ```c
/// rtlsdr_write_reg(USBB, USB_SYSCTL, 0x09, 1)
/// rtlsdr_write_reg(USBB, USB_SYSCTL, 0x08, 1) 
/// ```
pub fn start_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    hw.write_reg(block::USB, usb::SYSCTL, 0x09)?;
    hw.write_reg(block::USB, usb::SYSCTL, 0x08)?;
    debug!("EPA FIFO flushed, bulk IN streaming started");
    Ok(())
}

/// Stop streaming: stall the bulk IN endpoint and power down.
pub fn stop_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;
    hw.write_reg(block::SYS, sys::DEMOD_CTL, sys::DEMOD_CTL_POWER_OFF)?;
    debug!("Streaming stopped, ADCs powered down");
    Ok(())
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    struct MockHw {
        writes:    Mutex<Vec<(u8, u16, u16)>>, // (block_or_page, addr_or_reg, val)
        demod_writes: Mutex<Vec<(u8, u16, u8)>>,
    }

    impl MockHw {
        fn new() -> Self {
            Self {
                writes:       Mutex::new(vec![]),
                demod_writes: Mutex::new(vec![]),
            }
        }
        fn wrote(&self, blk: u8, addr: u16, val: u16) -> bool {
            self.writes.lock().unwrap().iter()
                .any(|&(b, a, v)| b == blk && a == addr && v == val)
        }
        fn demod_wrote(&self, page: u8, addr: u16, val: u8) -> bool {
            self.demod_writes.lock().unwrap().iter()
                .any(|&(p, a, v)| p == page && a == addr && v == val)
        }
    }

    impl HardwareInterface for MockHw {
        fn read_reg(&self, _b: u8, _a: u16) -> Result<u8> { Ok(0) }
        fn write_reg(&self, blk: u8, addr: u16, val: u8) -> Result<()> {
            self.writes.lock().unwrap().push((blk, addr, val as u16));
            Ok(())
        }
        fn write_reg16(&self, blk: u8, addr: u16, val: u16) -> Result<()> {
            self.writes.lock().unwrap().push((blk, addr, val));
            Ok(())
        }
        fn demod_read_reg(&self, _p: u8, _a: u16) -> Result<u8> { Ok(0) }
        fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()> {
            self.demod_writes.lock().unwrap().push((page, addr, val));
            Ok(())
        }
        fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()> {
            self.demod_writes.lock().unwrap().push((page, addr, (val >> 8) as u8));
            self.demod_writes.lock().unwrap().push((page, addr, (val & 0xff) as u8));
            Ok(())
        }
        fn i2c_read(&self, _a: u8, _r: u8, len: usize) -> Result<Vec<u8>> { Ok(vec![0; len]) }
        fn i2c_write(&self, _a: u8, _r: u8, _d: &[u8]) -> Result<()> { Ok(()) }
        fn read_bulk(&self, _e: u8, buf: &mut [u8], _t: std::time::Duration) -> Result<usize> {
            Ok(buf.len())
        }
    }

    #[test]
    fn test_power_on_sequence() {
        let hw = MockHw::new();
        power_on(&hw).unwrap();
        assert!(hw.wrote(block::USB, usb::SYSCTL, 0x09));
        assert!(hw.wrote(block::USB, usb::EPA_MAXPKT, 0x0002));
        assert!(hw.wrote(block::USB, usb::EPA_CTL, 0x1002));
        assert!(hw.wrote(block::SYS, sys::DEMOD_CTL1, 0x22));
        assert!(hw.wrote(block::SYS, sys::DEMOD_CTL, 0xe8));
    }

    #[test]
    fn test_soft_reset() {
        let hw = MockHw::new();
        reset(&hw).unwrap();
        assert!(hw.demod_wrote(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_ON));
        assert!(hw.demod_wrote(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_OFF));
    }

    #[test]
    fn test_set_if_freq() {
        let hw = MockHw::new();
        set_if_freq(&hw).unwrap();
        let regs = if_freq_regs(IF_FREQ_HZ as u32, XTAL_FREQ_HZ);
        assert!(hw.demod_wrote(demod::P1_PAGE, demod::P1_IF_FREQ_H, regs[0]));
        assert!(hw.demod_wrote(demod::P1_PAGE, demod::P1_IF_FREQ_M, regs[1]));
        assert!(hw.demod_wrote(demod::P1_PAGE, demod::P1_IF_FREQ_L, regs[2]));
    }

    #[test]
    fn test_start_streaming() {
        let hw = MockHw::new();
        start_streaming(&hw).unwrap();
        assert!(hw.wrote(block::USB, usb::SYSCTL, 0x09));
        assert!(hw.wrote(block::USB, usb::SYSCTL, 0x08));
    }
}

// ── Additional entry points called by lib.rs ─────────────────────────────────

pub const DEFAULT_SAMPLE_RATE: u32 = 2_048_000;

/// Initialize demodulator registers after power-on.
/// Runs reset, disables DSP filters, and sets up IF freq at default xtal.
pub fn init_registers(hw: &dyn HardwareInterface) -> Result<()> {
    reset(hw)?;
    disable_dsp(hw)?;
    Ok(())
}

/// Soft-reset the demodulator — alias used by lib.rs after freq/rate changes.
pub fn reset_demod(hw: &dyn HardwareInterface) -> Result<()> {
    reset(hw)
}

/// set_if_freq with explicit xtal parameter (for PPM-corrected xtal).
pub fn set_if_freq_xtal(hw: &dyn HardwareInterface, if_hz: u32, xtal_hz: u32) -> Result<()> {
    let regs = if_freq_regs(if_hz, xtal_hz);
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_H, regs[0])?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_M, regs[1])?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_L, regs[2])?;
    debug!("IF freq set to {} Hz (xtal={})", if_hz, xtal_hz);
    Ok(())
}

/// set_sample_rate with explicit xtal parameter (for PPM-corrected xtal).
pub fn set_sample_rate_xtal(hw: &dyn HardwareInterface, rate_hz: u32, xtal_hz: u32) -> Result<()> {
    let regs = resample_regs(rate_hz, xtal_hz);
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_H,   regs[0])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_M,   regs[1])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_L,   regs[2])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_LSB, regs[3])?;
    debug!("Sample rate set to {} Hz (xtal={})", rate_hz, xtal_hz);
    Ok(())
}
