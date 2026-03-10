//! RTL2832U baseband / demodulator initialization.
//!
//! All sequences match librtlsdr rtl-sdr-blog fork exactly.
//! Source: rtlsdr_init_baseband() in librtlsdr.c

use crate::device::HardwareInterface;
use crate::error::Result;
use crate::registers::{block, usb, sys, demod, if_freq_regs, resample_regs, IF_FREQ_HZ, XTAL_FREQ_HZ};
use log::debug;

pub const DEFAULT_SAMPLE_RATE: u32 = 2_048_000;

/// FIR default coefficients — DAB/FM Windows driver defaults.
/// Format: int8_t[8] followed by int12_t[8].
/// Source: fir_default[] in librtlsdr.c
const FIR_DEFAULT: [i32; 16] = [
    -54, -36, -41, -40, -32, -14, 14, 53,    // 8-bit signed
    101, 156, 215, 273, 327, 372, 404, 421,   // 12-bit signed
];

/// Pack the FIR coefficients into the 20-byte on-wire format used by the RTL2832U.
/// First 8 coefficients are int8, next 8 are int12 packed as described in librtlsdr.
fn pack_fir() -> [u8; 20] {
    let mut fir = [0u8; 20];
    // First 8: int8_t directly
    for i in 0..8 {
        fir[i] = FIR_DEFAULT[i] as i8 as u8;
    }
    // Next 8: int12_t packed two per three bytes
    for i in (0..8).step_by(2) {
        let val0 = FIR_DEFAULT[8 + i];
        let val1 = FIR_DEFAULT[8 + i + 1];
        fir[8 + i * 3 / 2]     = (val0 >> 4) as u8;
        fir[8 + i * 3 / 2 + 1] = (((val0 & 0x0f) << 4) | ((val1 >> 8) & 0x0f)) as u8;
        fir[8 + i * 3 / 2 + 2] = (val1 & 0xff) as u8;
    }
    fir
}

/// Write the FIR filter coefficients to demod page 1, regs 0x1c..=0x2f (20 bytes).
fn set_fir(hw: &dyn HardwareInterface) -> Result<()> {
    let fir = pack_fir();
    for (i, &byte) in fir.iter().enumerate() {
        hw.demod_write_reg(demod::P1_PAGE, 0x1c + i as u16, byte)?;
    }
    debug!("FIR filter coefficients written");
    Ok(())
}

/// Full RTL2832U baseband initialization — matches rtlsdr_init_baseband() exactly.
pub fn init_baseband(hw: &dyn HardwareInterface) -> Result<()> {
    // Initialize USB
    log::trace!("init_baseband: init USB SIE");
    hw.write_reg(block::USB, usb::SYSCTL, 0x09)?;
    // EPA_MAXPKT: 0x0002 — matches librtlsdr rtlsdr_init_baseband()
    hw.write_reg16(block::USB, usb::EPA_MAXPKT, 0x0002)?;
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;

    // Power on demod
    log::trace!("init_baseband: power on demod");
    hw.write_reg(block::SYS, sys::DEMOD_CTL1, 0x22)?;
    hw.write_reg(block::SYS, sys::DEMOD_CTL, sys::DEMOD_CTL_POWER_ON)?;

    log::trace!("init_baseband: settling after power-on...");
    std::thread::sleep(std::time::Duration::from_millis(100));

    // Reset demod (soft_rst)
    log::trace!("init_baseband: soft reset");
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_ON)?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_OFF)?;

    // Disable spectrum inversion and adjacent channel rejection
    hw.demod_write_reg(demod::P1_PAGE, 0x15, 0x00)?;
    hw.demod_write_reg16(demod::P1_PAGE, 0x16, 0x0000)?;

    // Clear DDC shift and IF frequency registers (regs 0x16..0x1b)
    for i in 0..6u16 {
        hw.demod_write_reg(demod::P1_PAGE, 0x16 + i, 0x00)?;
    }

    // Write FIR filter coefficients
    set_fir(hw)?;

    // Enable SDR mode, disable DAGC (bit 5) — page 0, reg 0x19
    hw.demod_write_reg(demod::P0_PAGE, 0x19, 0x05)?;

    // Init FSM state-holding registers
    hw.demod_write_reg(demod::P1_PAGE, 0x93, 0xf0)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x94, 0x0f)?;

    // Disable AGC (en_dagc, bit 0)
    hw.demod_write_reg(demod::P1_PAGE, 0x11, 0x00)?;

    // Disable RF and IF AGC loop
    hw.demod_write_reg(demod::P1_PAGE, 0x04, 0x00)?;

    // Disable PID filter
    hw.demod_write_reg(demod::P0_PAGE, 0x61, 0x60)?;

    // opt_adc_iq = 0, default ADC_I/ADC_Q datapath
    hw.demod_write_reg(demod::P0_PAGE, 0x06, 0x80)?;

    // Enable Zero-IF mode, DC cancellation, IQ estimation/compensation
    hw.demod_write_reg(demod::P1_PAGE, 0xb1, 0x1b)?;

    // Disable 4.096 MHz clock output on TP_CK0
    hw.demod_write_reg(demod::P0_PAGE, 0x0d, 0x83)?;

    debug!("RTL2832U baseband initialized");
    Ok(())
}

/// Aliases used by lib.rs
pub fn power_on(hw: &dyn HardwareInterface) -> Result<()> {
    log::info!("Starting init_baseband");
    let r = init_baseband(hw);
    log::info!("init_baseband result: {:?}", r);
    r
}
pub fn init_registers(hw: &dyn HardwareInterface) -> Result<()> { Ok(()) }  // folded into init_baseband
pub fn reset_demod(hw: &dyn HardwareInterface) -> Result<()> { reset(hw) }

/// Direct single demod register write — used for post-detection config.
pub fn write_reg_direct(hw: &dyn HardwareInterface, page: u8, addr: u16, val: u8) -> Result<()> {
    hw.demod_write_reg(page, addr, val)
}

/// Soft-reset the demodulator.
pub fn reset(hw: &dyn HardwareInterface) -> Result<()> {
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_ON)?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_SOFT_RESET, demod::P1_SOFT_RESET_OFF)?;
    debug!("Demod soft-reset");
    Ok(())
}

/// Program the IF frequency registers (uses default XTAL).
pub fn set_if_freq(hw: &dyn HardwareInterface) -> Result<()> {
    set_if_freq_xtal(hw, IF_FREQ_HZ as u32, XTAL_FREQ_HZ)
}

/// Program the IF frequency registers with explicit (PPM-corrected) xtal.
pub fn set_if_freq_xtal(hw: &dyn HardwareInterface, if_hz: u32, xtal_hz: u32) -> Result<()> {
    let regs = if_freq_regs(if_hz, xtal_hz);
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_H, regs[0])?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_M, regs[1])?;
    hw.demod_write_reg(demod::P1_PAGE, demod::P1_IF_FREQ_L, regs[2])?;
    debug!("IF freq {}Hz (xtal {}Hz)", if_hz, xtal_hz);
    Ok(())
}

/// Program the resampler (uses default XTAL).
pub fn set_sample_rate(hw: &dyn HardwareInterface, rate_hz: u32) -> Result<()> {
    set_sample_rate_xtal(hw, rate_hz, XTAL_FREQ_HZ)
}

/// Program the resampler with explicit (PPM-corrected) xtal.
pub fn set_sample_rate_xtal(hw: &dyn HardwareInterface, rate_hz: u32, xtal_hz: u32) -> Result<()> {
    let regs = resample_regs(rate_hz, xtal_hz);
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_H,   regs[0])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_M,   regs[1])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_L,   regs[2])?;
    hw.demod_write_reg(demod::P2_PAGE, demod::P2_RESAMPLE_LSB, regs[3])?;
    debug!("Sample rate {}Hz (xtal {}Hz)", rate_hz, xtal_hz);
    Ok(())
}

/// Flush EPA FIFO and start USB streaming — matches rtlsdr_reset_buffer().
pub fn start_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    // Stall then unstall the EPA endpoint — matches librtlsdr rtlsdr_reset_buffer():
    //   write_reg(USBB, USB_EPA_CTL, 0x1002, 2)  // stall
    //   write_reg(USBB, USB_EPA_CTL, 0x0000, 2)  // unstall
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x0000)?;
    debug!("EPA FIFO flushed, streaming started");
    Ok(())
}

/// Stop streaming: stall EPA and power down.
pub fn stop_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;
    hw.write_reg(block::SYS, sys::DEMOD_CTL, sys::DEMOD_CTL_POWER_OFF)?;
    debug!("Streaming stopped");
    Ok(())
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    struct MockHw {
        writes:       Mutex<Vec<(u16, u16, u16)>>,
        demod_writes: Mutex<Vec<(u8, u16, u8)>>,
    }

    impl MockHw {
        fn new() -> Self {
            Self {
                writes:       Mutex::new(vec![]),
                demod_writes: Mutex::new(vec![]),
            }
        }
        fn wrote(&self, blk: u16, addr: u16, val: u16) -> bool {
            self.writes.lock().unwrap().iter().any(|&(b,a,v)| b==blk && a==addr && v==val)
        }
        fn demod_wrote(&self, page: u8, addr: u16, val: u8) -> bool {
            self.demod_writes.lock().unwrap().iter().any(|&(p,a,v)| p==page && a==addr && v==val)
        }
    }

    impl HardwareInterface for MockHw {
        fn read_reg(&self, _: u16, _: u16) -> Result<u8> { Ok(0) }
        fn write_reg(&self, b: u16, a: u16, v: u8) -> Result<()> {
            self.writes.lock().unwrap().push((b, a, v as u16)); Ok(())
        }
        fn write_reg16(&self, b: u16, a: u16, v: u16) -> Result<()> {
            self.writes.lock().unwrap().push((b, a, v)); Ok(())
        }
        fn demod_read_reg(&self, _: u8, _: u16) -> Result<u8> { Ok(0) }
        fn demod_write_reg(&self, p: u8, a: u16, v: u8) -> Result<()> {
            self.demod_writes.lock().unwrap().push((p, a, v)); Ok(())
        }
        fn demod_write_reg16(&self, p: u8, a: u16, v: u16) -> Result<()> {
            self.demod_writes.lock().unwrap().push((p, a, (v>>8) as u8));
            self.demod_writes.lock().unwrap().push((p, a, (v&0xff) as u8));
            Ok(())
        }
        fn i2c_write_tuner(&self, _: u8, _: u8, _: &[u8]) -> Result<()> { Ok(()) }
        fn i2c_read_tuner(&self, _: u8, _: u8, l: usize) -> Result<Vec<u8>> { Ok(vec![0;l]) }
        fn i2c_read_raw(&self, _: u8, l: usize) -> Result<Vec<u8>> { Ok(vec![0;l]) }
        fn read_bulk(&self, _: u8, b: &mut [u8], _: std::time::Duration) -> Result<usize> { Ok(b.len()) }
        fn set_gpio_output(&self, _: u8) -> Result<()> { Ok(()) }
        fn set_gpio_bit(&self, _: u8, _: bool) -> Result<()> { Ok(()) }
        fn probe_tuner(&self) -> Result<crate::tuner::TunerType> { Ok(crate::tuner::TunerType::Unknown(0)) }
    }

    #[test]
    fn test_init_baseband_usb_sequence() {
        let hw = MockHw::new();
        init_baseband(&hw).unwrap();
        assert!(hw.wrote(block::USB, usb::SYSCTL,    0x09));
        assert!(hw.wrote(block::USB, usb::EPA_MAXPKT, 0x0002));
        assert!(hw.wrote(block::USB, usb::EPA_CTL,   0x1002));
        assert!(hw.wrote(block::SYS, sys::DEMOD_CTL1, 0x22));
        assert!(hw.wrote(block::SYS, sys::DEMOD_CTL, 0xe8));
    }

    #[test]
    fn test_init_baseband_sdr_mode() {
        let hw = MockHw::new();
        init_baseband(&hw).unwrap();
        // Enable SDR mode on page 0, reg 0x19
        assert!(hw.demod_wrote(demod::P0_PAGE, 0x19, 0x05));
        // Zero-IF + DC cancel + IQ comp on page 1, reg 0xb1
        assert!(hw.demod_wrote(demod::P1_PAGE, 0xb1, 0x1b));
        // FSM registers
        assert!(hw.demod_wrote(demod::P1_PAGE, 0x93, 0xf0));
        assert!(hw.demod_wrote(demod::P1_PAGE, 0x94, 0x0f));
    }

    #[test]
    fn test_pack_fir_length() {
        let fir = pack_fir();
        assert_eq!(fir.len(), 20);
        // First 8 bytes are the int8 coefficients directly
        assert_eq!(fir[0], (-54i8) as u8);
        assert_eq!(fir[7], 53u8);
    }

    #[test]
    fn test_start_streaming() {
        let hw = MockHw::new();
        start_streaming(&hw).unwrap();
        assert!(hw.wrote(block::USB, usb::EPA_CTL, 0x1002));
        assert!(hw.wrote(block::USB, usb::EPA_CTL, 0x0000));
    }
}
