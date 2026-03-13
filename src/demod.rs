//! RTL2832U baseband / demodulator initialization.

use crate::device::HardwareInterface;
use crate::error::Result;
use crate::registers::{block, demod, sys, usb};
use log::debug;

pub const DEFAULT_SAMPLE_RATE: u32 = 2_048_000;

const FIR_DEFAULT: [i32; 16] = [
    -54, -36, -41, -40, -32, -14, 14, 53, 101, 156, 215, 273, 327, 372, 404, 421,
];

fn pack_fir() -> [u8; 20] {
    let mut fir = [0u8; 20];
    for i in 0..8 {
        fir[i] = FIR_DEFAULT[i] as i8 as u8;
    }
    for i in (0..8).step_by(2) {
        let val0 = FIR_DEFAULT[8 + i];
        let val1 = FIR_DEFAULT[8 + i + 1];
        fir[8 + i * 3 / 2] = (val0 >> 4) as u8;
        fir[8 + i * 3 / 2 + 1] = (((val0 & 0x0f) << 4) | ((val1 >> 8) & 0x0f)) as u8;
        fir[8 + i * 3 / 2 + 2] = (val1 & 0xff) as u8;
    }
    fir
}

fn set_fir(hw: &dyn HardwareInterface) -> Result<()> {
    let fir = pack_fir();
    for (i, &byte) in fir.iter().enumerate() {
        hw.demod_write_reg(demod::P1_PAGE, 0x1c + i as u16, byte)?;
    }
    debug!("FIR filter coefficients written");
    Ok(())
}

pub fn init_baseband(hw: &dyn HardwareInterface) -> Result<()> {
    hw.write_reg(block::USB, usb::SYSCTL, 0x09)?;
    hw.write_reg16(block::USB, usb::EPA_MAXPKT, 0x0002)?;
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;

    hw.write_reg(block::SYS, sys::DEMOD_CTL1, 0x22)?;
    hw.write_reg(block::SYS, sys::DEMOD_CTL, sys::DEMOD_CTL_POWER_ON)?;

    std::thread::sleep(std::time::Duration::from_millis(100));

    hw.demod_write_reg(
        demod::P1_PAGE,
        demod::P1_SOFT_RESET,
        demod::P1_SOFT_RESET_ON,
    )?;
    hw.demod_write_reg(
        demod::P1_PAGE,
        demod::P1_SOFT_RESET,
        demod::P1_SOFT_RESET_OFF,
    )?;

    hw.demod_write_reg(demod::P1_PAGE, 0x15, 0x00)?;
    hw.demod_write_reg16(demod::P1_PAGE, 0x16, 0x0000)?;

    for i in 0..6u16 {
        hw.demod_write_reg(demod::P1_PAGE, 0x16 + i, 0x00)?;
    }

    set_fir(hw)?;

    hw.demod_write_reg(demod::P0_PAGE, 0x19, 0x05)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x93, 0xf0)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x94, 0x0f)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x11, 0x00)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x04, 0x00)?;
    hw.demod_write_reg(demod::P0_PAGE, 0x61, 0x60)?;
    hw.demod_write_reg(demod::P0_PAGE, 0x06, 0x80)?;
    hw.demod_write_reg(demod::P1_PAGE, 0xb1, 0x1b)?;
    hw.demod_write_reg(demod::P0_PAGE, 0x0d, 0x83)?;

    debug!("RTL2832U baseband initialized");
    Ok(())
}

pub fn set_tuner_low_if(hw: &dyn HardwareInterface) -> Result<()> {
    // 1. Disable Zero-IF mode
    hw.demod_write_reg(demod::P1_PAGE, 0xb1, 0x1a)?;
    // 2. Enable In-phase ADC input (required for Low-IF)
    hw.demod_write_reg(demod::P0_PAGE, 0x08, 0x4d)?;
    Ok(())
}

pub fn power_on(hw: &dyn HardwareInterface) -> Result<()> {
    init_baseband(hw)
}
pub fn reset_demod(hw: &dyn HardwareInterface) -> Result<()> {
    hw.demod_write_reg(
        demod::P1_PAGE,
        demod::P1_SOFT_RESET,
        demod::P1_SOFT_RESET_ON,
    )?;
    hw.demod_write_reg(
        demod::P1_PAGE,
        demod::P1_SOFT_RESET,
        demod::P1_SOFT_RESET_OFF,
    )?;
    Ok(())
}

pub fn write_reg_direct(hw: &dyn HardwareInterface, page: u8, addr: u16, val: u8) -> Result<()> {
    hw.demod_write_reg(page, addr, val)
}

pub fn set_if_freq_xtal(hw: &dyn HardwareInterface, if_hz: u32, xtal_hz: u32) -> Result<()> {
    let if_freq = ((if_hz as i64 * (1i64 << 22)) / xtal_hz as i64) * (-1);
    let val = (if_freq & 0x3fffff) as u32;
    hw.demod_write_reg(demod::P1_PAGE, 0x19, ((val >> 16) & 0x3f) as u8)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x1a, ((val >> 8) & 0xff) as u8)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x1b, (val & 0xff) as u8)?;
    debug!(
        "IF freq {}Hz (xtal {}Hz) if_freq_reg={}",
        if_hz, xtal_hz, if_freq
    );
    Ok(())
}

pub fn set_sample_rate_xtal(hw: &dyn HardwareInterface, rate_hz: u32, xtal_hz: u32) -> Result<()> {
    let rsamp_ratio = ((xtal_hz as u64) * (1u64 << 22) / rate_hz as u64) as u32 & 0x0ffffffc;
    let rsamp_ratio = rsamp_ratio | ((rsamp_ratio & 0x08000000) << 1);
    hw.demod_write_reg16(demod::P1_PAGE, 0x9f, (rsamp_ratio >> 16) as u16)?;
    hw.demod_write_reg16(demod::P1_PAGE, 0xa1, (rsamp_ratio & 0xffff) as u16)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x3f, 0x00)?;
    hw.demod_write_reg(demod::P1_PAGE, 0x3e, 0x00)?;
    Ok(())
}

pub fn start_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x0000)?;
    Ok(())
}

pub fn stop_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    hw.write_reg16(block::USB, usb::EPA_CTL, 0x1002)?;
    hw.write_reg(block::SYS, sys::DEMOD_CTL, sys::DEMOD_CTL_POWER_OFF)?;
    Ok(())
}
