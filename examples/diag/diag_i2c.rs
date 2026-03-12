//! # diag_i2c
//!
//! **I2C repeater and demod register pattern probe.** Powers up the RTL2832U,
//! brute-forces the correct read/write encoding for demod registers, and
//! validates the dummy-read flush sequence required after every demod write.
//!
//! ## Purpose
//! This tool was used to discover that:
//! 1. Demod reads use `wValue = (reg << 8) | 0x20`, `wIndex = page` (Pattern A)
//! 2. Every demod write must be followed by a dummy read of page 0x0a reg 0x01
//!    or subsequent transfers stall with a Pipe error.
//!
//! ## Expected output (healthy V4)
//! ```
//! [SUCCESS] Read Pattern A (wValue=(reg<<8)|0x20) works! Reg 0x01
//! Write... OK
//! Immediate Read back from Block 0... Ok(1)
//! ```
//!
//! ## Prerequisites
//! - RTL-SDR dongle connected and not claimed by another process

use rusb::{Context, UsbContext};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("--- RTL-SDR I2C / Demod Pattern Probe ---");
    let context = Context::new()?;
    let devices = context.devices()?;
    let mut handle = None;
    for device in devices.iter() {
        let desc = device.device_descriptor()?;
        if desc.vendor_id() == 0x0bda && (desc.product_id() == 0x2838 || desc.product_id() == 0x2832) {
            handle = Some(device.open()?);
            break;
        }
    }
    let handle = handle.ok_or_else(|| anyhow::anyhow!("No RTL2832U device found"))?;
    let _ = handle.detach_kernel_driver(0);
    handle.claim_interface(0)?;

    println!("1. Initializing USB SIE (Block 1)...");
    handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100))?;
    std::thread::sleep(Duration::from_millis(100));

    println!("2. Enabling Demod Power via SYS (Block 2)...");
    handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(100))?;
    handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(100))?;
    std::thread::sleep(Duration::from_millis(100));

    println!("3. Probing demod read encoding (Pattern A vs B)...");
    let mut working_read = None;
    for reg in 0..256u16 {
        let mut b = [0u8; 1];
        // Pattern A: wValue = (reg << 8) | 0x20, wIndex = page — this is correct
        if handle.read_control(0xc0, 0, (reg << 8) | 0x20, 1, &mut b, Duration::from_millis(10)).is_ok() {
            println!("   [SUCCESS] Pattern A works at reg 0x{:02x} — this is the correct encoding", reg);
            working_read = Some('A');
            break;
        }
        // Pattern B: wValue = reg, wIndex = page — legacy/wrong
        if handle.read_control(0xc0, 0, reg, 1, &mut b, Duration::from_millis(10)).is_ok() {
            println!("   [SUCCESS] Pattern B works at reg 0x{:02x} — unexpected", reg);
            working_read = Some('B');
            break;
        }
    }
    if working_read.is_none() {
        println!("   [FAIL] No read pattern worked — demod not responding");
    }

    println!("4. Testing demod write + dummy read flush (reg 0x19 = SDR mode enable)...");
    let reg: u16 = 0x19;
    let val: u8  = 0x05;
    let wvalue   = (reg << 8) | 0x20;
    let windex   = 0x10u16; // page 0

    println!("   Writing page0 reg 0x19 = 0x05 (enable SDR mode)...");
    let wr = handle.write_control(0x40, 0, wvalue, windex, &[val], Duration::from_millis(100));
    println!("   Write result: {:?}", wr);

    // Dummy read — required after every demod write to flush the hardware
    let mut b = [0u8; 1];
    let dr = handle.read_control(0xc0, 0, (0x01u16 << 8) | 0x20, 0x000a, &mut b, Duration::from_millis(100));
    println!("   Dummy read flush result: {:?}", dr);

    Ok(())
}
