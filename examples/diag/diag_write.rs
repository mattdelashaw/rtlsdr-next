//! # diag_write
//!
//! **Raw USB block scanner.** Powers up the RTL2832U and scans all 8 USB
//! control blocks (0–7) for a response to register 0x01.
//!
//! ## Purpose
//! Used to confirm which USB blocks the hardware actually responds to, and
//! to verify basic control transfer encoding before the driver is involved.
//! Useful when debugging a "Pipe error" on demod register writes.
//!
//! ## Expected output (healthy V4)
//! ```
//! Block 0 (wIndex=0x0000) -> Ok(1)
//! Block 1 (wIndex=0x0100) -> Ok(1)   ← demod responds here
//! Block 2 (wIndex=0x0200) -> Ok(1)   ← SYS responds here
//! Block 3..7 -> Err(Pipe)
//! ```
//!
//! ## Prerequisites
//! - RTL-SDR dongle connected and not claimed by another process
//! - USB permissions (udev rule or run with sudo)

use rusb::{Context, UsbContext};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    let context = Context::new()?;
    let devices = context.devices()?;
    let mut handle = None;
    for device in devices.iter() {
        let desc = device.device_descriptor()?;
        if desc.vendor_id() == 0x0bda && desc.product_id() == 0x2838 {
            handle = Some(device.open()?);
            break;
        }
    }
    let handle = handle.ok_or_else(|| anyhow::anyhow!("No RTL2832U device found (0x0bda:0x2838)"))?;
    let _ = handle.detach_kernel_driver(0);
    handle.claim_interface(0)?;

    // Power up USB SIE and demod
    handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100))?;
    handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(100))?;
    handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(100))?;
    std::thread::sleep(Duration::from_millis(100));

    println!("Scanning all blocks 0..7 for Reg 0x01 response...");
    for blk in 0..8u16 {
        let mut b = [0u8; 1];
        let res = handle.read_control(0xc0, 0, 0x0001, blk << 8, &mut b, Duration::from_millis(50));
        println!("  Block {} (wIndex=0x{:04x}) -> {:?}", blk, blk << 8, res);
    }

    Ok(())
}
