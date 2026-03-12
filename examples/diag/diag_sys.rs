//! # diag_sys
//!
//! **RTL2832U register block dump.** Opens the device raw (no driver), scans
//! USB/SYS/DEMOD blocks using both read encodings, then replays a minimal
//! power-up sequence to verify the hardware responds correctly.
//!
//! ## Purpose
//! Low-level register visibility tool. Use this when you suspect a control
//! transfer encoding issue or want to verify what the hardware returns for
//! specific registers before the driver touches anything.
//!
//! Scans blocks 1 (USB), 2 (SYS), and 4 (I2C bridge) using both:
//! - Pattern B: `wValue = reg` (regular registers)
//! - Pattern C: `wValue = (reg << 8) | 0x20` (demod paged registers)
//!
//! ## Expected output (healthy)
//! Most registers return 0x00 or a non-error value. Pipe errors on
//! non-existent registers are normal.
//!
//! ## Prerequisites
//! - RTL-SDR dongle connected and not claimed by another process

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

    // Minimal power-up so registers are readable
    handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100))?;
    handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(100))?;
    handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(100))?;
    std::thread::sleep(Duration::from_millis(100));

    for blk in [1u16, 2, 4] {
        println!("\n--- RTL2832U Block {} Register Dump ---", blk);
        for reg in 0..0x10u16 {
            let mut b = [0u8; 1];

            // Pattern B: regular register read (wValue = reg, wIndex = block << 8)
            let res_b = handle.read_control(0xc0, 0, reg, blk << 8, &mut b, Duration::from_millis(50));
            let val_b = res_b.map(|_| format!("0x{:02x}", b[0]))
                             .unwrap_or_else(|e| format!("Err({:?})", e));

            // Pattern C: demod paged register read (wValue = (reg << 8) | 0x20)
            let res_c = handle.read_control(0xc0, 0, (reg << 8) | 0x20, blk << 8, &mut b, Duration::from_millis(50));
            let val_c = res_c.map(|_| format!("0x{:02x}", b[0]))
                             .unwrap_or_else(|e| format!("Err({:?})", e));

            println!("  Reg 0x{:02x}:  Regular={}  Demod-paged={}", reg, val_b, val_c);
        }
    }

    Ok(())
}
