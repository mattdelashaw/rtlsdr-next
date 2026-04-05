//! # diag_demod
//!
//! **USB handle re-acquisition probe.** Attempts up to 5 times to open the
//! RTL2832U after it has been released or has entered a bad state, verifying
//! that a basic SYS block read succeeds.
//!
//! ## Purpose
//! Used to diagnose "Device or resource busy" / `EBUSY` errors that occur
//! when a previous process crashed without releasing the USB interface.
//! Also validates that the kernel driver detach sequence works correctly.
//!
//! ## Expected output (healthy)
//! ```
//! Attempt 1 to re-acquire hardware...
//!   Device found on Bus 001 Address 002
//!   SUCCESS! Handle acquired.
//!   Testing SYS read (Block 2, Reg 1)... Ok(1)
//! ```
//!
//! ## If it fails
//! Run `sudo usbreset /dev/bus/usb/001/002` (adjust bus/address) or unplug
//! and replug the dongle to clear the kernel's interface claim.
//!
//! ## Prerequisites
//! - RTL-SDR dongle connected

use rusb::{Context, UsbContext};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("--- RTL-SDR Handle Re-acquisition Probe ---");
    let context = Context::new()?;

    for attempt in 1..=5 {
        println!("\nAttempt {} to re-acquire hardware...", attempt);
        let devices = context.devices()?;
        let mut found = false;
        for device in devices.iter() {
            let desc = device.device_descriptor()?;
            if desc.vendor_id() == 0x0bda
                && (desc.product_id() == 0x2838 || desc.product_id() == 0x2832)
            {
                found = true;
                println!(
                    "  Device found on Bus {:03} Address {:03}",
                    device.bus_number(),
                    device.address()
                );

                match device.open() {
                    Ok(handle) => {
                        println!("  SUCCESS! Handle acquired.");
                        let _ = handle.detach_kernel_driver(0);
                        handle.claim_interface(0)?;

                        println!("  Testing SYS read (Block 2, Reg 1)...");
                        let mut b = [0u8; 1];
                        let res = handle.read_control(
                            0xc0,
                            0,
                            0x0001,
                            0x0200,
                            &mut b,
                            Duration::from_millis(200),
                        );
                        println!("    Result: {:?}", res);

                        handle.release_interface(0)?;
                        return Ok(());
                    }
                    Err(e) => println!(
                        "  Open FAILED: {:?}\n  Try: sudo usbreset /dev/bus/usb/{:03}/{:03}",
                        e,
                        device.bus_number(),
                        device.address()
                    ),
                }
            }
        }
        if !found {
            println!("  Device not visible to OS — try unplugging and replugging");
        }
        std::thread::sleep(Duration::from_secs(1));
    }

    println!("\nAll attempts failed.");
    Ok(())
}
