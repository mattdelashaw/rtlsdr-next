//! # diag_raw_clone
//!
//! **Raw V4 initialization sequence validator.** Resets the USB device,
//! reclaims the interface, and replays the exact control transfer sequence
//! used during RTL-SDR Blog V4 bring-up — without going through the driver.
//!
//! ## Purpose
//! The definitive "is it the hardware or the driver?" test. If this succeeds
//! but `Driver::new()` fails, the bug is in the driver. If this fails, it's
//! a USB permissions, kernel driver, or hardware issue.
//!
//! Specifically validates:
//! - USB device reset and reclaim sequence
//! - USB SIE init (`USB_SYSCTL = 0x09`)
//! - Demod power-on via SYS block
//! - Demod soft-reset write (`page1 reg 0x01 = 0x14`) with dummy read flush
//!
//! ## Expected output (healthy V4)
//! ```
//! Step 0: The librtlsdr 'Connectivity Check' (USB_SYSCTL=0x09) -> Ok(1)
//! Step 4: The Critical Test (Demod Soft Reset) -> Ok(1)
//! [SUCCESS] Hardware accepted the write!
//! Flush read from Page 0x0a: OK
//! ```
//!
//! ## Prerequisites
//! - RTL-SDR Blog V4 connected and not claimed by another process
//! - USB permissions (udev rule or run with sudo)

use rusb::{Context, UsbContext};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("--- RTL-SDR Blog V4 Raw Init Sequence Validator ---");

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
    let mut handle =
        handle.ok_or_else(|| anyhow::anyhow!("No RTL2832U device found (0x0bda:0x2838)"))?;

    println!("Resetting USB device...");
    let _ = handle.reset();
    std::thread::sleep(Duration::from_millis(500));

    // Re-open after reset
    let devices = context.devices()?;
    let mut new_handle = None;
    for device in devices.iter() {
        let desc = device.device_descriptor()?;
        if desc.vendor_id() == 0x0bda && desc.product_id() == 0x2838 {
            new_handle = Some(device.open()?);
            break;
        }
    }
    let handle =
        new_handle.ok_or_else(|| anyhow::anyhow!("Device lost after reset — unplug and replug"))?;

    let active_config = handle.active_configuration().unwrap_or(0);
    if active_config != 1 {
        println!("Setting USB configuration 1 (was {})...", active_config);
        handle.set_active_configuration(1)?;
    }

    match handle.kernel_driver_active(0) {
        Ok(true) => {
            println!("Detaching kernel driver...");
            handle.detach_kernel_driver(0)?;
        }
        Ok(false) => println!("Kernel driver already detached."),
        Err(e) => println!("Kernel driver check failed: {:?}", e),
    }
    handle.claim_interface(0)?;
    println!("Interface 0 claimed.\n");

    // Note: V4 does NOT get a GPIO pulse — librtlsdr branches to `found:` before
    // the GPIO code when it detects the V4 EEPROM strings. This is intentional.

    println!("Step 0: USB SIE init (USB_SYSCTL = 0x09)");
    let res = handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100));
    println!("   Result: {:?}", res);

    println!("Step 1: Demod power-on via SYS block");
    handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(50))?;
    handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(50))?;
    std::thread::sleep(Duration::from_millis(100));

    println!("Step 2: Demod soft-reset (page1 reg 0x01 = 0x14)");
    // wValue = (addr << 8) | 0x20 = (0x01 << 8) | 0x20 = 0x0120
    // wIndex = (DEMOD_block=0 << 8) | 0x10 | page1 = 0x0011
    let res = handle.write_control(0x40, 0, 0x0120, 0x0011, &[0x14], Duration::from_millis(100));
    println!("   Write result: {:?}", res);

    if res.is_ok() {
        println!("   [SUCCESS] Hardware accepted the write!");
        // Dummy read flush — required after every demod write
        let mut b = [0u8; 1];
        let _ = handle.read_control(
            0xc0,
            0,
            (0x01u16 << 8) | 0x20,
            0x000a,
            &mut b,
            Duration::from_millis(100),
        );
        println!("   Dummy read flush: OK");
    } else {
        println!(
            "   [FAIL] Hardware rejected the write — check USB permissions and kernel driver status"
        );
    }

    Ok(())
}
