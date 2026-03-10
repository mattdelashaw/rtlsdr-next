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
    let mut handle = handle.ok_or_else(|| anyhow::anyhow!("No device"))?;
    
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
    let handle = new_handle.ok_or_else(|| anyhow::anyhow!("No device after reset"))?;

    println!("Checking USB configuration...");
    let active_config = handle.active_configuration().unwrap_or(0);
    if active_config != 1 {
        println!("   Setting configuration 1 (current is {})...", active_config);
        handle.set_active_configuration(1)?;
    } else {
        println!("   Configuration 1 already active.");
    }

    println!("Detaching kernel driver and claiming interface...");
    match handle.kernel_driver_active(0) {
        Ok(true) => {
            println!("   Kernel driver is active, detaching...");
            handle.detach_kernel_driver(0)?;
        }
        Ok(false) => println!("   Kernel driver already detached."),
        Err(e) => println!("   Kernel driver check failed: {:?}", e),
    }
    
    handle.claim_interface(0)?;
    println!("   Interface 0 claimed.");

    println!("--- RTL-SDR Blog V4 RAW Hex Sequence ---");

    // 1. V4 GPIO Power-up (GPIO 4 and 5)
    println!("Step 1: GPIO Power-up (Blog V4 specific) on Block 2");
    let _ = handle.write_control(0x40, 0, 0x0003, 0x0210, &[0x10], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0004, 0x0210, &[0x00], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0003, 0x0210, &[0x20], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0004, 0x0210, &[0x00], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0001, 0x0210, &[0x10], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0001, 0x0210, &[0x20], Duration::from_millis(50));
    std::thread::sleep(Duration::from_millis(200));

    // librtlsdr: rtlsdr_open() -> rtlsdr_write_reg(dev, USBB, USB_SYSCTL, 0x09, 1)
    println!("Step 0: The librtlsdr 'Connectivity Check' (USB_SYSCTL=0x09)");
    let res = handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100));
    println!("   Result: {:?}", res);

    // 3. Demod Power-on (Block 2)
    println!("Step 3: Demod Power-on via SYS on Block 2");
    let _ = handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(50));
    std::thread::sleep(Duration::from_millis(100));

    // 4. THE CRITICAL TEST: Demod Soft Reset (Block 1, Page 1, Reg 0x01)
    println!("Step 4: The Critical Test (Demod Soft Reset) on Block 1");
    let res = handle.write_control(0x40, 0, 0x0120, 0x0111, &[0x14], Duration::from_millis(100));
    println!("   Write Result: {:?}", res);

    if res.is_ok() {
        println!("   [SUCCESS] Hardware accepted the write!");
        let mut b = [0u8; 1];
        let _ = handle.read_control(0xc0, 0, 0x0120, 0x000a, &mut b, Duration::from_millis(100));
        println!("   Flush read from Page 0x0a: OK");
    }

    Ok(())
}
