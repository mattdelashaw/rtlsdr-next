use rusb::{Context, UsbContext};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("--- RTL-SDR 'Mimicry' Resurrection ---");
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
    let handle = handle.ok_or_else(|| anyhow::anyhow!("No device"))?;
    let _ = handle.detach_kernel_driver(0);
    let _ = handle.claim_interface(0);

    // librtlsdr: rtlsdr_open() -> rtlsdr_set_i2c_repeater(0)
    // This is the very first thing it does. It writes to Demod.
    
    println!("1. Initializing USB SIE (Block 1)...");
    let _ = handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100));
    std::thread::sleep(Duration::from_millis(100));

    println!("2. Enabling Demod Power via SYS (Block 2)...");
    let _ = handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(100));
    let _ = handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(100));
    std::thread::sleep(Duration::from_millis(100));

    println!("3. Probing ALL Demod Patterns (READ)...");
    let mut working_read = None;
    for reg in 0..256u16 {
        let mut b = [0u8; 1];
        // Read Pattern A: wValue = (reg << 8) | 0x20, wIndex = page
        if handle.read_control(0xc0, 0, (reg << 8) | 0x20, 1, &mut b, Duration::from_millis(10)).is_ok() {
            println!("   [SUCCESS] Read Pattern A (wValue=(reg<<8)|0x20) works! Reg 0x{:02x}", reg);
            working_read = Some('A'); break;
        }
        // Read Pattern B: wValue = reg, wIndex = page
        if handle.read_control(0xc0, 0, reg, 1, &mut b, Duration::from_millis(10)).is_ok() {
            println!("   [SUCCESS] Read Pattern B (wValue=reg) works! Reg 0x{:02x}", reg);
            working_read = Some('B'); break;
        }
    }

    println!("4. Probing Demod WRITE Pattern C + Dummy Read...");
    let reg = 0x19;
    let val = 0x05; 
    let windex = 0x10 | 0;
    let wvalue = (reg << 8) | 0x20;
    
    println!("   Write...");
    let _ = handle.write_control(0x40, 0, wvalue, windex, &[val], Duration::from_millis(100));
    
    println!("   Immediate Read back from Block 0...");
    let mut b = [0u8; 1];
    let r_res = handle.read_control(0xc0, 0, wvalue, 0, &mut b, Duration::from_millis(100));
    println!("      Read Result: {:?}", r_res);

    println!("   Immediate Read from I2C Block (Block 6)...");
    let r_res = handle.read_control(0xc0, 0, 0, 0x0600, &mut b, Duration::from_millis(100));
    println!("      I2C Block Read Result: {:?}", r_res);

    Ok(())
}
