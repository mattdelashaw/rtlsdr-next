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
    let handle = handle.ok_or_else(|| anyhow::anyhow!("No device"))?;
    let _ = handle.detach_kernel_driver(0);
    let _ = handle.claim_interface(0);

    // 1. Power up
    let _ = handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(100));
    let _ = handle.write_control(0x40, 0, 0x000b, 0x0210, &[0x22], Duration::from_millis(100));
    let _ = handle.write_control(0x40, 0, 0x0000, 0x0210, &[0xe8], Duration::from_millis(100));
    std::thread::sleep(Duration::from_millis(100));

    // 2. Scan all blocks 0..7 for a response to Reg 0x01
    println!("Scanning all blocks 0..7 for Reg 0x01 response...");
    for blk in 0..8u16 {
        let mut b = [0u8; 1];
        let res = handle.read_control(0xc0, 0, 0x0001, blk << 8, &mut b, Duration::from_millis(50));
        println!("  Block {} (wIndex=0x{:04x}) -> {:?}", blk, blk << 8, res);
    }

    Ok(())
}
