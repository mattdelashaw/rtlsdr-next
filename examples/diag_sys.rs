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

    for blk in [1, 2, 4] {
        println!("\n--- RTL2832U Block {} Dump ---", blk);
        for reg in 0..0x10u16 {
            let mut b = [0u8; 1];
            
            // Pattern B: wValue = reg
            let res_b = handle.read_control(0xc0, 0, reg, blk << 8, &mut b, Duration::from_millis(50));
            let val_b = if res_b.is_ok() { format!("0x{:02x}", b[0]) } else { format!("{:?}", res_b.err().unwrap()) };

            // Pattern C: wValue = (reg << 8) | 0x20
            let res_c = handle.read_control(0xc0, 0, (reg << 8) | 0x20, blk << 8, &mut b, Duration::from_millis(50));
            let val_c = if res_c.is_ok() { format!("0x{:02x}", b[0]) } else { format!("{:?}", res_c.err().unwrap()) };

            println!("  Reg 0x{:02x}:  B={}  C={}", reg, val_b, val_c);
        }
    }

    println!("\n--- Full librtlsdr + V4 Sequence ---");
    // librtlsdr: Step 0
    println!("   Step 0: USB_SYSCTL=0x09...");
    let _ = handle.write_control(0x40, 0, 0x0000, 0x0110, &[0x09], Duration::from_millis(50));

    // V4 Power-up
    println!("   V4 Power-up...");
    let _ = handle.write_control(0x40, 0, 0x0003, 0x0210, &[0x10], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0004, 0x0210, &[0x00], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0003, 0x0210, &[0x20], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0004, 0x0210, &[0x00], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0001, 0x0210, &[0x10], Duration::from_millis(50));
    let _ = handle.write_control(0x40, 0, 0x0001, 0x0210, &[0x20], Duration::from_millis(50));
    std::thread::sleep(Duration::from_millis(250));

    // Repeater Enable on Block 4
    println!("   Enabling Repeater on Block 4...");
    let mut b = [0u8; 1];
    let _ = handle.write_control(0x40, 0, 0x0120, 0x0411, &[0x18], Duration::from_millis(50));
    let _ = handle.read_control(0xc0, 0, 0x0120, 0x040a, &mut b, Duration::from_millis(50));
    std::thread::sleep(Duration::from_millis(100));

    // Tuner Read
    println!("--- Real I2C Block 6 Multi-Read (Post Block 4 Repeater) ---");
    for reg in 0..5u16 {
        let wvalue = (0x34 << 8) | reg;
        let mut b = [0u8; 1];
        let _ = handle.read_control(0xc0, 0, wvalue, 0x0600, &mut b, Duration::from_millis(50));
        println!("      Reg 0x{:02x} -> Val: 0x{:02x}", reg, b[0]);
    }

    Ok(())
}
