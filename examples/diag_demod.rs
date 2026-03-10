use rusb::{Context, UsbContext};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("--- RTL-SDR 'Reconciliation' ---");
    let context = Context::new()?;
    
    for attempt in 1..=5 {
        println!("\nAttempt {} to re-acquire hardware...", attempt);
        let devices = context.devices()?;
        let mut found = false;
        for device in devices.iter() {
            let desc = device.device_descriptor()?;
            if desc.vendor_id() == 0x0bda && (desc.product_id() == 0x2838 || desc.product_id() == 0x2832) {
                found = true;
                println!("  Device found on Bus {:03} Address {:03}", device.bus_number(), device.address());
                
                match device.open() {
                    Ok(mut handle) => {
                        println!("  SUCCESS! Handle acquired.");
                        println!("  Detaching kernel...");
                        let _ = handle.detach_kernel_driver(0);
                        let _ = handle.claim_interface(0);
                        
                        println!("  Testing SYS read (Block 2, Reg 1)...");
                        let mut b = [0u8; 1];
                        let res = handle.read_control(0xc0, 0, 0x0001, 0x0200, &mut b, Duration::from_millis(200));
                        println!("    Result: {:?}", res);
                        
                        println!("  Releasing and closing.");
                        let _ = handle.release_interface(0);
                        return Ok(());
                    }
                    Err(e) => {
                        println!("  Open FAILED: {:?}", e);
                    }
                }
            }
        }
        if !found { println!("  Device not seen by OS!"); }
        std::thread::sleep(Duration::from_secs(1));
    }
    
    Ok(())
}
