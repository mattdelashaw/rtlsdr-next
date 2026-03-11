use rtlsdr_next::Driver;
use env_logger::Env;

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(Env::default().default_filter_or("debug")).init();

    println!("--- RTL-SDR Hardware Probe ---");
    
    match Driver::new() {
        Ok(mut driver) => {
            println!("\nSUCCESS! Driver initialized.");
            println!("  Manufacturer: {}", driver.info.manufacturer);
            println!("  Product:      {}", driver.info.product);
            println!("  Is V4:        {}", driver.info.is_v4);
            
            println!("\nAttempting to set frequency to 100 MHz (FM)...");
            match driver.set_frequency(100_000_000) {
                Ok(actual) => println!("  Success! Actual frequency: {} Hz", actual),
                Err(e) => println!("  Frequency set FAILED: {:?}", e),
            }

            println!("\nAttempting to set frequency to 7 MHz (HF - V4 check)...");
            match driver.set_frequency(7_000_000) {
                Ok(actual) => println!("  Success! Actual frequency: {} Hz", actual),
                Err(e) => println!("  HF Frequency set FAILED: {:?}", e),
            }

            println!("\nAttempting to read 1 second of samples...");
            let mut stream = driver.stream();
            let start = std::time::Instant::now();
            let mut total_bytes = 0;
            
            // In a real app we'd use tokio, but for a quick probe we can use block_on or just wait
            let runtime = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
            runtime.block_on(async {
                while start.elapsed().as_secs() < 1 {
                    if let Some(res) = stream.next().await {
                        let samples = res.unwrap();
                        total_bytes += samples.len();
                    }
                }
            });

            println!("  Success! Read {} bytes in {} ms", total_bytes, start.elapsed().as_millis());
        }
        Err(e) => {
            println!("\nFAILED to initialize driver: {:?}", e);
            println!("\nPossible causes:");
            println!("  1. Permission denied (check udev rules or run with sudo)");
            println!("  2. Device in use by another process (check rtl_sdr, rtl_tcp, etc.)");
            println!("  3. Hardware not connected or faulty.");
        }
    }

    Ok(())
}
