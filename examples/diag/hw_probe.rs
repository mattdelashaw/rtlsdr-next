//! # hw_probe
//!
//! **End-to-end driver smoke test.** Initializes the full `Driver` stack,
//! tunes to FM and HF frequencies, then streams 1 second of samples and
//! reports throughput.
//!
//! ## Purpose
//! The first thing to run after a fresh install or hardware change. Exercises
//! the complete init path (baseband, tuner probe, PLL, streaming) and gives a
//! clear PASS/FAIL with actionable error messages.
//!
//! ## Expected output (healthy V4)
//! ```
//! SUCCESS! Driver initialized.
//!   Manufacturer: RTLSDRBlog
//!   Product:      Blog V4
//!   Is V4:        true
//! Frequency set to 100 MHz... Success! Actual: 100000000 Hz
//! Frequency set to 7 MHz (HF)... Success! Actual: 7000000 Hz
//! Read 2097152 bytes in 1003 ms
//! ```
//!
//! ## Prerequisites
//! - RTL-SDR dongle connected and not claimed by another process
//! - USB permissions (udev rule or run with sudo)

use env_logger::Env;
use rtlsdr_next::Driver;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(Env::default().default_filter_or("debug")).init();

    println!("--- RTL-SDR Hardware Probe ---");

    match Driver::new() {
        Ok(mut driver) => {
            println!("\nSUCCESS! Driver initialized.");
            println!("  Manufacturer: {}", driver.info.manufacturer);
            println!("  Product:      {}", driver.info.product);
            println!("  Is V4:        {}", driver.info.is_v4);

            println!("\nSetting frequency to 100 MHz (FM band)...");
            match driver.set_frequency(100_000_000) {
                Ok(actual) => println!("  Success! Actual: {} Hz", actual),
                Err(e) => println!("  FAILED: {:?}", e),
            }

            println!("\nSetting frequency to 7 MHz (HF — V4 upconverter path)...");
            match driver.set_frequency(7_000_000) {
                Ok(actual) => println!("  Success! Actual: {} Hz", actual),
                Err(e) => println!("  FAILED: {:?}", e),
            }

            println!("\nStreaming for 1 second...");
            let mut stream = driver.stream();
            let start = std::time::Instant::now();
            let mut total_bytes = 0usize;

            while start.elapsed().as_secs() < 1 {
                match stream.next().await {
                    Some(Ok(samples)) => total_bytes += samples.len(),
                    Some(Err(e)) => {
                        println!("  Stream error: {:?}", e);
                        break;
                    }
                    None => break,
                }
            }

            println!(
                "  Read {} bytes in {} ms",
                total_bytes,
                start.elapsed().as_millis()
            );
            if total_bytes > 1_000_000 {
                println!("  PASS — data rate looks healthy");
            } else {
                println!("  WARN — low byte count, check gain and antenna");
            }
        }
        Err(e) => {
            println!("\nFAILED to initialize driver: {:?}", e);
            println!("\nPossible causes:");
            println!("  1. Permission denied — check udev rules or run with sudo");
            println!("  2. Device busy — kill rtl_tcp, OpenWebRX, or other SDR processes");
            println!("  3. Hardware not connected or EEPROM corrupted");
            println!("\nEEPROM recovery (V4):");
            println!(
                "  ~/rtl-sdr-blog/build/src/rtl_eeprom -m RTLSDRBlog -p \"Blog V4\" -s 00000001"
            );
        }
    }

    Ok(())
}
