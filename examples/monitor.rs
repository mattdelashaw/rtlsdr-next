use log::{error, info};
use rtlsdr_next::Driver;
use std::time::Instant;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Initialize logging so we can see the driver's internal state
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    info!("Starting rtlsdr-next monitor example...");

    // 1. Discover and open the first available RTL-SDR device
    let mut driver = match Driver::with_index(0) {
        Ok(d) => d,
        Err(e) => {
            eprintln!(
                "Error opening device: {:?}\n(Check that your RTL-SDR is plugged in and you have USB permissions)",
                e
            );
            return Ok(());
        }
    };

    info!(
        "Device Info: {} {}",
        driver.info.manufacturer, driver.info.product
    );
    info!(
        "Hardware Version: {}",
        if driver.info.is_v4 {
            "V4 (R828D)"
        } else {
            "V3/Generic"
        }
    );

    // 2. Configure the hardware
    // Apply a 50 PPM correction (typical for some crystals)
    driver.set_ppm(0).await?;

    // Set frequency to 100 MHz (VHF FM Band)
    let actual_freq = driver.set_frequency(100_000_000, None).await?;
    info!(
        "Center Frequency set to: {:.3} MHz",
        actual_freq as f64 / 1e6
    );

    // Set gain to ~30 dB
    let actual_gain = driver.tuner.set_gain(29.7)?;
    info!("Manual Gain set to: {:.1} dB", actual_gain);

    // 3. Create a decimated F32 stream
    // Decimate by 8: 2.048 MSPS -> 256 kSPS
    let factor = 8;
    let mut stream = driver.stream_f32(factor);
    let output_rate = 2_048_000 / factor;
    info!(
        "Stream initialized: {} kSPS (Decimation Factor: {})",
        output_rate / 1000,
        factor
    );

    info!("Monitoring signals (Ctrl+C to stop)...");

    let start_time = Instant::now();
    let mut total_samples = 0usize;
    let mut block_count = 0;

    // 4. Main processing loop
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("\nShutdown requested...");
                break;
            }
            res = stream.next() => {
                let iq_data = match res {
                    Some(Ok(data)) => data,
                    Some(Err(e)) => {
                        error!("\nHardware stream error: {:?}", e);
                        break;
                    }
                    None => break,
                };

                total_samples += iq_data.len() / 2; // I and Q are interleaved
                block_count += 1;

                // Calculate basic statistics every 10 blocks to show the driver is working
                if block_count % 10 == 0 {
                    let mut mag_sum = 0.0f32;
                    for i in (0..iq_data.len()).step_by(2) {
                        let i_val = iq_data[i];
                        let q_val = iq_data[i+1];
                        mag_sum += (i_val * i_val + q_val * q_val).sqrt();
                    }
                    let avg_mag = mag_sum / (iq_data.len() / 2) as f32;

                    let elapsed = start_time.elapsed().as_secs_f64();
                    let throughput = (total_samples as f64 / elapsed) / 1000.0;

                    print!(
                        "\rBlocks: {:<5} | Avg Mag: {:.4} | Throughput: {:.2} kSPS",
                        block_count, avg_mag, throughput
                    );
                    use std::io::Write;
                    std::io::stdout().flush()?;
                }
            }
        }
    }

    info!("Cleaning up...");
    Ok(())
}
