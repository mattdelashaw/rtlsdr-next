use log::{error, info};
use rtlsdr_next::Driver;
use rtlsdr_next::dsp::FmDemodulator;
use std::time::Instant;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    info!("Starting rtlsdr-next FM radio example...");

    // 1. Open device
    let mut driver = Driver::with_index(0)?;
    info!(
        "Device: {} {}",
        driver.info.manufacturer, driver.info.product
    );

    // 2. Configure for FM Broadcast
    // Typical FM station in many regions: 100.0 MHz
    let freq = 100_000_000;
    driver.set_frequency(freq, None)?;
    driver.tuner.set_gain(32.8)?;

    // We want to decimate down to something manageable for audio.
    // RTL-SDR at 2.048 MSPS / 8 = 256 kSPS.
    // This is wide enough for a 200kHz FM broadcast channel.
    let factor = 8;
    let mut stream = driver
        .stream_f32(factor)
        .with_dc_removal(0.01)
        .with_agc(1.0, 0.01, 0.01);

    let mut fm = FmDemodulator::new();

    info!("Demodulating at {:.1} MHz...", freq as f64 / 1e6);
    info!("Press Ctrl+C to stop.");

    let mut block_count = 0;
    let start_time = Instant::now();

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
                        error!("Stream error: {:?}", e);
                        break;
                    }
                    None => break,
                };

                // Software FM Demodulation
                let audio = fm.process(&iq_data);

                block_count += 1;
                if block_count % 20 == 0 {
                    let elapsed = start_time.elapsed().as_secs_f64();
                    let audio_mag: f32 = audio.iter().map(|v| v.abs()).sum::<f32>() / audio.len() as f32;

                    print!(
                        "\rBlocks: {:<5} | Audio Avg Mag: {:.4} | Elapsed: {:.1}s",
                        block_count, audio_mag, elapsed
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
