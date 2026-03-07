use std::sync::Arc;

pub mod device;
pub mod demod;
pub mod error;
pub mod registers;
pub mod tuner;
pub mod tuners;
pub mod stream;
pub mod converter;
pub mod dsp;
pub mod server;

pub use device::{Device, DeviceInfo};
pub use error::{Error, Result};
pub use tuner::{Tuner, FilterRange};
pub use stream::{SampleStream, F32Stream};
pub use server::SharingServer;
pub use demod::DEFAULT_SAMPLE_RATE;

/// The main entry point for the next-generation RTL-SDR driver.
///
/// # Example
///
/// ```no_run
/// use rtlsdr_next::Driver;
///
/// #[tokio::main]
/// async fn main() -> anyhow::Result<()> {
///     let mut driver = Driver::new()?;
///
///     println!("Device: {} {}", driver.info.manufacturer, driver.info.product);
///     println!("V4: {}", driver.info.is_v4);
///
///     let mut stream = driver.stream();
///     while let Some(res) = stream.next().await {
///         let samples = res?; // Handle hardware errors (e.g. disconnect)
///         // Process samples...
///     }
///     Ok(())
/// }
/// ```
pub struct Driver {
    device: Arc<Device<rusb::Context>>,
    /// Probed EEPROM metadata including is_v4 flag.
    pub info: DeviceInfo,
    /// Access the underlying tuner to change frequency or gain.
    pub tuner: Box<dyn Tuner>,
    /// Current output sample rate in Hz.
    pub sample_rate: u32,
    /// Current center frequency in Hz.
    pub frequency: u64,
    /// Frequency correction in Parts Per Million.
    pub ppm: i32,
}

impl Driver {
    /// Discovers and initializes the first available RTL-SDR device.
    ///
    /// Full initialization sequence:
    /// 1. Open USB device, probe EEPROM strings, detect V4
    /// 2. Power on RTL2832U ADCs and demodulator PLL
    /// 3. Write demodulator static init table
    /// 4. Initialize tuner (R828D shadow register upload)
    /// 5. Program IF frequency registers (3.57 MHz for R828D)
    /// 6. Program resampler for default sample rate (2.048 MSPS)
    /// 7. Soft-reset demodulator
    /// 8. Flush EPA FIFO and enable bulk IN
    pub fn new() -> Result<Self> {
        let device = Arc::new(Device::open()?);
        let hw = device.as_ref();

        let is_v4 = device.info.is_v4;
        let info  = device.info.clone();

        // ── Steps 2–3: RTL2832U baseband init ────────────────────────────
        demod::power_on(hw)?;
        demod::init_registers(hw)?;

        // ── Step 4: tuner init ────────────────────────────────────────────
        let tuner: Box<dyn Tuner> =
            Box::new(tuners::r828d::R828D::new(device.clone(), is_v4));
        tuner.initialize()?;

        // ── Steps 5–8: demodulator sync ───────────────────────────────────
        // Use nominal 28.8MHz for initial sync
        demod::set_if_freq(hw, registers::IF_FREQ_HZ as u32, 28_800_000)?;
        demod::set_sample_rate(hw, DEFAULT_SAMPLE_RATE, 28_800_000)?;
        demod::reset_demod(hw)?;
        demod::start_streaming(hw)?;

        Ok(Self {
            device,
            info,
            tuner,
            sample_rate: DEFAULT_SAMPLE_RATE,
            frequency:   0,
            ppm:         0,
        })
    }

    /// Set the center frequency. Re-syncs the demodulator after PLL relock.
    pub fn set_frequency(&mut self, hz: u64) -> Result<u64> {
        let actual = self.tuner.set_frequency(hz)?;
        let hw = self.device.as_ref();
        let xtal = self.corrected_xtal_hz();

        // After PLL relock, re-program IF and reset demod to flush state
        demod::set_if_freq(hw, registers::IF_FREQ_HZ as u32, xtal)?;
        demod::reset_demod(hw)?;
        self.frequency = hz;
        Ok(actual)
    }

    /// Set the output sample rate in Hz. Valid range: ~225 kSPS to 3.2 MSPS.
    pub fn set_sample_rate(&mut self, rate_hz: u32) -> Result<()> {
        let hw = self.device.as_ref();
        let xtal = self.corrected_xtal_hz();

        demod::set_sample_rate(hw, rate_hz, xtal)?;
        demod::reset_demod(hw)?;
        self.sample_rate = rate_hz;
        Ok(())
    }

    /// Set the crystal frequency correction in Parts Per Million (PPM) and re-sync hardware.
    pub fn set_ppm(&mut self, ppm: i32) -> Result<()> {
        self.ppm = ppm;
        self.tuner.set_ppm(ppm)?;
        
        // Re-apply current frequency and sample rate with the new correction
        let freq = self.frequency;
        let rate = self.sample_rate;
        
        if freq > 0 {
            self.set_frequency(freq)?;
        }
        self.set_sample_rate(rate)?;
        
        Ok(())
    }

    fn corrected_xtal_hz(&self) -> u32 {
        let nominal = 28_800_000i64;
        let offset = (nominal * self.ppm as i64) / 1_000_000;
        (nominal + offset) as u32
    }

    /// Create a new asynchronous sample stream.
    pub fn stream(&self) -> SampleStream {
        SampleStream::new(self.device.clone())
    }

    /// Create a high-level DSP stream that produces decimated F32 samples.
    ///
    /// The stream automatically handles U8 -> F32 conversion and applies a
    /// windowed-sinc low-pass filter before decimation.
    ///
    /// # Arguments
    /// * `factor` - Decimation factor (e.g. 10 to turn 2.048 MSPS -> 204.8 kSPS).
    pub fn stream_f32(&self, factor: usize) -> F32Stream {
        F32Stream::new(self.stream(), factor)
    }

    /// Start a sharing server allowing multiple local apps to receive samples
    /// via a Unix domain socket.
    pub async fn start_sharing<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> Result<SharingServer> {
        let mut stream = self.stream();
        let (tx, rx)   = tokio::sync::broadcast::channel::<Arc<Vec<u8>>>(16);

        tokio::spawn(async move {
            while let Some(res) = stream.next().await {
                match res {
                    Ok(samples) => {
                        let _ = tx.send(Arc::new(samples));
                    }
                    Err(e) => {
                        log::error!("Hardware stream error during sharing: {:?}", e);
                        break;
                    }
                }
            }
        });

        SharingServer::start(path, rx)
            .await
            .map_err(|e| Error::Tuner(format!("Server error: {:?}", e)))
    }
}
