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
pub mod rtl_tcp;
pub mod websdr;

pub use device::{Device, DeviceInfo};
pub use error::{Error, Result};
pub use tuner::{Tuner, FilterRange};
pub use stream::{SampleStream, F32Stream, PooledBuffer};
pub use server::SharingServer;

/// Concrete stream types — use these when you need to name the stream type
/// in a struct or function signature. The generic `SampleStream<T>` and
/// `F32Stream<T>` are usable too, but require importing `rusb::Context`.
pub type RawSampleStream = stream::SampleStream<rusb::Context>;
pub type RawF32Stream    = stream::F32Stream<rusb::Context>;
pub use rtl_tcp::TcpServer;
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

use tuner::TunerType;

impl Driver {
    /// Discovers and initializes the first available RTL-SDR device.
    pub fn new() -> Result<Self> {
        let device = Arc::new(Device::open()?);
        let hw = device.as_ref();

        let is_v4 = device.info.is_v4;
        let info  = device.info.clone();

        // ── Steps 2–3: RTL2832U baseband init ────────────────────────────
        demod::power_on(hw)?;
        demod::init_registers(hw)?;

        // ── Step 4: tuner init ────────────────────────────────────────────
        // Probe the tuner type
        let tuner_type = device.probe_tuner()?;
        log::info!("Detected Tuner: {:?}", tuner_type);

        let tuner: Box<dyn Tuner> = match tuner_type {
            TunerType::R820T => {
                Box::new(tuners::r828d::R828D::new(device.clone(), is_v4))
            }
            TunerType::Unknown(_) if is_v4 => {
                // EEPROM confirmed V4 but I2C probe failed — power sequencing
                // issue or cold start. Trust the EEPROM and proceed with R828D.
                log::warn!("Tuner I2C probe returned Unknown but EEPROM says V4 — forcing R828D");
                Box::new(tuners::r828d::R828D::new(device.clone(), true))
            }
            _ => {
                return Err(Error::UnsupportedTuner(format!("{:?} not yet supported", tuner_type)));
            }
        };

        tuner.initialize()?;

        // ── Steps 5–8: demodulator sync ───────────────────────────────────
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
        
        let freq = self.frequency;
        let rate = self.sample_rate;
        
        if freq > 0 {
            let _ = self.set_frequency(freq);
        }
        let _ = self.set_sample_rate(rate);
        
        Ok(())
    }

    fn corrected_xtal_hz(&self) -> u32 {
        let nominal = 28_800_000i64;
        let offset = (nominal * self.ppm as i64) / 1_000_000;
        (nominal + offset) as u32
    }

    /// Create a new asynchronous sample stream.
    pub fn stream(&self) -> SampleStream<rusb::Context> {
        SampleStream::new(self.device.clone())
    }

    /// Create a high-level DSP stream that produces decimated F32 samples.
    pub fn stream_f32(&self, factor: usize) -> F32Stream<rusb::Context> {
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
                        // For broadcasting, we currently clone once per block into an Arc.
                        // This allows multiple clients to share the same Arc without further clones.
                        let _ = tx.send(Arc::new(samples.to_vec()));
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

    /// Start an rtl_tcp compatible server, consuming the Driver.
    ///
    /// Takes ownership because the TCP server needs exclusive access to the
    /// hardware stream — there is only one dongle. Call this instead of
    /// keeping the Driver around.
    pub async fn start_rtl_tcp(self, addr: &str) -> Result<TcpServer> {
        TcpServer::start(self, addr)
            .await
            .map_err(|e| Error::Tuner(format!("TCP Server error: {:?}", e)))
    }
}