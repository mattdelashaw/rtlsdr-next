use std::sync::Arc;

pub mod device;
pub mod demod;

use device::HardwareInterface;
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
pub use rtl_tcp::TcpServer;
pub use demod::DEFAULT_SAMPLE_RATE;

pub struct Driver {
    device:     Arc<Device<rusb::Context>>,
    pub info:   DeviceInfo,
    pub tuner:  Box<dyn Tuner>,
    pub sample_rate: u32,
    pub frequency:   u64,
    pub ppm:         i32,
}

use tuner::TunerType;

impl Driver {
    pub fn new() -> Result<Self> {
        let device = Arc::new(Device::open()?);
        let hw     = device.as_ref();

        // ── 1. RTL2832U baseband init ──
        demod::power_on(hw)?;
        std::thread::sleep(std::time::Duration::from_millis(100));

        // ── 2. Read strings ──
        let info = device.read_info();
        let is_v4 = info.is_v4;
        log::info!("Found RTL2832U — manufacturer: {:?} product: {:?} is_v4: {}", info.manufacturer, info.product, info.is_v4);

        if is_v4 {
            log::info!("Applying RTL-SDR Blog V4 GPIO power-up sequence...");
            hw.set_gpio_output(4)?;
            hw.set_gpio_output(5)?;
            hw.set_gpio_bit(4, true)?;
            hw.set_gpio_bit(5, true)?;
            std::thread::sleep(std::time::Duration::from_millis(250));
        }

        // ── 3. Probe and initialize tuner ──
        let tuner_type = hw.probe_tuner()?;
        log::info!("Detected Tuner: {:?}", tuner_type);

        let tuner: Box<dyn Tuner> = match tuner_type {
            TunerType::R820T | TunerType::R828D => Box::new(tuners::r828d::R828D::new(device.clone(), is_v4)),
            TunerType::Unknown(_) if is_v4 => Box::new(tuners::r828d::R828D::new(device.clone(), true)),
            _ => return Err(Error::UnsupportedTuner(format!("{:?} not yet supported", tuner_type))),
        };

        tuner.initialize()?;

        // ── 4. Post-detection demod config for Low-IF ──
        if matches!(tuner_type, TunerType::R820T | TunerType::R828D) || is_v4 {
            demod::set_tuner_low_if(hw)?;
            demod::write_reg_direct(hw, registers::demod::P1_PAGE, 0x15, 0x01)?;
        }

        // ── 5. Demodulator sync ──
        let initial_if = 2_300_000u32;
        tuner.set_if_freq(initial_if as u64)?;
        demod::set_if_freq_xtal(hw, initial_if, 28_800_000)?;
        demod::set_sample_rate_xtal(hw, DEFAULT_SAMPLE_RATE, 28_800_000)?;
        demod::reset_demod(hw)?;
        demod::start_streaming(hw)?;

        log::info!("RTL-SDR driver ready");

        Ok(Self { device, info, tuner, sample_rate: DEFAULT_SAMPLE_RATE, frequency: 0, ppm: 0 })
    }

    pub fn set_frequency(&mut self, hz: u64) -> Result<u64> {
        let actual = self.tuner.set_frequency(hz)?;
        let hw     = self.device.as_ref();
        let xtal   = self.corrected_xtal_hz();
        let if_hz  = self.tuner.get_if_freq();
        demod::set_if_freq_xtal(hw, if_hz as u32, xtal)?;
        demod::reset_demod(hw)?;
        self.frequency = hz;
        log::trace!("Setting frequency: {:?}", hz);
        Ok(actual)
    }

    pub fn set_sample_rate(&mut self, rate_hz: u32) -> Result<()> {
        let hw   = self.device.as_ref();
        let xtal = self.corrected_xtal_hz();
        let if_hz = if rate_hz < 2_500_000 { 2_300_000 } else { 3_570_000 };
        self.tuner.set_if_freq(if_hz)?;
        demod::set_sample_rate_xtal(hw, rate_hz, xtal)?;
        demod::set_if_freq_xtal(hw, if_hz as u32, xtal)?;
        demod::reset_demod(hw)?;
        self.sample_rate = rate_hz;
        Ok(())
    }

    pub fn set_ppm(&mut self, ppm: i32) -> Result<()> {
        self.ppm = ppm;
        self.tuner.set_ppm(ppm)?;
        let freq = self.frequency;
        let rate = self.sample_rate;
        if freq > 0 { let _ = self.set_frequency(freq); }
        let _ = self.set_sample_rate(rate);
        Ok(())
    }

    pub fn set_bias_t(&self, on: bool) -> Result<()> {
        let hw = self.device.as_ref();
        hw.set_gpio_output(0)?;
        hw.set_gpio_bit(0, on)?;
        log::info!("Bias-T turned {}", if on { "ON" } else { "OFF" });
        Ok(())
    }

    fn corrected_xtal_hz(&self) -> u32 {
        let nominal = 28_800_000i64;
        let offset  = (nominal * self.ppm as i64) / 1_000_000;
        (nominal + offset) as u32
    }

    pub fn stream(&self) -> SampleStream<rusb::Context> { SampleStream::new(self.device.clone()) }
    pub fn stream_f32(&self, factor: usize) -> F32Stream<rusb::Context> { F32Stream::new(self.stream(), factor) }

    pub async fn start_sharing<P: AsRef<std::path::Path>>(&self, path: P) -> Result<SharingServer> {
        let mut stream = self.stream();
        let (tx, rx)   = tokio::sync::broadcast::channel::<Arc<Vec<u8>>>(16);
        tokio::spawn(async move {
            while let Some(res) = stream.next().await {
                match res {
                    Ok(samples) => { let _ = tx.send(Arc::new(samples.to_vec())); }
                    Err(e) => { log::error!("Stream error: {:?}", e); break; }
                }
            }
        });
        SharingServer::start(path, rx).await.map_err(|e| Error::Tuner(format!("Server error: {:?}", e)))
    }

    pub async fn start_rtl_tcp(self, addr: &str) -> Result<TcpServer> {
        TcpServer::start(self, addr).await.map_err(|e| Error::Tuner(format!("TCP Server error: {:?}", e)))
    }
}
