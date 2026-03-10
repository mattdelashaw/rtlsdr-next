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
        let is_v4  = device.info.is_v4;
        let info   = device.info.clone();

        // ── 1. RTL2832U baseband init ─────────────────────────────────────
        demod::power_on(hw)?;

        // ── 2. GPIO reset for non-V4 sticks ─────────────────────────────
        // librtlsdr: set_gpio_output(4), bit(4,1), bit(4,0) — reset pulse.
        // V4 is detected by EEPROM and jumps to `found` BEFORE this GPIO code,
        // so V4 does NOT get this pulse. We mirror that exactly.
        if !is_v4 {
            hw.set_gpio_output(4)?;
            hw.set_gpio_bit(4, true)?;
            hw.set_gpio_bit(4, false)?;
        }

        // ── 3. Probe and initialize tuner ────────────────────────────────
        let tuner_type = hw.probe_tuner()?;
        log::info!("Detected Tuner: {:?}", tuner_type);

        let tuner: Box<dyn Tuner> = match tuner_type {
            TunerType::R820T | TunerType::R828D => {
                Box::new(tuners::r828d::R828D::new(device.clone(), is_v4))
            }
            TunerType::Unknown(_) if is_v4 => {
                log::warn!("Tuner I2C probe returned Unknown but EEPROM says V4 — forcing R828D");
                Box::new(tuners::r828d::R828D::new(device.clone(), true))
            }
            _ => {
                return Err(Error::UnsupportedTuner(format!("{:?} not yet supported", tuner_type)));
            }
        };

        tuner.initialize()?;

        // ── 4. Post-detection demod config (matches librtlsdr found: block) ──
        // Disable Zero-IF mode (0x1b -> 0x1a: clear bit 0)
        demod::write_reg_direct(hw, registers::demod::P1_PAGE, 0xb1, 0x1a)?;
        // Only enable In-phase ADC input
        demod::write_reg_direct(hw, registers::demod::P0_PAGE, 0x08, 0x4d)?;

        // ── 5. Demodulator sync ───────────────────────────────────────────
        demod::set_if_freq_xtal(hw, registers::IF_FREQ_HZ as u32, 28_800_000)?;
        demod::set_sample_rate_xtal(hw, DEFAULT_SAMPLE_RATE, 28_800_000)?;
        demod::reset_demod(hw)?;
        demod::start_streaming(hw)?;

        log::info!("RTL-SDR driver ready");

        Ok(Self {
            device,
            info,
            tuner,
            sample_rate: DEFAULT_SAMPLE_RATE,
            frequency:   0,
            ppm:         0,
        })
    }

    pub fn set_frequency(&mut self, hz: u64) -> Result<u64> {
        let actual = self.tuner.set_frequency(hz)?;
        let hw     = self.device.as_ref();
        let xtal   = self.corrected_xtal_hz();
        demod::set_if_freq_xtal(hw, registers::IF_FREQ_HZ as u32, xtal)?;
        demod::reset_demod(hw)?;
        self.frequency = hz;
        log::trace!("Setting frequency: {:?}", hz);
        Ok(actual)
    }

    pub fn set_sample_rate(&mut self, rate_hz: u32) -> Result<()> {
        let hw   = self.device.as_ref();
        let xtal = self.corrected_xtal_hz();
        demod::set_sample_rate_xtal(hw, rate_hz, xtal)?;
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

    fn corrected_xtal_hz(&self) -> u32 {
        let nominal = 28_800_000i64;
        let offset  = (nominal * self.ppm as i64) / 1_000_000;
        (nominal + offset) as u32
    }

    pub fn stream(&self) -> SampleStream<rusb::Context> {
        SampleStream::new(self.device.clone())
    }

    pub fn stream_f32(&self, factor: usize) -> F32Stream<rusb::Context> {
        let s: SampleStream<rusb::Context> = self.stream();
        F32Stream::new(s, factor)
    }

    pub async fn start_sharing<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> Result<SharingServer> {
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

        SharingServer::start(path, rx)
            .await
            .map_err(|e| Error::Tuner(format!("Server error: {:?}", e)))
    }

    pub async fn start_rtl_tcp(self, addr: &str) -> Result<TcpServer> {
        TcpServer::start(self, addr)
            .await
            .map_err(|e| Error::Tuner(format!("TCP Server error: {:?}", e)))
    }
}
