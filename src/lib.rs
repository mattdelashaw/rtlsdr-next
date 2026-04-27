use log::debug;
use std::sync::Arc;
use tokio::sync::broadcast;

pub mod config;
pub mod daemon;
pub mod demod;
pub mod device;

use device::HardwareInterface;
pub mod converter;
pub mod dsp;
pub mod error;
pub mod registers;
pub mod rtl_tcp;
pub mod server;
pub mod stream;
pub mod tuner;
pub mod tuners;
pub mod websdr;

pub use config::DaemonConfig;
pub use daemon::Daemon;
pub use demod::DEFAULT_SAMPLE_RATE;
pub use device::{Device, DeviceInfo};
pub use error::{Error, Result};
pub use rtl_tcp::TcpServer;
#[cfg(unix)]
pub use server::SharingServer;
pub use stream::{F32Stream, PooledBuffer, SampleStream, StreamConfig};
pub use tuner::{BoardConfig, FilterRange, InputPath, Tuner};

pub struct Driver {
    device: Arc<Device<rusb::Context>>,
    pub info: DeviceInfo,
    pub tuner_type: TunerType,
    pub tuner: Box<dyn Tuner>,
    pub orchestrator: Box<dyn BoardOrchestrator>,
    pub sample_rate: u32,
    pub frequency: u64,
    pub ppm: i32,
    pub nominal_xtal: u32,
    pub stream_config: StreamConfig,
    flush_tx: broadcast::Sender<()>,
}

use tuner::BoardOrchestrator;
use tuner::TunerType;

impl Driver {
    pub fn with_index(index: u32) -> Result<Self> {
        let device = Arc::new(Device::open(index)?);
        let hw = device.as_ref();

        // ── 1. RTL2832U baseband init ──────────────────────────────────────
        demod::power_on(hw)?;
        std::thread::sleep(std::time::Duration::from_millis(100));

        // ── 2. Identify board ──────────────────────────────────────────────
        let info = device.read_info();
        let board = if info.is_v4 {
            BoardConfig::BlogV4
        } else {
            BoardConfig::Generic
        };
        let orchestrator = board.orchestrator();

        log::info!(
            "Found RTL2832U — manufacturer: {:?} product: {:?} board: {:?}",
            info.manufacturer,
            info.product,
            board
        );

        // ── 3. V4 GPIO power-up (board-level, not tuner-level) ────────────
        if let BoardConfig::BlogV4 = board {
            log::info!("Applying RTL-SDR Blog V4 GPIO power-up sequence...");
            hw.set_gpio_output(4)?;
            hw.set_gpio_output(5)?;
            hw.set_gpio_bit(4, true)?;
            hw.set_gpio_bit(5, true)?;
            std::thread::sleep(std::time::Duration::from_millis(250));
        }

        // ── 4. Probe and initialize tuner chip ────────────────────────────
        let tuner_type = hw.probe_tuner()?;
        log::info!("Detected tuner chip: {:?}", tuner_type);

        let mut xtal_hz: u64 = match board {
            BoardConfig::BlogV4 => 28_800_000,
            BoardConfig::Generic => 16_000_000,
        };

        // Most R820T/R828D/E4000/FC0012/13 sticks (even generic) use 28.8 MHz.
        if matches!(
            tuner_type,
            TunerType::R820T
                | TunerType::R828D
                | TunerType::E4000
                | TunerType::FC0012
                | TunerType::FC0013
        ) {
            xtal_hz = 28_800_000;
        }

        let tuner: Box<dyn Tuner> = match tuner_type {
            TunerType::R820T => Box::new(tuners::r82xx::R82xx::new(
                device.clone(),
                tuner_type,
                registers::tuner_ids::R82XX_I2C_ADDR,
                xtal_hz,
            )),
            TunerType::R828D => Box::new(tuners::r82xx::R82xx::new(
                device.clone(),
                tuner_type,
                registers::tuner_ids::R828D_I2C_ADDR,
                xtal_hz,
            )),
            TunerType::Unknown(_) if info.is_v4 => Box::new(tuners::r82xx::R82xx::new(
                device.clone(),
                TunerType::R828D,
                registers::tuner_ids::R828D_I2C_ADDR,
                xtal_hz,
            )),
            TunerType::E4000 => Box::new(tuners::e4k::E4k::new(device.clone(), xtal_hz)),
            TunerType::FC0012 => Box::new(tuners::fc001x::Fc001x::new(
                device.clone(),
                tuner_type,
                registers::tuner_ids::FC0012_I2C_ADDR,
                xtal_hz,
            )),
            TunerType::FC0013 => Box::new(tuners::fc001x::Fc001x::new(
                device.clone(),
                tuner_type,
                registers::tuner_ids::FC0013_I2C_ADDR,
                xtal_hz,
            )),
            _ => {
                return Err(Error::UnsupportedTuner(format!(
                    "{:?} not yet supported",
                    tuner_type
                )));
            }
        };

        tuner.initialize()?;

        // ── 5. Post-detection demod config for Low-IF / Zero-IF ───────────
        if matches!(tuner_type, TunerType::R820T | TunerType::R828D) || info.is_v4 {
            demod::set_tuner_low_if(hw)?;
            demod::write_reg_direct(
                hw,
                registers::demod::P1_PAGE,
                registers::demod::P1_DDC_SYNC,
                registers::demod::P1_DDC_SYNC_NORMAL,
            )?;
        } else if matches!(
            tuner_type,
            TunerType::E4000 | TunerType::FC0012 | TunerType::FC0013
        ) {
            demod::set_tuner_zero_if(hw)?;
            demod::write_reg_direct(
                hw,
                registers::demod::P1_PAGE,
                registers::demod::P1_DDC_SYNC,
                registers::demod::P1_DDC_SYNC_NORMAL,
            )?;
        }

        // ── 6. Demodulator sync ────────────────────────────────────────────
        let initial_if = if matches!(
            tuner_type,
            TunerType::E4000 | TunerType::FC0012 | TunerType::FC0013
        ) {
            0
        } else {
            2_300_000u32
        };
        tuner.set_if_freq(initial_if as u64)?;

        demod::reset_demod(hw)?;
        demod::set_if_freq_xtal(hw, initial_if, xtal_hz as u32)?;
        demod::set_sample_rate_xtal(hw, DEFAULT_SAMPLE_RATE, xtal_hz as u32)?;
        demod::write_reg_direct(
            hw,
            registers::demod::P1_PAGE,
            registers::demod::P1_DDC_SYNC,
            registers::demod::P1_DDC_SYNC_NORMAL,
        )?;

        demod::start_streaming(hw)?;

        log::info!("RTL-SDR driver ready");
        let (flush_tx, _) = broadcast::channel(16);
        Ok(Self {
            device,
            info,
            tuner_type,
            tuner,
            orchestrator,
            sample_rate: DEFAULT_SAMPLE_RATE,
            frequency: 0,
            ppm: 0,
            nominal_xtal: xtal_hz as u32,
            stream_config: StreamConfig::default(),
            flush_tx,
        })
    }

    pub fn set_frequency(
        &mut self,
        hz: u64,
        band: Option<&std::sync::Arc<parking_lot::RwLock<crate::daemon::HardwareBand>>>,
    ) -> Result<u64> {
        let _t = std::time::Instant::now();
        let plan = self.orchestrator.plan_tuning(hz);

        if let Err(e) = self.tuner.apply_notch(plan.in_notch) {
            log::error!("Failed to apply notch filter hint: {:?}", e);
            return Err(e);
        }

        let actual = match self.tuner.set_frequency(plan.tuner_hz) {
            Ok(f) => f,
            Err(e) => {
                log::error!(
                    "Tuner failed to set frequency {} Hz: {:?}",
                    plan.tuner_hz,
                    e
                );
                return Err(e);
            }
        };

        if let Some(path) = plan.input_path
            && let Err(e) = self.apply_input_path(hz, path)
        {
            log::error!("Failed to apply input path: {:?}", e);
            return Err(e);
        }

        // Hardware path settling: GPIO and triplexer switches require time
        // for the analog transient to die down before re-syncing the demod.
        std::thread::sleep(std::time::Duration::from_millis(100));

        let hw = self.device.as_ref();
        let xtal = self.corrected_xtal_hz();

        // Force explicit IF sync to 2.3MHz to ensure alignment with demodulator
        let if_hz = 2_300_000u64;
        self.tuner.set_if_freq(if_hz)?;

        // Clean the demodulator state before re-syncing
        demod::reset_demod(hw)?;

        // Use the actual frequency set by the hardware for precise demodulator sync.
        demod::set_if_freq_xtal(hw, if_hz as u32, xtal)?;

        // The RTL2832U demodulator register 0x15 controls the DDC sync and spectral inversion.
        // For Low-IF (R828D/R820T), we must explicitly write the correct mode.
        let sync_val = if plan.spectral_inv {
            registers::demod::P1_DDC_SYNC_INVERT
        } else {
            registers::demod::P1_DDC_SYNC_NORMAL
        };
        debug!(
            "demod::write_reg_direct: P1_PAGE, P1_DDC_SYNC, sync_val={:02x} (spectral_inv={})",
            sync_val, plan.spectral_inv
        );
        demod::write_reg_direct(hw, registers::demod::P1_PAGE, registers::demod::P1_DDC_SYNC, sync_val)?;

        if let Some(band) = band {
            let span = band.read().span_hz;
            *band.write() = crate::daemon::HardwareBand {
                center_hz: hz,
                span_hz: span,
                spectral_inv: plan.spectral_inv,
            };
        }

        self.frequency = hz;
        log::info!(
            "Frequency set to {} Hz (requested: {}, Tuner: {}, HF_Inv: {}, took: {}ms)",
            hz,
            hz,
            actual,
            plan.spectral_inv,
            _t.elapsed().as_millis()
        );

        // Final flush to clear any remaining transient samples after tuning
        let _ = self.flush_tx.send(());
        Ok(hz)
    }

    fn apply_input_path(&self, _freq_hz: u64, path: InputPath) -> Result<()> {
        let hw = self.device.as_ref();
        hw.set_gpio_output(5)?;
        match path {
            InputPath::Hf => {
                log::debug!("V4 input: HF (cable 2, GPIO5 low)");
                hw.set_gpio_bit(5, false)?;
            }
            InputPath::Vhf => {
                log::debug!("V4 input: VHF (cable 1, GPIO5 high)");
                hw.set_gpio_bit(5, true)?;
            }
            InputPath::Uhf => {
                log::debug!("V4 input: UHF (air in, GPIO5 high)");
                hw.set_gpio_bit(5, true)?;
            }
        }
        self.tuner.set_input_path(path)
    }

    pub fn set_sample_rate(&mut self, rate_hz: u32) -> Result<()> {
        let _t = std::time::Instant::now();
        let hw = self.device.as_ref();
        let xtal = self.corrected_xtal_hz();

        let current_if = self.tuner.get_if_freq();
        let if_hz = if current_if > 0 {
            if rate_hz < 2_500_000 {
                2_300_000
            } else {
                3_570_000
            }
        } else {
            0
        };

        self.tuner.set_if_freq(if_hz)?;
        self.tuner.set_bandwidth(rate_hz)?;

        demod::reset_demod(hw)?;
        demod::set_sample_rate_xtal(hw, rate_hz, xtal)?;
        demod::set_if_freq_xtal(hw, if_hz as u32, xtal)?;
        demod::write_reg_direct(
            hw,
            registers::demod::P1_PAGE,
            registers::demod::P1_DDC_SYNC,
            registers::demod::P1_DDC_SYNC_NORMAL,
        )?;

        self.sample_rate = rate_hz;
        log::info!(
            "Sample rate set to {} Hz (took: {}ms)",
            rate_hz,
            _t.elapsed().as_millis()
        );
        Ok(())
    }

    pub fn set_ppm(&mut self, ppm: i32) -> Result<()> {
        self.ppm = ppm;
        self.tuner.set_ppm(ppm)?;
        let freq = self.frequency;
        let rate = self.sample_rate;
        if freq > 0 {
            let _ = self.set_frequency(freq, None);
        }
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

    pub fn set_agc(&self, on: bool) -> Result<()> {
        let hw = self.device.as_ref();
        // Bit 5 of P0_AGC_CTL enables/disables the RTL2832U demodulator AGC.
        // 0x25 = ON, 0x05 = OFF (preserves other default bits)
        let val = if on { 0x25 } else { 0x05 };
        demod::write_reg_direct(
            hw,
            registers::demod::P0_PAGE,
            registers::demod::P0_AGC_CTL,
            val,
        )?;
        log::trace!("Demod AGC turned {}", if on { "ON" } else { "OFF" });
        Ok(())
    }

    fn corrected_xtal_hz(&self) -> u32 {
        let nominal = self.nominal_xtal as i64;
        let offset = (nominal * self.ppm as i64) / 1_000_000;
        let corrected = (nominal + offset) as u32;
        debug!(
            "corrected_xtal_hz: nominal={}, ppm={}, corrected={}",
            nominal, self.ppm, corrected
        );
        corrected
    }

    pub fn stream(&self) -> SampleStream<rusb::Context> {
        SampleStream::new(
            self.device.clone(),
            self.flush_tx.subscribe(),
            self.stream_config,
        )
    }

    pub fn stream_f32(&self, factor: usize) -> F32Stream<rusb::Context> {
        F32Stream::new(self.stream(), factor, self.stream_config)
    }

    #[cfg(unix)]
    pub async fn start_sharing<P: AsRef<std::path::Path>>(&self, path: P) -> Result<SharingServer> {
        let mut stream = self.stream();
        let (tx, rx) = tokio::sync::broadcast::channel::<Arc<Vec<u8>>>(16);
        tokio::spawn(async move {
            while let Some(res) = stream.next().await {
                match res {
                    Ok(samples) => {
                        let _ = tx.send(Arc::new(samples.to_vec()));
                    }
                    Err(e) => {
                        log::error!("Stream error: {:?}", e);
                        break;
                    }
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
