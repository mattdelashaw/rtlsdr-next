//! Hardware orchestrator — owns the `Driver`, runs the broadcast pump,
//! and wires up all configured protocol servers.
//!
//! The `Daemon` is the single owner of the RTL-SDR hardware. All servers
//! receive raw IQ as subscribers to a `broadcast::Sender<Arc<Vec<u8>>>`.
//! No server is allowed to call `Driver::stream()` directly.
//!
//! # Lifecycle
//! ```text
//! Daemon::start(&cfg)
//!   └─ Driver init (open USB, power on, probe tuner, apply config)
//!   └─ Broadcast pump task (SampleStream → broadcast channel)
//!
//! daemon.run(&cfg)
//!   └─ Spawn rtl_tcp server      (if cfg.servers.rtl_tcp is Some)
//!   └─ Spawn WebSDR server       (if cfg.servers.websdr is Some)
//!   └─ Spawn Unix socket server  (if cfg.servers.unix_socket is Some, unix only)
//!   └─ Block on Ctrl-C / SIGTERM
//! ```

use std::sync::Arc;

use anyhow::Result;
use log::{error, info, warn};
use tokio::sync::{Mutex, broadcast};
use tokio_util::sync::CancellationToken;

use crate::config::DaemonConfig;
use crate::Driver;

// ── HardwareBand ─────────────────────────────────────────────────────────────
//
// Tracks the current hardware center frequency and usable receive window.
// Added here now so it is available for Phase 3 DDC work without moving files.

/// Describes the frequency range currently being digitised by the hardware.
#[derive(Clone, Copy, Debug)]
pub struct HardwareBand {
    /// Hardware center frequency in Hz.
    pub center_hz: u64,
    /// Total span in Hz — equal to the hardware sample rate.
    pub span_hz: u32,
}

impl HardwareBand {
    /// Returns `true` if `freq_hz` falls within the current hardware window.
    pub fn contains(&self, freq_hz: u64) -> bool {
        let half = self.span_hz as u64 / 2;
        freq_hz >= self.center_hz.saturating_sub(half) && freq_hz <= self.center_hz + half
    }

    /// Signed offset in Hz from the hardware center to the requested frequency.
    /// Used as the NCO shift in per-client DDC pipelines (Phase 3).
    pub fn offset_hz(&self, freq_hz: u64) -> i64 {
        freq_hz as i64 - self.center_hz as i64
    }
}

// ── Daemon ────────────────────────────────────────────────────────────────────

pub struct Daemon {
    driver: Arc<Mutex<Driver>>,
    sample_tx: broadcast::Sender<Arc<Vec<u8>>>,
    cancel: CancellationToken,
}

impl Daemon {
    /// Initialise the hardware and start the broadcast pump.
    ///
    /// This is the only place `Driver::new()` is called for daemon operation.
    /// The pump task runs for the lifetime of the `Daemon`.
    pub async fn start(cfg: &DaemonConfig) -> Result<Self> {
        // ── 1. Open device and apply config ───────────────────────────────────
        // Driver::new() reads RTLSDR_DEVICE_INDEX from the environment; we
        // override by setting it before the call and restoring it after.
        // This keeps Driver::new() unchanged while still supporting --device.
        let device_index = cfg.hardware.device_index;
        // SAFETY: single-threaded at startup, no other threads reading this var.
        std::env::set_var("RTLSDR_DEVICE_INDEX", device_index.to_string());

        let mut driver = tokio::task::spawn_blocking(Driver::new)
            .await
            .map_err(|e| anyhow::anyhow!("Driver init panicked: {:?}", e))?
            .map_err(|e| anyhow::anyhow!("Driver init failed: {:?}", e))?;

        std::env::remove_var("RTLSDR_DEVICE_INDEX");

        info!(
            "Hardware: {} {} (V4: {})",
            driver.info.manufacturer, driver.info.product, driver.info.is_v4
        );

        // ── 2. Apply hardware config ───────────────────────────────────────────
        // Order matters: sample rate first (resets demod), then frequency
        // (resets PLL), then gain (no dependencies).
        let hw_cfg = &cfg.hardware;

        driver
            .set_sample_rate(hw_cfg.sample_rate)
            .map_err(|e| anyhow::anyhow!("set_sample_rate failed: {:?}", e))?;

        driver
            .set_frequency(hw_cfg.initial_freq)
            .map_err(|e| anyhow::anyhow!("set_frequency failed: {:?}", e))?;

        driver
            .tuner
            .set_gain(hw_cfg.initial_gain)
            .map_err(|e| anyhow::anyhow!("set_gain failed: {:?}", e))?;

        if hw_cfg.ppm != 0 {
            driver
                .set_ppm(hw_cfg.ppm)
                .map_err(|e| anyhow::anyhow!("set_ppm failed: {:?}", e))?;
        }

        if hw_cfg.bias_t {
            driver
                .set_bias_t(true)
                .map_err(|e| anyhow::anyhow!("set_bias_t failed: {:?}", e))?;
        }

        // Apply stream config from the config file.
        driver.stream_config = cfg.stream.clone().into();

        // ── 3. Broadcast pump ─────────────────────────────────────────────────
        let (sample_tx, _) = broadcast::channel::<Arc<Vec<u8>>>(32);
        let cancel = CancellationToken::new();

        {
            let pump_tx = sample_tx.clone();
            let pump_cancel = cancel.clone();
            let mut stream = driver.stream();

            tokio::spawn(async move {
                info!("Broadcast pump started.");
                loop {
                    tokio::select! {
                        _ = pump_cancel.cancelled() => {
                            info!("Broadcast pump stopping (cancel).");
                            break;
                        }
                        res = stream.next() => match res {
                            Some(Ok(buf)) => {
                                // Arc clone is free — no data copy.
                                let block = Arc::new(buf.to_vec());
                                // Lagged receivers are silently dropped by tokio broadcast.
                                // A send error here means zero subscribers — that's fine.
                                let _ = pump_tx.send(block);
                            }
                            Some(Err(e)) => {
                                error!("Broadcast pump stream error: {:?}", e);
                                break;
                            }
                            None => {
                                warn!("Broadcast pump stream ended unexpectedly.");
                                break;
                            }
                        }
                    }
                }
            });
        }

        Ok(Self {
            driver: Arc::new(Mutex::new(driver)),
            sample_tx,
            cancel,
        })
    }

    /// Spawn all configured servers and block until a shutdown signal is received.
    pub async fn run(self, cfg: &DaemonConfig) -> Result<()> {
        let mut join_set = tokio::task::JoinSet::new();

        // ── rtl_tcp ───────────────────────────────────────────────────────────
        if let Some(addr) = &cfg.servers.rtl_tcp {
            let driver = self.driver.clone();
            let rx = self.sample_tx.subscribe();
            let addr = addr.clone();
            join_set.spawn(async move {
                if let Err(e) =
                    crate::rtl_tcp::TcpServer::start_shared(driver, rx, &addr).await
                {
                    error!("rtl_tcp server error: {:?}", e);
                }
            });
            info!("rtl_tcp server spawned on {}", addr);
        }

        // ── WebSDR ────────────────────────────────────────────────────────────
        if let Some(addr) = &cfg.servers.websdr {
            let driver = self.driver.clone();
            let rx = self.sample_tx.subscribe();
            let addr = addr.clone();
            let tls = cfg.tls.pair();
            join_set.spawn(async move {
                if let Err(e) =
                    crate::websdr::WebSdrServer::start_shared(driver, rx, &addr, tls).await
                {
                    error!("WebSDR server error: {:?}", e);
                }
            });
            info!("WebSDR server spawned on {}", addr);
        }

        // ── Unix Domain Socket ────────────────────────────────────────────────
        #[cfg(unix)]
        if let Some(path) = &cfg.servers.unix_socket {
            let rx = self.sample_tx.subscribe();
            let path = std::path::PathBuf::from(path);
            join_set.spawn(async move {
                match crate::server::SharingServer::start(&path, rx).await {
                    Ok(server) => {
                        // SharingServer runs until its CancellationToken is dropped.
                        // We keep it alive here for the process lifetime.
                        info!("Unix socket server running on {}", path.display());
                        // Park until the task is aborted on shutdown.
                        std::future::pending::<()>().await;
                        drop(server);
                    }
                    Err(e) => error!("Unix socket server error: {:?}", e),
                }
            });
        }

        // ── Shutdown signal ───────────────────────────────────────────────────
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("Received Ctrl-C, shutting down...");
            }
            _ = Self::sigterm() => {
                info!("Received SIGTERM, shutting down...");
            }
            // If any server task exits unexpectedly, shut everything down.
            Some(res) = join_set.join_next() => {
                if let Err(e) = res {
                    error!("Server task panicked: {:?}", e);
                }
                info!("A server task exited; initiating shutdown.");
            }
        }

        self.cancel.cancel();
        join_set.shutdown().await;

        Ok(())
    }

    /// Resolves immediately on SIGTERM (Unix), or never on Windows.
    async fn sigterm() {
        #[cfg(unix)]
        {
            use tokio::signal::unix::{SignalKind, signal};
            if let Ok(mut sig) = signal(SignalKind::terminate()) {
                sig.recv().await;
            } else {
                std::future::pending::<()>().await;
            }
        }
        #[cfg(not(unix))]
        {
            std::future::pending::<()>().await;
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hardware_band_contains_center() {
        let band = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
        };
        assert!(band.contains(100_000_000)); // dead center
        assert!(band.contains(100_700_000)); // near upper edge
        assert!(band.contains(99_300_000));  // near lower edge
    }

    #[test]
    fn test_hardware_band_excludes_outside() {
        let band = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
        };
        assert!(!band.contains(101_000_000)); // outside upper edge (768 kHz)
        assert!(!band.contains(99_000_000));  // outside lower edge
    }

    #[test]
    fn test_hardware_band_offset_positive() {
        let band = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
        };
        assert_eq!(band.offset_hz(100_500_000), 500_000);
    }

    #[test]
    fn test_hardware_band_offset_negative() {
        let band = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
        };
        assert_eq!(band.offset_hz(99_800_000), -200_000);
    }

    #[test]
    fn test_hardware_band_offset_center_is_zero() {
        let band = HardwareBand {
            center_hz: 101_100_000,
            span_hz: 1_536_000,
        };
        assert_eq!(band.offset_hz(101_100_000), 0);
    }
}
