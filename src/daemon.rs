//! Hardware orchestrator — owns the `Driver`, runs the broadcast pump,
//! and wires up all configured protocol servers.
//!
//! The `Daemon` is the single owner of the RTL-SDR hardware. All servers
//! receive a clone of the broadcast `Sender` so each can subscribe fresh
//! per-client receivers directly — no relay task, no extra broadcast hop.

use std::sync::Arc;

use anyhow::Result;
use log::{error, info, warn};
use parking_lot::RwLock;
use tokio::sync::{Mutex, broadcast, mpsc};
use tokio_util::sync::CancellationToken;

use crate::Driver;
use crate::config::DaemonConfig;

// ── HardwareBand ──────────────────────────────────────────────────────────────

/// Describes the frequency range currently being digitised by the hardware.
///
/// Shared via `Arc<RwLock<HardwareBand>>` (parking_lot) — many client tasks
/// read it on every IQ block; the write path (hardware retune) is rare.
#[derive(Clone, Copy, Debug)]
pub struct HardwareBand {
    pub center_hz: u64,
    pub span_hz: u32,
    pub spectral_inv: bool,
}

impl HardwareBand {
    /// Returns `true` if `freq_hz` falls within the current hardware window.
    pub fn contains(&self, freq_hz: u64) -> bool {
        let half = self.span_hz as u64 / 2;
        freq_hz >= self.center_hz.saturating_sub(half) && freq_hz <= self.center_hz + half
    }

    /// Signed offset in Hz from the hardware center to the requested frequency.
    pub fn offset_hz(&self, freq_hz: u64) -> i64 {
        let diff = freq_hz as i64 - self.center_hz as i64;
        // If inverted, a 'higher' frequency in air is 'lower' in baseband.
        if self.spectral_inv { -diff } else { diff }
    }
}

// ── Retune command ────────────────────────────────────────────────────────────

#[derive(Debug)]
pub struct RetuneRequest {
    pub center_hz: u64,
}

// ── Daemon ────────────────────────────────────────────────────────────────────

pub struct Daemon {
    driver: Arc<Mutex<Driver>>,
    sample_tx: broadcast::Sender<Arc<Vec<u8>>>,
    pub band: Arc<RwLock<HardwareBand>>,
    pub retune_tx: mpsc::Sender<RetuneRequest>,
    cancel: CancellationToken,
}

impl Daemon {
    /// Initialise hardware and start the broadcast pump.
    ///
    /// `Driver::new()` performs USB I/O and `thread::sleep` — called via
    /// `spawn_blocking` to avoid stalling the Tokio runtime.
    pub async fn start(cfg: &DaemonConfig) -> Result<Self> {
        let mut driver = Driver::with_index(cfg.hardware.device_index)?;

        driver.set_sample_rate(cfg.hardware.sample_rate).await?;
        driver
            .set_frequency(cfg.hardware.initial_freq, None)
            .await?;
        driver.tuner.set_gain(cfg.hardware.initial_gain)?;
        if cfg.hardware.ppm != 0 {
            driver.set_ppm(cfg.hardware.ppm).await?;
        }
        if cfg.hardware.bias_t {
            driver.set_bias_t(true).await?;
        }
        driver.stream_config = cfg.stream.clone().into();

        let plan = driver.orchestrator.plan_tuning(cfg.hardware.initial_freq);
        let band = Arc::new(RwLock::new(HardwareBand {
            center_hz: cfg.hardware.initial_freq,
            span_hz: cfg.hardware.sample_rate,
            spectral_inv: plan.spectral_inv,
        }));

        let (retune_tx, retune_rx) = mpsc::channel::<RetuneRequest>(8);
        let (sample_tx, _) = broadcast::channel::<Arc<Vec<u8>>>(64);
        let cancel = CancellationToken::new();

        let driver = Arc::new(Mutex::new(driver));
        let band_clone = band.clone();
        let cancel_clone = cancel.clone();
        let tx_clone = sample_tx.clone();

        let driver_clone = driver.clone();
        tokio::spawn(async move {
            run_pump(driver_clone, tx_clone, band_clone, retune_rx, cancel_clone).await;
        });

        Ok(Self {
            driver,
            band,
            retune_tx,
            sample_tx,
            cancel,
        })
    }

    /// Spawn all configured servers and block until shutdown.
    pub async fn run(self, cfg: &DaemonConfig) -> Result<()> {
        let mut join_set = tokio::task::JoinSet::new();

        // rtl_tcp — pass Sender clone, not a Receiver
        if let Some(addr) = &cfg.servers.rtl_tcp {
            let driver = self.driver.clone();
            let tx = self.sample_tx.clone();
            let band = self.band.clone();
            let retune_tx = self.retune_tx.clone();
            let addr = addr.clone();
            info!("rtl_tcp spawned on {}", addr);
            join_set.spawn(async move {
                if let Err(e) =
                    crate::rtl_tcp::TcpServer::start_shared(driver, tx, band, retune_tx, &addr)
                        .await
                {
                    error!("rtl_tcp error: {:?}", e);
                }
            });
        }

        // WebSDR — pass Sender clone, not a Receiver
        if let Some(addr) = &cfg.servers.websdr {
            let driver = self.driver.clone();
            let tx = self.sample_tx.clone();
            let band = self.band.clone();
            let retune_tx = self.retune_tx.clone();
            let addr = addr.clone();
            let tls = cfg.tls.pair();
            let sample_rate = cfg.hardware.sample_rate;
            info!("WebSDR spawned on {}", addr);
            join_set.spawn(async move {
                if let Err(e) = crate::websdr::WebSdrServer::start_shared(
                    driver,
                    tx,
                    band,
                    retune_tx,
                    sample_rate,
                    &addr,
                    tls,
                )
                .await
                {
                    error!("WebSDR error: {:?}", e);
                }
            });
        }

        #[cfg(unix)]
        if let Some(path) = &cfg.servers.unix_socket {
            // SharingServer still uses a Receiver — leave as-is for now.
            let rx = self.sample_tx.subscribe();
            let path = std::path::PathBuf::from(path);
            join_set.spawn(async move {
                match crate::server::SharingServer::start(&path, rx).await {
                    Ok(server) => {
                        info!("Unix socket running on {}", path.display());
                        std::future::pending::<()>().await;
                        drop(server);
                    }
                    Err(e) => error!("Unix socket error: {:?}", e),
                }
            });
        }

        tokio::select! {
            _ = tokio::signal::ctrl_c() => { info!("Ctrl-C, shutting down..."); }
            _ = Self::sigterm()          => { info!("SIGTERM, shutting down..."); }
            Some(res) = join_set.join_next() => {
                if let Err(e) = res { error!("Server task panicked: {:?}", e); }
                info!("A server task exited; shutting down.");
            }
        }

        self.cancel.cancel();
        join_set.shutdown().await;
        Ok(())
    }

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
        std::future::pending::<()>().await;
    }
}

// ── Broadcast pump ────────────────────────────────────────────────────────────

async fn run_pump(
    driver: Arc<Mutex<Driver>>,
    sample_tx: broadcast::Sender<Arc<Vec<u8>>>,
    band: Arc<RwLock<HardwareBand>>,
    mut retune_rx: mpsc::Receiver<RetuneRequest>,
    cancel: CancellationToken,
) {
    info!("Broadcast pump started.");

    let mut stream = {
        let d = driver.lock().await;
        d.stream()
    };

    loop {
        tokio::select! {
            biased;

            _ = cancel.cancelled() => {
                info!("Broadcast pump stopping.");
                break;
            }

            Some(req) = retune_rx.recv() => {
                let current = band.read().center_hz;
                if req.center_hz == current { continue; }

                info!("Retune: {} Hz → {} Hz", current, req.center_hz);
                let (result, plan) = {
                    let mut d = driver.lock().await;
                    let res = d.set_frequency(req.center_hz, None).await;
                    let plan = d.orchestrator.plan_tuning(req.center_hz);
                    (res, plan)
                };

                match result {
                    Ok(logical_hz) => {
                        let span = band.read().span_hz;
                        *band.write() = HardwareBand {
                            center_hz: logical_hz,
                            span_hz: span,
                            spectral_inv: plan.spectral_inv,
                        };
                        info!("Retuned to {} Hz (Inv: {})", logical_hz, plan.spectral_inv);

                        // Send flush signal through broadcast to clear client pipelines
                        let _ = sample_tx.send(Arc::new(Vec::new()));

                        stream.close();
                        stream = { let d = driver.lock().await; d.stream() };
                    }
                    Err(e) => error!("Retune failed: {:?}", e),
                }
            }

            res = stream.next() => match res {
                Some(Ok(buf)) => {
                    let _ = sample_tx.send(Arc::new(buf.to_vec()));
                }
                Some(Err(e)) => { error!("Pump stream error: {:?}", e); break; }
                None         => { warn!("Pump stream ended."); break; }
            }
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_contains_center() {
        let b = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
            spectral_inv: false,
        };
        assert!(b.contains(100_000_000));
    }
    #[test]
    fn test_contains_edge() {
        let b = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
            spectral_inv: false,
        };
        assert!(b.contains(100_768_000));
        assert!(b.contains(99_232_000));
    }
    #[test]
    fn test_excludes_outside() {
        let b = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
            spectral_inv: false,
        };
        assert!(!b.contains(101_000_000));
        assert!(!b.contains(99_000_000));
    }
    #[test]
    fn test_offset_positive() {
        let b = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
            spectral_inv: false,
        };
        assert_eq!(b.offset_hz(100_500_000), 500_000);
    }
    #[test]
    fn test_offset_negative() {
        let b = HardwareBand {
            center_hz: 100_000_000,
            span_hz: 1_536_000,
            spectral_inv: false,
        };
        assert_eq!(b.offset_hz(99_800_000), -200_000);
    }
    #[test]
    fn test_offset_zero() {
        let b = HardwareBand {
            center_hz: 101_100_000,
            span_hz: 1_536_000,
            spectral_inv: false,
        };
        assert_eq!(b.offset_hz(101_100_000), 0);
    }
}
