//! rtlsdr-next unified daemon.
//!
//! Starts one hardware driver and feeds raw IQ to any combination of:
//!   - rtl_tcp  (SDR++, GQRX, OpenWebRX+)
//!   - WebSDR   (browser WebSocket)
//!   - Unix Domain Socket  (local process sharing, Unix only)
//!
//! Configuration is layered: compiled defaults → TOML file → CLI flags.
//!
//! # Quick start
//! ```
//! # Minimal: rtl_tcp only, all defaults
//! rtlsdr-daemon --rtl-tcp 0.0.0.0:1234
//!
//! # From a config file with a CLI override
//! rtlsdr-daemon -c /etc/rtlsdr-next/config.toml --gain 25.0
//!
//! # Both servers simultaneously
//! rtlsdr-daemon --rtl-tcp 0.0.0.0:1234 --websdr 0.0.0.0:8080
//! ```

use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use log::info;
use rtlsdr_next::config::{CliOverrides, DaemonConfig};
use rtlsdr_next::daemon::Daemon;

// ── CLI definition ────────────────────────────────────────────────────────────

/// rtlsdr-next unified daemon — single hardware handle, multiple protocol servers.
#[derive(Parser, Debug)]
#[command(
    name = "rtlsdr-daemon",
    version,
    about,
    long_about = None,
)]
struct Cli {
    /// Path to TOML configuration file.
    /// CLI flags override individual values from the file.
    #[arg(short = 'c', long, value_name = "PATH")]
    config: Option<PathBuf>,

    // ── Hardware ──────────────────────────────────────────────────────────────

    /// USB device index (0 = first dongle found).
    #[arg(long, value_name = "INDEX")]
    device: Option<u32>,

    /// Hardware IQ sample rate in Hz (225000–3200000).
    #[arg(long, value_name = "HZ")]
    sample_rate: Option<u32>,

    /// Initial center frequency in Hz.
    #[arg(long, value_name = "HZ")]
    freq: Option<u64>,

    /// Initial RF gain in dB. Snapped to nearest valid tuner step.
    #[arg(long, value_name = "DB")]
    gain: Option<f32>,

    /// Crystal frequency correction in PPM.
    #[arg(long, value_name = "PPM")]
    ppm: Option<i32>,

    /// Enable bias tee (5 V on SMA). Only use with DC-capable antennas/LNAs.
    #[arg(long, default_value_t = false)]
    bias_t: bool,

    // ── Stream ────────────────────────────────────────────────────────────────

    /// Number of USB transfer buffers in the pool.
    #[arg(long, value_name = "N")]
    buffers: Option<usize>,

    /// Size of each USB transfer buffer in bytes (must be a multiple of 512).
    #[arg(long, value_name = "BYTES")]
    buffer_size: Option<usize>,

    // ── Servers ───────────────────────────────────────────────────────────────

    /// Enable rtl_tcp server. Accepts SDR++, GQRX, OpenWebRX+.
    /// Example: 0.0.0.0:1234
    #[arg(long, value_name = "ADDR")]
    rtl_tcp: Option<String>,

    /// Enable WebSDR server (browser WebSocket).
    /// Example: 0.0.0.0:8080
    #[arg(long, value_name = "ADDR")]
    websdr: Option<String>,

    /// Enable Unix Domain Socket server for local process sharing (Unix only).
    /// Example: /tmp/rtlsdr.sock
    #[arg(long, value_name = "PATH")]
    unix: Option<String>,

    // ── TLS (WebSDR wss://) ───────────────────────────────────────────────────

    /// Path to TLS certificate PEM file. Enables wss:// on the WebSDR server.
    /// Must be paired with --key.
    #[arg(long, value_name = "PATH")]
    cert: Option<PathBuf>,

    /// Path to TLS private key PEM file. Must be paired with --cert.
    #[arg(long, value_name = "PATH")]
    key: Option<PathBuf>,

    // ── Logging ───────────────────────────────────────────────────────────────

    /// Log level: error | warn | info | debug | trace
    #[arg(long, value_name = "LEVEL", default_value = "info")]
    log_level: String,
}

impl Cli {
    /// Convert the parsed CLI args into a `CliOverrides`, ready to pass to
    /// `DaemonConfig::load`. The `bias_t` flag is only forwarded when the user
    /// explicitly passed `--bias-t` (i.e. it was set to true), preserving
    /// the config-file value when the flag is absent.
    fn into_overrides(self) -> (Option<PathBuf>, CliOverrides) {
        let bias_t = if self.bias_t { Some(true) } else { None };
        let overrides = CliOverrides {
            device_index:  self.device,
            sample_rate:   self.sample_rate,
            initial_freq:  self.freq,
            initial_gain:  self.gain,
            ppm:           self.ppm,
            bias_t,
            num_buffers:   self.buffers,
            buffer_size:   self.buffer_size,
            rtl_tcp:       self.rtl_tcp,
            websdr:        self.websdr,
            unix_socket:   self.unix,
            cert:          self.cert,
            key:           self.key,
        };
        (self.config, overrides)
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    let log_level = cli.log_level.clone();

    // Initialise logging before anything else so early errors are visible.
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or(&log_level),
    )
    .init();

    let (config_path, overrides) = cli.into_overrides();

    // Build the merged, validated config.
    let cfg = DaemonConfig::load(config_path.as_deref(), &overrides)?;

    // Log the effective config at debug so it's always available in verbose runs.
    log::debug!("Effective configuration:\n{:#?}", cfg);

    info!(
        "rtlsdr-next daemon starting (device {}, {} Hz, {:.1} dB gain)",
        cfg.hardware.device_index,
        cfg.hardware.sample_rate,
        cfg.hardware.initial_gain,
    );

    if let Some(addr) = &cfg.servers.rtl_tcp {
        info!("rtl_tcp   : {}", addr);
    }
    if let Some(addr) = &cfg.servers.websdr {
        info!(
            "WebSDR    : {} ({})",
            addr,
            if cfg.tls.pair().is_some() { "wss://" } else { "ws://" }
        );
    }
    if let Some(path) = &cfg.servers.unix_socket {
        info!("Unix sock : {}", path);
    }

    // Start the daemon (initialises hardware, spawns broadcast pump and servers).
    let daemon = Daemon::start(&cfg).await?;
    daemon.run(&cfg).await?;

    info!("Daemon shut down cleanly.");
    Ok(())
}
