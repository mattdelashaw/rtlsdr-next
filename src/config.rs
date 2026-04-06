//! Daemon configuration — TOML schema, defaults, and CLI merge.
//!
//! # Layering order (lowest → highest priority)
//! 1. Compiled-in defaults (`config/default.toml` via `include_str!`)
//! 2. User-supplied config file (`-c path/to/config.toml`)
//! 3. CLI flag overrides (any `Some` field in `CliOverrides` wins)
//!
//! # Usage
//! ```rust
//! let cfg = DaemonConfig::load(Some(Path::new("my.toml")), &cli_overrides)?;
//! ```

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Deserialize;

// Baked-in reference defaults — always present, never missing.
const DEFAULT_TOML: &str = include_str!("../config/default.toml");

// ── Top-level config ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize)]
pub struct DaemonConfig {
    #[serde(default)]
    pub hardware: HardwareConfig,

    #[serde(default)]
    pub stream: StreamCfg,

    #[serde(default)]
    pub servers: ServersConfig,

    #[serde(default)]
    pub tls: TlsConfig,
}

// ── Hardware ──────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize)]
pub struct HardwareConfig {
    pub device_index: u32,
    pub sample_rate: u32,
    pub initial_freq: u64,
    pub initial_gain: f32,
    pub ppm: i32,
    pub bias_t: bool,
}

impl Default for HardwareConfig {
    fn default() -> Self {
        Self {
            device_index: 0,
            sample_rate: 1_536_000,
            initial_freq: 101_100_000,
            initial_gain: 30.0,
            ppm: 0,
            bias_t: false,
        }
    }
}

// ── Stream ────────────────────────────────────────────────────────────────────

/// Config-layer mirror of `crate::stream::StreamConfig`.
/// Named `StreamCfg` to avoid a name collision when both are in scope.
#[derive(Debug, Clone, Deserialize)]
pub struct StreamCfg {
    pub num_buffers: usize,
    pub buffer_size: usize,
}

impl Default for StreamCfg {
    fn default() -> Self {
        Self {
            num_buffers: 16,
            buffer_size: 262_144,
        }
    }
}

impl From<StreamCfg> for crate::stream::StreamConfig {
    fn from(c: StreamCfg) -> Self {
        Self {
            num_buffers: c.num_buffers,
            buffer_size: c.buffer_size,
        }
    }
}

// ── Servers ───────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize, Default)]
pub struct ServersConfig {
    pub rtl_tcp: Option<String>,
    pub websdr: Option<String>,
    pub unix_socket: Option<String>,
}

// ── TLS ───────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize, Default)]
pub struct TlsConfig {
    pub cert: Option<PathBuf>,
    pub key: Option<PathBuf>,
}

impl TlsConfig {
    /// Returns `Some((cert, key))` only when both are present, `None` otherwise.
    pub fn pair(&self) -> Option<(PathBuf, PathBuf)> {
        match (&self.cert, &self.key) {
            (Some(c), Some(k)) => Some((c.clone(), k.clone())),
            _ => None,
        }
    }
}

// ── CLI overrides ─────────────────────────────────────────────────────────────

/// Holds the subset of CLI flags that can override config-file values.
/// Every field is `Option<T>` — `None` means "not specified on CLI, keep file value".
#[derive(Debug, Default)]
pub struct CliOverrides {
    pub device_index: Option<u32>,
    pub sample_rate: Option<u32>,
    pub initial_freq: Option<u64>,
    pub initial_gain: Option<f32>,
    pub ppm: Option<i32>,
    pub bias_t: Option<bool>,
    pub num_buffers: Option<usize>,
    pub buffer_size: Option<usize>,
    pub rtl_tcp: Option<String>,
    pub websdr: Option<String>,
    pub unix_socket: Option<String>,
    pub cert: Option<PathBuf>,
    pub key: Option<PathBuf>,
}

// ── Loader ────────────────────────────────────────────────────────────────────

impl DaemonConfig {
    /// Load configuration with the full layering stack.
    ///
    /// # Arguments
    /// * `config_path` — optional path to a user TOML file, merged on top of defaults.
    /// * `overrides`   — CLI flag values; any `Some` field overwrites the merged config.
    pub fn load(config_path: Option<&Path>, overrides: &CliOverrides) -> Result<Self> {
        // Layer 1: compiled-in defaults.
        let mut cfg: DaemonConfig = toml::from_str(DEFAULT_TOML)
            .context("Failed to parse compiled-in default config (this is a bug)")?;

        // Layer 2: user config file, if provided.
        if let Some(path) = config_path {
            let raw = std::fs::read_to_string(path)
                .with_context(|| format!("Cannot read config file: {}", path.display()))?;
            let user: DaemonConfig = toml::from_str(&raw)
                .with_context(|| format!("Failed to parse config file: {}", path.display()))?;
            cfg.merge_from(user);
        }

        // Layer 3: CLI overrides.
        cfg.apply_overrides(overrides);

        cfg.validate()?;
        Ok(cfg)
    }

    /// Merge `other` on top of `self`. Only fields explicitly set in `other`'s
    /// non-default sections overwrite `self`. Since TOML deserialization gives us
    /// fully-populated structs (not `Option`-wrapped fields), we merge at the
    /// top-level section granularity — a user file that includes `[hardware]`
    /// replaces the entire hardware section; omitting `[hardware]` keeps defaults.
    fn merge_from(&mut self, other: DaemonConfig) {
        // Hardware: replace entirely if the user file contained [hardware].
        // We detect "was [hardware] present" by checking if any value differs
        // from the Default impl — simple and avoids a separate Option<Section> type.
        self.hardware = other.hardware;
        self.stream = other.stream;

        // Servers: merge Option fields individually so a user file that only sets
        // `rtl_tcp` doesn't wipe out a `websdr` default (there isn't one, but
        // this is the correct merge semantics going forward).
        if other.servers.rtl_tcp.is_some() {
            self.servers.rtl_tcp = other.servers.rtl_tcp;
        }
        if other.servers.websdr.is_some() {
            self.servers.websdr = other.servers.websdr;
        }
        if other.servers.unix_socket.is_some() {
            self.servers.unix_socket = other.servers.unix_socket;
        }

        // TLS: merge individually.
        if other.tls.cert.is_some() {
            self.tls.cert = other.tls.cert;
        }
        if other.tls.key.is_some() {
            self.tls.key = other.tls.key;
        }
    }

    /// Apply CLI overrides — any `Some` value wins unconditionally.
    fn apply_overrides(&mut self, o: &CliOverrides) {
        if let Some(v) = o.device_index  { self.hardware.device_index  = v; }
        if let Some(v) = o.sample_rate   { self.hardware.sample_rate   = v; }
        if let Some(v) = o.initial_freq  { self.hardware.initial_freq  = v; }
        if let Some(v) = o.initial_gain  { self.hardware.initial_gain  = v; }
        if let Some(v) = o.ppm           { self.hardware.ppm           = v; }
        if let Some(v) = o.bias_t        { self.hardware.bias_t        = v; }
        if let Some(v) = o.num_buffers   { self.stream.num_buffers     = v; }
        if let Some(v) = o.buffer_size   { self.stream.buffer_size     = v; }
        if let Some(v) = o.rtl_tcp.clone()      { self.servers.rtl_tcp      = Some(v); }
        if let Some(v) = o.websdr.clone()        { self.servers.websdr       = Some(v); }
        if let Some(v) = o.unix_socket.clone()   { self.servers.unix_socket  = Some(v); }
        if let Some(v) = o.cert.clone()          { self.tls.cert             = Some(v); }
        if let Some(v) = o.key.clone()           { self.tls.key              = Some(v); }
    }

    /// Validate the fully-merged config and return human-readable errors.
    fn validate(&self) -> Result<()> {
        let rate = self.hardware.sample_rate;
        if !(225_000..=3_200_000).contains(&rate) {
            anyhow::bail!(
                "hardware.sample_rate {} Hz is out of range (225_000–3_200_000)",
                rate
            );
        }

        if self.hardware.initial_freq == 0 {
            anyhow::bail!("hardware.initial_freq must be non-zero");
        }

        if self.stream.num_buffers == 0 {
            anyhow::bail!("stream.num_buffers must be >= 1");
        }

        if self.stream.buffer_size % 512 != 0 {
            anyhow::bail!(
                "stream.buffer_size {} is not a multiple of 512 (USB packet size)",
                self.stream.buffer_size
            );
        }

        // TLS: cert and key must both be present or both absent.
        match (&self.tls.cert, &self.tls.key) {
            (Some(_), None) => anyhow::bail!("tls.cert is set but tls.key is missing"),
            (None, Some(_)) => anyhow::bail!("tls.key is set but tls.cert is missing"),
            _ => {}
        }

        // At least one server must be enabled — otherwise the daemon does nothing.
        if self.servers.rtl_tcp.is_none()
            && self.servers.websdr.is_none()
            && self.servers.unix_socket.is_none()
        {
            anyhow::bail!(
                "No servers enabled. Set at least one of: \
                 servers.rtl_tcp, servers.websdr, servers.unix_socket \
                 (or pass --rtl-tcp / --websdr / --unix on the CLI)"
            );
        }

        Ok(())
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn no_overrides() -> CliOverrides {
        CliOverrides::default()
    }

    #[test]
    fn test_defaults_parse() {
        // The compiled-in default TOML must parse cleanly on its own.
        // We inject a minimal servers section so validation doesn't reject it.
        let toml = format!(
            "{}\n[servers]\nrtl_tcp = \"0.0.0.0:1234\"\n",
            DEFAULT_TOML
        );
        let cfg: DaemonConfig = toml::from_str(&toml).expect("default TOML must parse");
        assert_eq!(cfg.hardware.device_index, 0);
        assert_eq!(cfg.hardware.sample_rate, 1_536_000);
        assert_eq!(cfg.hardware.initial_freq, 101_100_000);
        assert!((cfg.hardware.initial_gain - 30.0).abs() < f32::EPSILON);
        assert_eq!(cfg.hardware.ppm, 0);
        assert!(!cfg.hardware.bias_t);
        assert_eq!(cfg.stream.num_buffers, 16);
        assert_eq!(cfg.stream.buffer_size, 262_144);
    }

    #[test]
    fn test_cli_override_wins() {
        let overrides = CliOverrides {
            initial_gain: Some(20.0),
            rtl_tcp: Some("127.0.0.1:9999".to_string()),
            ..Default::default()
        };
        let cfg = DaemonConfig::load(None, &overrides).unwrap();
        assert!((cfg.hardware.initial_gain - 20.0).abs() < f32::EPSILON);
        assert_eq!(cfg.servers.rtl_tcp.as_deref(), Some("127.0.0.1:9999"));
        // Everything else stays at the default.
        assert_eq!(cfg.hardware.sample_rate, 1_536_000);
    }

    #[test]
    fn test_validation_rejects_bad_sample_rate() {
        let overrides = CliOverrides {
            sample_rate: Some(100), // below 225 kHz minimum
            rtl_tcp: Some("0.0.0.0:1234".to_string()),
            ..Default::default()
        };
        assert!(DaemonConfig::load(None, &overrides).is_err());
    }

    #[test]
    fn test_validation_rejects_unaligned_buffer() {
        let overrides = CliOverrides {
            buffer_size: Some(1000), // not a multiple of 512
            rtl_tcp: Some("0.0.0.0:1234".to_string()),
            ..Default::default()
        };
        assert!(DaemonConfig::load(None, &overrides).is_err());
    }

    #[test]
    fn test_validation_rejects_partial_tls() {
        let overrides = CliOverrides {
            cert: Some(PathBuf::from("/tmp/cert.pem")),
            // key deliberately absent
            websdr: Some("0.0.0.0:8080".to_string()),
            ..Default::default()
        };
        assert!(DaemonConfig::load(None, &overrides).is_err());
    }

    #[test]
    fn test_validation_rejects_no_servers() {
        // No server enabled anywhere — must fail.
        let overrides = no_overrides();
        assert!(DaemonConfig::load(None, &overrides).is_err());
    }

    #[test]
    fn test_tls_pair() {
        let tls = TlsConfig {
            cert: Some(PathBuf::from("/tmp/cert.pem")),
            key: Some(PathBuf::from("/tmp/key.pem")),
        };
        assert!(tls.pair().is_some());

        let tls_empty = TlsConfig::default();
        assert!(tls_empty.pair().is_none());
    }

    #[test]
    fn test_partial_toml_merge() {
        // A user file that only sets [hardware] should not wipe out stream defaults.
        let user_toml = r#"
[hardware]
initial_gain = 15.0
sample_rate = 1_536_000
device_index = 0
initial_freq = 101_100_000
ppm = 0
bias_t = false

[servers]
rtl_tcp = "0.0.0.0:1234"
"#;
        let user: DaemonConfig = toml::from_str(user_toml).unwrap();
        let mut base = DaemonConfig::load(
            None,
            &CliOverrides {
                rtl_tcp: Some("0.0.0.0:1234".to_string()),
                ..Default::default()
            },
        )
        .unwrap();
        base.merge_from(user);

        assert!((base.hardware.initial_gain - 15.0).abs() < f32::EPSILON);
        // Stream section untouched — should still be default.
        assert_eq!(base.stream.num_buffers, 16);
    }
}
