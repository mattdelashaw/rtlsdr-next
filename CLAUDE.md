# rtlsdr-next

Async Rust RTL-SDR driver. Tokio-native stream architecture. Primary target: RTL-SDR Blog V4 on Raspberry Pi 5 (aarch64).

## Stack
- Rust edition 2024, stable toolchain
- `rusb` for USB, `tokio` for async, `parking_lot` for sync primitives
- `libusb1-sys` with `vendored` feature — no system libusb needed on Windows
- `criterion` for benchmarks, `env_logger` for logging
- `clap` (derive) for daemon CLI, `toml` + `serde` for config file parsing

## Architecture
- `Device<T>` / `HardwareInterface` trait — raw USB control transfers, I2C bridge
- `Tuner` trait — pure chip drivers (`tuners/r82xx.rs`, `tuners/e4k.rs`, `tuners/fc001x.rs`), no board logic
- `BoardOrchestrator` trait — `GenericOrchestrator` / `V4Orchestrator` produce a `TuningPlan` (tuner_hz, spectral_inv, input_path, in_notch). All V4 board logic lives here, never in chip drivers.
- `Driver::set_frequency()` — orchestrates: plan → apply_notch → chip tune → GPIO → demod sync → flush
- `SampleStream` — blocking USB thread feeding `tokio::mpsc`; responds to `flush_rx` broadcast on frequency change
- `F32Stream` — DSP pipeline: convert → decimate → DC remove → AGC
- `PooledBuffer<B>` — zero-allocation buffer pool; Drop uses `try_send` with thread fallback
- `StreamConfig` — configurable buffer count and size (`num_buffers: 16, buffer_size: 262144` default)
- `SharingServer` — Unix Domain Socket server, `#[cfg(unix)]` only
- `Daemon` — unified hardware orchestrator; owns `Driver`, runs broadcast pump, wires all servers
- `DaemonConfig` — layered TOML config (compiled defaults → file → CLI flags)

## Repo Layout
- `src/bin/rtl_tcp.rs` — standalone rtl_tcp server binary (`-a/--address`, `-p/--port`)
- `src/bin/websdr.rs` — standalone WebSDR server binary (`-a/--address`, `-p/--port`)
- `src/bin/daemon.rs` — unified daemon binary (clap CLI, all servers, TOML config)
- `src/config.rs` — `DaemonConfig`, `CliOverrides`, TOML schema and merge logic
- `src/daemon.rs` — `Daemon` struct, broadcast pump, `HardwareBand` for DDC (Phase 3)
- `examples/` — fm_radio, monitor, hw_probe, diag_*
- `examples/diag/` — raw USB diagnostic tools (bypass driver, speak libusb directly)
- `assets/websdr_ui.html` — WebSDR frontend, embedded via `include_str!()`
- `config/default.toml` — reference config with all defaults documented; baked into binary via `include_str!()`

## Commands
- Build: `cargo build --release`
- Install: `cargo install --path .`
- Test: `cargo test --release`
- Fmt check: `cargo fmt --all -- --check`
- Clippy: `cargo clippy -- -D warnings`
- Bench (Rust only): `cargo bench --bench dsp_bench`
- Bench vs C: `RUSTFLAGS="-C target-cpu=native" cargo bench --bench vs_librtlsdr_bench --features bench-c`
- Daemon (rtl_tcp only): `rtlsdr-daemon --rtl-tcp 0.0.0.0:1234`
- Daemon (both servers): `rtlsdr-daemon --rtl-tcp 0.0.0.0:1234 --websdr 0.0.0.0:8080`
- Daemon (from config): `rtlsdr-daemon -c /etc/rtlsdr-next/config.toml`

## Rules
- `parking_lot::Mutex` everywhere — never `std::sync::Mutex` in new code
- Never hold any mutex across an `.await` point
- Buffer pool: never bare `try_send` in `Drop` — use the fallback thread pattern in `PooledBuffer`
- All hardware errors must propagate — no silent `let _ =` on ops affecting observable state
- WebSocket hardware commands must send confirmation or error frame back via unified sink channel
- Audio bytes sent over WebSocket use `to_le_bytes()` — never raw pointer casting
- PPM correction: always compute from `nominal_xtal`, never accumulate on `xtal_freq` directly
- Chip drivers must never contain GPIO, triplexer, or board-specific logic
- `SharingServer` and all of `server.rs` is `#[cfg(unix)]` — never import `tokio::net::UnixListener` without this gate
- `Driver::new()` performs USB I/O and `thread::sleep` during init — always call it via `tokio::task::spawn_blocking` from async contexts. Calling it directly on a Tokio async thread will stall the runtime.
- In the daemon, only `Daemon::start` calls `Driver::new()`. All servers receive an `Arc<Mutex<Driver>>` — they never construct a `Driver` themselves.
- `TcpServer::start_shared` and `WebSdrServer::start_shared` are the daemon-facing entry points. The original `start(driver: Driver, ...)` signatures are kept for the standalone binaries only.

## I2C Repeater Pattern (r82xx.rs)
- `write_reg_mask` — opens/closes repeater itself (standalone use)
- `write_reg_mask_raw` — no repeater toggle, must be inside `with_repeater`
- `with_repeater(|| { ... })` — single open/close for entire mux+pll sequence
- Before: ~20 repeater toggles = ~270ms. After: 1 toggle = ~45ms for tuner chip. Total system latency (including DDC sync) is ~110ms.

## rtl_tcp Protocol (SDR++ specifics)
- `0x01` set frequency, `0x02` set sample rate, `0x04` set gain (tenths dB)
- `0x03` gain mode — apply 30dB default for both auto and manual (SDR++ doesn't always follow with 0x04)
- `0x0d` — confirmation request SDR++ sends after every `0x13`, silently ignored
- `0x13` — SDR++ gain-by-index slider, maps to `tuner.set_gain_by_index(arg)`
- `0x0e` bias-T

## Daemon Config (config/default.toml + src/config.rs)
Config is layered: compiled-in defaults → user TOML file (`-c`) → CLI flags (highest priority).
```toml
[hardware]
device_index  = 0
sample_rate   = 1_536_000
initial_freq  = 101_100_000
initial_gain  = 30.0
ppm           = 0
bias_t        = false

[stream]
num_buffers  = 16
buffer_size  = 262144

[servers]
rtl_tcp     = "0.0.0.0:1234"   # opt-in
websdr      = "0.0.0.0:8080"   # opt-in
unix_socket = "/tmp/rtlsdr.sock"  # opt-in, unix only

[tls]
cert = "/etc/letsencrypt/live/example.com/fullchain.pem"
key  = "/etc/letsencrypt/live/example.com/privkey.pem"
```
Validation rejects: sample_rate outside 225k–3.2M, buffer_size not multiple of 512,
partial TLS (cert without key or vice versa), no servers enabled.

## StreamConfig
Default (~1s latency, dropout-resistant):
```rust
StreamConfig { num_buffers: 16, buffer_size: 262144 }
```
Low-latency (~100ms, higher dropout risk):
```rust
StreamConfig { num_buffers: 4, buffer_size: 65536 }
```
`SampleStream` flushes stale buffers automatically on `set_frequency` via broadcast.
GQRX has its own internal buffer — remaining lag after our flush is client-side.

## Key Constants
- R820T I2C: `0x34`, R828D I2C: `0x74`, check val: `0x69`
- E4000 I2C: `0xc8` (probe reg `0x02`, any Ok response)
- FC0012 I2C: `0xc6`, FC0013 I2C: `0xc6`, chip IDs: `0xa1` / `0x63`
- V4 xtal: `28_800_000 Hz`, generic xtal: `16_000_000 Hz`
- Demod dummy read after every write: `page 0x0a reg 0x01`
- I2C max chunk: 7 bytes data + 1 byte reg per transfer
- FC001x VCO range: `2_600_000_000..=3_900_000_000` Hz — frequencies in gaps return `InvalidFrequency`

## Platform Notes
- Windows: `libusb1-sys` vendored feature — no Zadig needed to build, only to run
- Windows USB runtime: still requires Zadig WinUSB driver swap before the dongle is accessible
- `SharingServer` Unix socket is `#[cfg(unix)]` — use rtl_tcp for local sharing on Windows
- `RUST_LOG=value command` is Unix only — Windows: `$env:RUST_LOG = "value"; command`
- Do not set `target-cpu=native` on x86_64 — AVX-512 throttling on Zen 4 hurts decimator

## Done (March 2026)
- `set_bandwidth` now called after sample rate change (analog filter optimization)
- WebSDR `Demod { mode }` command implemented with dynamic FM/AM switching
- `0x0d` rtl_tcp confirmation response implemented for SDR++ compatibility
- `rtl_tcp` now uses unified writer task to allow command responses during data streaming

## Done (April 2026)
- `DaemonConfig` — layered TOML + CLI config system (`src/config.rs`, `config/default.toml`)
- `Daemon` — unified hardware orchestrator with broadcast pump (`src/daemon.rs`)
- `rtlsdr-daemon` binary — clap CLI, all three servers, TOML config (`src/bin/daemon.rs`)
- `TcpServer::start_shared` — daemon-facing rtl_tcp entry point (injected broadcast receiver)
- `WebSdrServer::start_shared` — daemon-facing WebSDR entry point (injected broadcast receiver)
- `HardwareBand` — in-band detection and NCO offset computation (Phase 3 scaffold)

## Known Pending Items
- Async USB transfers (libusb async API) — current blocking thread model works, noted for future
- FC001x gain table is simplified (auto/manual toggle only) — discrete gain steps not mapped
- Phase 3: NCO + ComplexMixer in dsp.rs for per-client DDC
- Phase 3: Per-client WebSDR pipeline (run_pipeline spawned per WebSocket client)
- Phase 3: Tuning arbitration (in-band retune via NCO vs hardware retune)

@.claude/rules/hardware.md
@.claude/rules/dsp.md
