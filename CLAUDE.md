# rtlsdr-next

Async Rust RTL-SDR driver. Tokio-native stream architecture. Primary target: RTL-SDR Blog V4 on Raspberry Pi 5 (aarch64).

## Stack
- Rust edition 2024, stable toolchain
- `rusb` for USB, `tokio` for async, `parking_lot` for sync primitives
- `libusb1-sys` with `vendored` feature — no system libusb needed on Windows
- `criterion` for benchmarks, `env_logger` for logging

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

## Repo Layout
- `src/bin/rtl_tcp.rs` — installable rtl_tcp server binary (`-a/--address`, `-p/--port`)
- `src/bin/websdr.rs` — REMOVED from bin, back to `examples/websdr.rs`
- `examples/` — fm_radio, monitor, hw_probe, diag_*
- `examples/diag/` — raw USB diagnostic tools (bypass driver, speak libusb directly)
- `assets/websdr_ui.html` — WebSDR frontend, embedded via `include_str!()`

## Commands
- Build: `cargo build --release`
- Install: `cargo install --path .`
- Test: `cargo test --release`
- Fmt check: `cargo fmt --all -- --check`
- Clippy: `cargo clippy -- -D warnings`
- Bench (Rust only): `cargo bench --bench dsp_bench`
- Bench vs C: `RUSTFLAGS="-C target-cpu=native" cargo bench --bench vs_librtlsdr_bench --features bench-c`

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

## I2C Repeater Pattern (r82xx.rs)
- `write_reg_mask` — opens/closes repeater itself (standalone use)
- `write_reg_mask_raw` — no repeater toggle, must be inside `with_repeater`
- `with_repeater(|| { ... })` — single open/close for entire mux+pll sequence
- Before: ~20 repeater toggles = ~270ms per `set_frequency`. After: 1 toggle = ~45ms

## rtl_tcp Protocol (SDR++ specifics)
- `0x01` set frequency, `0x02` set sample rate, `0x04` set gain (tenths dB)
- `0x03` gain mode — apply 30dB default for both auto and manual (SDR++ doesn't always follow with 0x04)
- `0x0d` — confirmation request SDR++ sends after every `0x13`, silently ignored
- `0x13` — SDR++ gain-by-index slider, maps to `tuner.set_gain_by_index(arg)`
- `0x0e` bias-T

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

## Known Pending Items
- Async USB transfers (libusb async API) — current blocking thread model works, noted for future
- FC001x gain table is simplified (auto/manual toggle only) — discrete gain steps not mapped

@.claude/rules/hardware.md
@.claude/rules/dsp.md