# rtlsdr-next

Async Rust RTL-SDR driver. Tokio-native stream architecture. Primary target: RTL-SDR Blog V4 on Raspberry Pi 5 (aarch64).

## Stack
- Rust edition 2024, stable toolchain
- `rusb` for USB, `tokio` for async, `parking_lot` for sync primitives
- `criterion` for benchmarks, `env_logger` for logging

## Architecture
- `Device<T>` / `HardwareInterface` trait — raw USB control transfers, I2C bridge
- `Tuner` trait — pure chip drivers (`tuners/r82xx.rs`), no board logic
- `BoardOrchestrator` trait in `tuner.rs` — `GenericOrchestrator` / `V4Orchestrator` produce a `TuningPlan` (tuner_hz, spectral_inv, input_path, in_notch). All V4 board logic lives here, never in the chip driver.
- `Driver::set_frequency()` — orchestrates: plan → apply_notch → chip tune → GPIO → demod sync → flush
- `SampleStream` — blocking USB thread feeding a `tokio::mpsc` channel; responds to `flush_rx` broadcast to drain stale buffers on frequency change
- `F32Stream` — DSP pipeline: convert → decimate → DC remove → AGC
- `PooledBuffer<B>` — zero-allocation buffer pool; Drop uses `try_send` with thread fallback, never silently drops
- `StreamConfig` — configurable buffer count and size, passed through `Driver::stream()` and `Driver::stream_f32()`

## Commands
- Build: `cargo build --release`
- Test: `cargo test --release`
- Bench (Rust only): `cargo bench --bench dsp_bench`
- Bench vs C: `RUSTFLAGS="-C target-cpu=native" cargo bench --bench vs_librtlsdr_bench --features bench-c`
- Pi native build: `RUSTFLAGS="-C target-cpu=native" cargo build --release`

## Rules
- Use `parking_lot::Mutex` everywhere — never `std::sync::Mutex` in new code
- Never hold any mutex across an `.await` point
- Buffer pool: never use bare `try_send` to return buffers in `Drop` — use the fallback pattern in `PooledBuffer`
- All hardware errors must propagate — no silent `let _ =` on ops that affect observable state (frequency, gain)
- WebSocket hardware commands must send a confirmation or error frame back to the client via the unified sink channel
- Audio bytes sent over WebSocket use `to_le_bytes()` — never raw pointer casting
- PPM correction: always compute from `nominal_xtal`, never accumulate offset on `xtal_freq` directly
- Tuner chip drivers (`r82xx.rs`) must never contain GPIO, triplexer, or board-specific logic

## StreamConfig
Default (dropout-resistant, ~1s latency):
```rust
StreamConfig { num_buffers: 16, buffer_size: 262144 }
```
Low-latency (snappier frequency switching, higher dropout risk on busy system):
```rust
StreamConfig { num_buffers: 4, buffer_size: 65536 }  // ~100ms latency
```
Note: `SampleStream` flushes stale buffers automatically on `set_frequency` via broadcast signal.
GQRX has its own internal audio buffer that we cannot flush — remaining lag after our flush is client-side.
Suggest GQRX users set: Preferences → Audio → Buffer size to minimum.

## Key Constants
- R820T I2C: `0x34`, R828D I2C: `0x74`, check val: `0x69`
- V4 xtal: `28_800_000 Hz`, generic xtal: `16_000_000 Hz`
- Demod dummy read after every write: `page 0x0a reg 0x01`
- I2C max chunk: 7 bytes data + 1 byte reg per transfer

@.claude/rules/hardware.md
@.claude/rules/dsp.md