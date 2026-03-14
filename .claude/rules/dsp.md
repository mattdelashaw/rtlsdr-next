# DSP Rules

## Converter
- Formula: `(x as f32 - 127.5) / 127.5` — matches librtlsdr Blog V4 LUT exactly
- `0 = -1.0`, `255 = +1.0`, `127.5` is the true center (not representable as u8)
- Standard convert and inverted convert (V4 HF path: Q = -Q) are separate functions
- No SIMD in converter — Cortex-A76 is memory-bandwidth-bound, scalar beats LUT
- Benchmark baseline (Pi 5, 256KB block, identical formula):
  - C LUT: ~172µs / 1.42 GiB/s
  - Rust scalar: ~164µs / 1.49 GiB/s
  - Rust inverted (single-pass): ~171µs / 1.43 GiB/s vs C two-pass: ~256µs / 976 MiB/s

## Decimator
- 33-tap FIR, configurable factor (4 / 8 / 16)
- Factor 8 is the default operating point: 2.048 MSPS → 256 kSPS
- NEON path on aarch64: check bounds outside unsafe block, never use `get_unchecked` without verifying index
- Decimator benchmark: factor 4 = 337 MSa/s, factor 8 = 426 MSa/s, factor 16 = 492 MSa/s

## Dynamic IF
- Sample rate < 2.5 MSPS: `if_hz = 2_300_000`
- Sample rate ≥ 2.5 MSPS: `if_hz = 3_570_000`
- Both tuner and demod IF must be updated together on `set_sample_rate`

## Stream Pipeline
- `SampleStream`: USB blocking thread → `tokio::mpsc`, configurable via `StreamConfig`
- Default: 16 × 256KB = ~1s latency. Low-latency: 4 × 64KB = ~100ms
- On `set_frequency`: `Driver` sends on `flush_tx` broadcast; `SampleStream::next()` drains
  the receiver via `try_recv` loop before returning fresh data. Dropped `PooledBuffer`s
  return to pool via their `Drop` impl — no manual cleanup needed.
- `F32Stream`: convert → optional decimate → optional DC remove → optional AGC
- Pool starvation prevention: `PooledBuffer::Drop` uses `try_send` first, thread fallback for full channel
- Never use bare `try_send` to return a buffer in Drop — silent loss = server hang after hours

## WebSocket (websdr.rs)
- Unified sink: single `mpsc::channel(128)` → one writer task. Waterfall, audio, and command
  reply tasks all send into it. Eliminates mutex contention on the WebSocket sender.
- Output sample rate: 51_200 Hz (matches `websdr_ui.html` `SAMPLE_RATE` constant)
- Frame format: `[b'A'][f32 LE bytes...]` for audio, `[b'W'][u8 magnitudes x 1024]` for waterfall
- Driver mutex held only for duration of `set_frequency` / `set_gain` — a few ms at most.
  Audio/waterfall frames queue in the mpsc channel during that window, not dropped.