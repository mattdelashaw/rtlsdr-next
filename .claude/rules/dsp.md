# DSP Rules

## Converter
- Formula: `(x as f32 - 127.5) / 127.5` — matches librtlsdr Blog V4 LUT exactly
- Standard convert and inverted convert (V4 HF path: Q = -Q) are separate functions
- No SIMD in converter — Cortex-A76 is memory-bandwidth-bound, scalar beats LUT
- Benchmark baseline: C LUT ~1.42 GiB/s, Rust scalar ~1.49 GiB/s, Rust inverted single-pass ~1.43 GiB/s vs C two-pass ~976 MiB/s

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
- `SampleStream`: USB blocking thread → `tokio::mpsc`, 16 × 256KB `PooledBuffer<TransportBuffer>`
- `F32Stream`: convert → optional decimate → optional DC remove → optional AGC
- Pool starvation prevention: `PooledBuffer::Drop` uses `try_send` first, thread fallback for full channel
- Never use bare `try_send` to return a buffer in Drop — silent loss = server hang after hours

## WebSDR Audio
- Output sample rate: 51_200 Hz (matches `websdr_ui.html` `SAMPLE_RATE` constant)
- Frame format: `[b'A'][f32 LE bytes...]`
- Waterfall frame format: `[b'W'][u8 magnitudes x 1024]`
