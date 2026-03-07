# rtlsdr-next 📡
❗*Not hardware tested*❗

A high-performance, asynchronous, and safety-first Rust driver for RTL2832U-based Software Defined Radios (SDR). 

Designed for the modern era (2026+), this driver moves away from the legacy C callback model toward a **Tokio-native Stream** architecture, with specific optimizations for high-bandwidth ARM hosts like the **Raspberry Pi 5**.

## 🚀 Key Features

*   **Async-First Architecture:** Built on `Tokio`. Treat your SDR as a standard `Stream` of samples with backpressure and graceful shutdown.
*   **NEON SIMD Acceleration:** Both sample conversion (`u8` -> `f32`) and DSP decimation (FIR filtering) are optimized with ARM NEON intrinsics for extreme performance on Pi 5.
*   **Modular Tuner Trait:** Clean abstraction for tuner chips. Fully supports the **RTL-SDR Blog V4** (R828D) with integrated triplexer band-switching and interleaved 3-stage gain control.
*   **Precision Frequency Correction:** Integrated PPM correction that dynamically adjusts both the tuner PLL and the RTL2832U resampler/NCO.
*   **Production-Ready Logging:** Uses the `log` crate for structured, configurable output.
*   **Device Sharing:** Includes a built-in Unix Domain Socket server for sharing a single hardware device across multiple local applications.
*   **Safety-First:** 100% memory-safe Rust (outside of essential USB FFI). Robust error handling for USB hotplug and disconnects.

## 🏗 Quick Start

```rust
use rtlsdr_next::Driver;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // 1. Open device
    let mut driver = Driver::new()?;
    driver.set_frequency(100_000_000)?; // 100 MHz
    
    // 2. Get a decimated F32 stream (2.048 MSPS -> 256 kSPS)
    let mut stream = driver.stream_f32(8);

    // 3. Process samples
    while let Some(res) = stream.next().await {
        let iq_samples = res?; // Propagates hardware errors like disconnects
        // Process interleaved [I, Q, I, Q...] f32 samples
    }
    Ok(())
}
```

## 🛠 Running the Example

Once your hardware is plugged in, try the included monitor example:
```bash
RUST_LOG=info cargo run --example monitor
```

## 🗺 Roadmap

- [x] **Phase 1: Hardware Bridge** (USB Vendor Requests, I2C Bridge)
- [x] **Phase 2: Modular Tuner Support** (R828D / V4 Triplexer and Interleaved Gain)
- [x] **Phase 3: Async Pipe** (Tokio Stream with error propagation and shutdown)
- [x] **Phase 4: DSP Integrated Decimation** (NEON SIMD Low-Pass Filters)
- [x] **Phase 5: Soft Device Sharing** (Unix Domain Socket Server with CancellationToken)
- [ ] **Phase 6: Legacy Hardware** (Support for older hardware)
- [ ] **Phase 7: Cross Platform** (More focus on other environments)

## 📝 Notes


## 📜 License

Licensed under the Apache License, Version 2.0 (the "License").
