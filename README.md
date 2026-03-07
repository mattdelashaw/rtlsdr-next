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
- [ ] **Phase 8: Driver Driver** (Drivers for the drivers)
- [ ] **Phase 9: Does This End?** (Probably turn this list into more of a TODO. Def need configurables)

## 📝 Notes

1. **Rafael Micro R820T / R820T2 / R828D**
   These are the most common tuners. The R828D (used in the V4) is a specialized version with multiple inputs, but the core registers are nearly identical to the R820T.
   * [R820T Datasheet (PDF)](https://github.com/josebury/R820T-Datasheet/blob/master/R820T_Datasheet-Non_Disclosure_Working_Draft.pdf): This is a leaked "Non-Disclosure" draft, but it's the gold standard for understanding the registers.
   * [RTL-SDR Blog V4 Guide](https://www.rtl-sdr.com/V4/): Essential reading for the R828D specifically.

2. **Elonics E4000**
   This was the "original" high-end tuner for the RTL2832U before the company went out of business. It has a massive frequency range (up to 2.2 GHz).
   * [Osmocom E4000 Wiki](https://osmocom.org/projects/rtl-sdr/wiki/E4000): The primary source for register maps and the history of this tuner.
   * [E4000 Driver Source (C)](https://github.com/osmocom/rtl-sdr/blob/master/src/tuner_e4k.c): In the world of SDR, the source code is often the best documentation. This file shows exactly how to initialize it.

3. **Fitipower FC0012 / FC0013**
   These are cheaper, simpler tuners often found in small "nano" dongles.
   * [FC0012/13 Driver Source](https://github.com/osmocom/rtl-sdr/blob/master/src/tuner_fc0012.c): Documentation is nearly non-existent for these, so the Osmocom C driver is the only reliable reference for how the I2C registers work.

4. **General RTL-SDR Reverse Engineering**
   * [The RTL-SDR Wiki (Osmocom)](https://osmocom.org/projects/rtl-sdr/wiki/Rtl-sdr): This is the "Bible" of RTL-SDR. It explains the RTL2832U chip, the different tuners, and the I2C bridge logic.
   * [RTL2832U Register Map (Community Spreadsheets)](https://github.com/steve-m/librtlsdr/blob/master/src/librtlsdr.c): The comments in this C file are essentially the manual for the main RTL2832U chip.


## 📜 License

Licensed under the Apache License, Version 2.0 (the "License").
