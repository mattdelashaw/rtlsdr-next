# rtlsdr-next 📡
❗*Not hardware tested - bare with me here(README) too*❗

A high-performance, asynchronous, and safety-first Rust driver for RTL2832U-based Software Defined Radios (SDR). 

Designed for the modern era (2026+), this driver moves away from the legacy C callback model toward a **Tokio-native Stream** architecture, with specific optimizations for high-bandwidth ARM hosts like the **Raspberry Pi 5**.

## 🚀 Key Features

*   **Async-First Architecture:** Built on `Tokio`. SDR data is a standard `Stream` with backpressure and graceful shutdown.
*   **Zero-Allocation Pipeline:** Uses a custom `PooledVec` system to eliminate memory allocations in the hot loop. The CPU cycles are spent on DSP, not memory management.
*   **NEON SIMD Acceleration:** optimized ARM NEON intrinsics for `u8` -> `f32` conversion and FIR filtering.
*   **Automatic Tuner Probing:** Performs an I2C handshake to identify Rafael Micro (R820T/R828D), Elonics (E4000), or Fitipower (FC0012/13) chips automatically.
*   **Zero-Copy Broadcasting:** Efficiently share a single hardware device across multiple local apps using `Arc`-based broadcasting over Unix Domain Sockets.
*   **Precision Frequency Correction:** Integrated PPM correction for both the tuner PLL and the RTL2832U resampler.

## 🚀 Performance

Benchmarked on an ARM64 host (Pi 5 equivalent):
*   **Converter (`u8` to `f32`):** ~1.14 GiB/s throughput.
*   **Decimator (FIR Filter):** ~136 Million samples/sec.
*   **Efficiency:** CPU usage is effectively negligible for standard 2.4 MSPS radio streams.

## 🏗 Quick Start
... (keep existing code) ...

## 📊 Benchmarking

The project includes a professional Criterion benchmark suite to verify DSP performance:
```bash
cargo bench --bench dsp_bench
```

## 🛠 Running the Example
... (keep existing commands) ...

## 🗺 Roadmap - Phase Shifting

- [x] **Phase 1: Hardware Bridge** (USB Vendor Requests, I2C Bridge)
- [x] **Phase 2: Modular Tuner Support** (R828D / V4 Triplexer and Interleaved Gain)
- [x] **Phase 3: Async Pipe** (Tokio Stream with error propagation and shutdown)
- [x] **Phase 4: DSP Integrated Decimation** (NEON SIMD Low-Pass Filters)
- [x] **Phase 5: Soft Device Sharing** (Unix Domain Socket Server with CancellationToken)
- [x] **Phase 6: Automatic Probing** (Handshake-based hardware detection)
- [x] **Phase 7: Zero-Allocation Optimization** (Buffer Pooling and in-place processing)
- [x] **Phase 8: DMA-Pool Bridge** (Linking PooledVec directly to libusb DMA-pinned memory)
- [ ] **Phase 9: Legacy Implementation** (Full register maps for E4000/FC0012)
- [ ] **Phase 10: Cross-Platform** (Ensure the `aarch64` benefits to are passed to everyone)
- [ ] **Phase 11: Does This End?** (Probably turn this list into more of a TODO. Def need configurables) 

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
