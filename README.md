# rtlsdr-next 📡

A high-performance, asynchronous, and safety-first Rust driver for RTL2832U-based Software Defined Radios (SDR). 

Designed for the modern era (2026+), this driver moves away from the legacy C callback model toward a **Tokio-native Stream** architecture, with specific optimizations for high-bandwidth ARM hosts like the **Raspberry Pi 5**.

## 🚀 Key Features

*   **Async-First Architecture:** Built on `Tokio`. Treat your SDR as a standard `Stream` of samples.
*   **Zero-Copy Transport:** Utilizes `libusb_dev_mem_alloc` for DMA-pinned memory on supported Linux hosts (Pi 5/RP1), bypassing expensive kernel-to-user copies.
*   **Modular Tuner Trait:** Clean abstraction for tuner chips. Fully supports the **RTL-SDR Blog V4** (R828D) with integrated triplexer band-switching.
*   **SIMD Accelerated DSP:** Built-in conversion from `u8` (offset binary) to `f32` (complex) using **ARM NEON** intrinsics.
*   **Cross-Platform:** Native support for Linux, Windows, and macOS. Graceful fallbacks for non-DMA environments.
*   **Safety-First:** 100% memory-safe Rust (outside of essential USB FFI). No more segfaults from global state or race conditions.

## 🏗 Project Architecture

### 1. The Async Pipe
Unlike legacy drivers that block threads with callbacks, `rtlsdr-next` uses a background bulk-transfer loop that pushes data into a lock-free channel.
```rust
let driver = Driver::new()?;
let mut stream = driver.stream();

while let Some(samples) = stream.next().await {
    // Process samples (Vec<u8> or converted f32)
}
```

### 2. Transport Layer
The driver detects the host capabilities at runtime:
*   **Linux (Pi 5/Modern):** Uses Pinned DMA buffers for zero-copy.
*   **Generic/Windows/macOS:** Falls back to standard heap-allocated asynchronous bulk transfers.

### 3. Modular Tuners
Support for the R828D (V4) is implemented as a separate module, handling the internal 28.8MHz upconversion and notch filters automatically based on the requested frequency.

## 🛠 Installation

Add this to your `Cargo.toml`:
```toml
[dependencies]
rtlsdr-next = { git = "https://github.com/mattdelashaw/rtlsdr-next" }
```

### System Dependencies
*   **Linux:** `libusb-1.0-devel`
*   **Windows:** RTL-SDR Blog V3/V4 drivers (via Zadig/WinUSB)
*   **macOS:** `brew install libusb`

## 🗺 Roadmap

- [x] **Phase 1: Hardware Bridge** (USB Vendor Requests, I2C Bridge, Pi 5 Reset Hacks)
- [x] **Phase 2: Modular Tuner Support** (R828D / V4 Triplexer logic)
- [x] **Phase 3: Async Pipe** (Tokio Stream, DMA Buffer Abstraction)
- [x] **Phase 4: DSP Integrated Decimation** (SIMD-based Low-Pass Filters)
- [x] **Phase 5: Soft Device Sharing** (Unix Domain Socket Server)

## 📜 License
Licensed under the Apache License, Version 2.0 (the "License"). You may obtain a copy of the License at [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0).
