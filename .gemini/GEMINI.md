# rtlsdr-next: Project Context & Architecture

This project is a high-performance, async-native Rust driver for RTL2832U-based SDRs, with first-class support for the **RTL-SDR Blog V4**.

## 🏗 Architecture Overview

- **`src/device.rs`**: Core USB communication via `rusb`. Implements the `HardwareInterface` trait.
  - *Critical:* Every `demod_write_reg` must be followed by a dummy read of `page 0x0a, reg 0x01` to sync the hardware.
  - *Critical:* I2C writes must be chunked to 7 data bytes (+1 register byte) to avoid RTL2832U buffer overflows.
- **`src/lib.rs`**: The `Driver` orchestrator.
  - Handles V4-specific GPIO power-up (GPIO 4 & 5).
  - Manages the V4 HF upconverter (28.8 MHz offset) and spectral inversion.
- **`src/tuners/`**: Chip-specific logic (currently R828D/R820T).
- **`src/dsp.rs`**: Performance-critical SIMD (NEON) code for filtering and conversion.
- **`src/stream.rs`**: Zero-allocation buffer pooling and Tokio-native `Stream` implementation.

## 📡 Hardware Specifics (RTL-SDR Blog V4)

- **Tuner:** R828D (I2C address `0x74`).
- **Mode:** Uses Low-IF (not Zero-IF). Requires specific demodulator register tweaks (`page1 reg 0xb1 = 0x1a`).
- **HF Path:** When tuning < 28.8 MHz, the driver adds a 28.8 MHz offset and sets the `spectral_inv` flag in the DDC sync register (`0x15`).

## 🛠 Related Workspace Context

- **`rtlsdr_sys`**: Low-level C bindings (reference only).
- **`rtl-sdr-rs`**: Pure Rust alternative (generic focus).
- **`rtl-sdr-blog`**: The original C source code (the "source of truth" for register maps).

## ✅ Verification Workflows

- **Driver Smoke Test:** `cargo run --release --example hw_probe` (Run after ANY hardware-touching change).
- **Performance/SIMD Check:** `cargo bench --bench dsp_bench` (Run after modifying `dsp.rs`).
- **V4 Logic Regression:** `cargo run --release --example diag_raw_clone` (Compares driver behavior to raw V4 init sequence).

## 🆘 Hardware Troubleshooting (The "Panic Room")

- **USB Pipe Error / Stalls:** If the device stops responding, run `cargo run --release --example diag_demod`. It attempts 5 re-acquisition pulses to reset the USB state machine without unplugging.
- **"Device Busy":** Usually means a previous process didn't drop the `rusb` handle. Kill any hanging `rtl_tcp` or `websdr` processes.

## 🚀 Development Guidelines

- **Performance:** Prioritize zero-allocation and SIMD in the "hot path" (streaming/DSP).
- **Safety:** Leverage `rusb` for safe USB handling but maintain "C-like" precision for hardware registers.
- **Async:** Everything should be compatible with the `Tokio` runtime.

## Gemini Added Memories
- Always propose an approach and ask for confirmation before implementing changes for Inquiries. Only act autonomously when given a direct command (e.g., 'hey go do X').

