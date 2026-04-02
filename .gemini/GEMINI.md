# rtlsdr-next: Project Context & Architecture

## 🛑 Mandates & Protocol

1. **Inquiry vs. Directive:** 
   - For **Inquiries** (questions, bug reports without "fix it", vague requests), you MUST research and propose a detailed strategy first. **DO NOT** implement changes until you receive confirmation.
   - For **Directives** ("fix this", "implement X", "apply proposed plan"), you may proceed autonomously.
   - Default to "Inquiry" mode if there is any ambiguity.

2. **Hardware Safety:**
   - Every `demod_write_reg` must be followed by a dummy read of `page 0x0a, reg 0x01` to sync the hardware.
   - I2C writes must be chunked to 7 data bytes (+1 register byte) to avoid RTL2832U buffer overflows.

## 🏗 Architecture Overview

- **`src/device.rs`**: Core USB communication via `rusb`. Implements the `HardwareInterface` trait.
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

## 📝 Recent Architectural Updates (April 2026)

- **Audio Pipeline Enhancements:**
  - **Variable Filter Bandwidth:** Implemented `update_cutoff` in `Decimator` to allow real-time filter adjustment (1.8kHz - 6.0kHz) without clearing history.
  - **Audio AGC with Hang Time:** Added `AudioAgc` struct specifically for post-demodulation SSB/AM audio. Features a configurable "hang" period to prevent noise pumping during speech pauses.
  - **NFM Continuity:** Fixed a critical state-erasure bug where NFM decimators were recreated every frame; state now correctly persists across USB transfers.
  - **Mono DC Removal:** Added `process_mono` to `DcRemover` to correctly handle single-channel audio without stride errors.
- **Production & Resiliency:**
  - **Multi-Device Support:** Implemented `RTLSDR_DEVICE_INDEX` environment variable to allow selecting specific dongles in multi-device setups.
  - **WebSDR Idle Optimization:** Added an idle-path check in the WebSDR pipeline; DSP processing now throttles to ~0% CPU when no clients are connected.
  - **rtl_tcp Resiliency:** Wrapped the hardware stream in a restart loop to ensure the server remains operational after USB pipe errors or unexpected disconnects.
  - **Protocol Robustness:** Added range clamping to the WebSDR bandwidth command to prevent DSP panics on invalid client input.
- **SSB Phasing Fix:** Corrected time-alignment in `SsbDemodulator` by removing incorrect group-delay history; I and Q branches are now perfectly aligned.
