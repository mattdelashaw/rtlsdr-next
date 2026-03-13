//! Benchmarks: rtlsdr-next vs librtlsdr baseline
//!
//! This benchmark compares the rtlsdr-next Rust implementation against the
//! literal C code used in the librtlsdr (RTL-SDR Blog V4 fork).
//!
//! Two scenarios are tested:
//!   1. Standard conversion (U8 -> F32)
//!   2. V4 HF conversion (U8 -> F32 with spectral inversion Q = -Q)
//!
//! The C code is compiled only if the `bench-c` feature is enabled.

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use rtlsdr_next::converter;
use rtlsdr_next::converter::Converter;
use rtlsdr_next::converter::ScalarConverter;
use rtlsdr_next::dsp::Decimator;

// ── Block size matches the driver's real transfer size ───────────────────────
const BLOCK_SIZE: usize = 256 * 1024; // 256 KB — one USB bulk transfer

// ============================================================
// FFI: Actual C code from the librtlsdr project
// ============================================================

#[cfg(feature = "bench-c")]
unsafe extern "C" {
    /// Standard LUT-based conversion from the C library.
    fn librtlsdr_v4_convert(src: *const u8, dst: *mut f32, len: libc::size_t);

    /// V4 bridge logic: LUT conversion followed by a second pass for inversion.
    fn librtlsdr_v4_bridge_convert_inverted(src: *const u8, dst: *mut f32, len: libc::size_t);
}

// ============================================================
// Converter benchmarks
// ============================================================

fn bench_converter(c: &mut Criterion) {
    let src = vec![127u8; BLOCK_SIZE];
    let mut dst = vec![0.0f32; BLOCK_SIZE];

    let mut group = c.benchmark_group("Converter / 256KB block");
    group.throughput(Throughput::Bytes(BLOCK_SIZE as u64));

    // ── 1. REAL C CODE (librtlsdr LUT) ──────────────────────────────────
    #[cfg(feature = "bench-c")]
    group.bench_function("librtlsdr (C LUT)", |b| {
        b.iter(|| unsafe {
            librtlsdr_v4_convert(src.as_ptr(), dst.as_mut_ptr(), BLOCK_SIZE);
            black_box(&dst);
        })
    });

    // ── 2. rtlsdr-next (Rust Scalar) ─────────────────────────────────────
    group.bench_function("rtlsdr-next (Rust Scalar)", |b| {
        b.iter(|| {
            ScalarConverter.convert(black_box(&src), black_box(&mut dst));
        })
    });

    group.finish();
}

// ============================================================
// V4 HF Inversion benchmarks
// ============================================================

fn bench_v4_hf_inversion(c: &mut Criterion) {
    let src = vec![127u8; BLOCK_SIZE];
    let mut dst = vec![0.0f32; BLOCK_SIZE];

    let mut group = c.benchmark_group("Converter / V4 HF HF (Inverted)");
    group.throughput(Throughput::Bytes(BLOCK_SIZE as u64));

    // ── 1. REAL C CODE BRIDGE (Two-pass: LUT then Sign-flip) ─────────────
    #[cfg(feature = "bench-c")]
    group.bench_function("librtlsdr Bridge (C 2-pass)", |b| {
        b.iter(|| unsafe {
            librtlsdr_v4_bridge_convert_inverted(src.as_ptr(), dst.as_mut_ptr(), BLOCK_SIZE);
            black_box(&dst);
        })
    });

    // ── 2. rtlsdr-next (Rust Single-pass) ────────────────────────────────
    group.bench_function("rtlsdr-next (Rust 1-pass)", |b| {
        b.iter(|| {
            ScalarConverter.convert_inverted(black_box(&src), black_box(&mut dst));
        })
    });

    group.finish();
}

// ============================================================
// FIR decimation
// ============================================================

fn bench_decimator(c: &mut Criterion) {
    let input = vec![0.5f32; BLOCK_SIZE];

    let mut group = c.benchmark_group("Decimation / 256KB block");
    group.throughput(Throughput::Elements(BLOCK_SIZE as u64));

    for factor in [4usize, 8, 16] {
        let mut dec = Decimator::with_factor(factor);
        group.bench_with_input(
            BenchmarkId::new("rtlsdr-next FIR", factor),
            &factor,
            |b, _| {
                b.iter(|| {
                    let _ = dec.process(black_box(&input));
                })
            },
        );
    }

    group.finish();
}

// ============================================================
// End-to-end pipeline: convert + decimate
// ============================================================

fn bench_pipeline(c: &mut Criterion) {
    let src_u8 = vec![127u8; BLOCK_SIZE];
    let mut f32_buf = vec![0.0f32; BLOCK_SIZE];

    let mut group = c.benchmark_group("Full pipeline / 256KB block");
    group.throughput(Throughput::Bytes(BLOCK_SIZE as u64));

    // rtlsdr-next: full convert + anti-alias FIR + decimate in one pass
    for factor in [4usize, 8, 16] {
        let mut dec = Decimator::with_factor(factor);
        group.bench_with_input(
            BenchmarkId::new("rtlsdr-next (convert + FIR ÷)", factor),
            &factor,
            |b, _| {
                b.iter(|| {
                    converter::convert(black_box(&src_u8), &mut f32_buf);
                    let _ = dec.process(black_box(&f32_buf));
                })
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_converter,
    bench_v4_hf_inversion,
    bench_decimator,
    bench_pipeline,
);
criterion_main!(benches);
