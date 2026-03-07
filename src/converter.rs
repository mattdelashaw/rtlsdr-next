/// Converts interleaved RTL-SDR u8 samples (offset binary, I,Q,I,Q...)
/// into interleaved f32 complex samples centered at 0.0.
///
/// RTL-SDR offset binary: 0 = -1.0, 127 = ~0.0, 255 = +1.0
/// Formula: f = (u8 - 127.0) / 128.0
///
/// Two implementations:
///   - ScalarConverter: portable, used on x86/macOS and as fallback
///   - NeonConverter:   aarch64 SIMD, processes 16 samples/cycle on Pi 5
///
/// The public `convert` function dispatches at runtime on aarch64,
/// falling back to scalar everywhere else.
pub trait Converter: Send + Sync {
    fn convert(&self, src: &[u8], dst: &mut [f32]);
}

// ============================================================
// Scalar implementation (all platforms)
// ============================================================

pub struct ScalarConverter;

impl Converter for ScalarConverter {
    #[inline]
    fn convert(&self, src: &[u8], dst: &mut [f32]) {
        debug_assert_eq!(src.len(), dst.len(), "src and dst must be same length");
        for (d, &s) in dst.iter_mut().zip(src.iter()) {
            *d = (s as f32 - 127.0) / 128.0;
        }
    }
}

// ============================================================
// NEON implementation (aarch64 only)
// ============================================================
//
// The u8 → f32 widening chain:
//
//   vld1q_u8      load 16 x u8  into uint8x16_t
//   vmovl_u8      widen low  8 x u8  → 8 x u16  (uint16x8_t)  [vget_low_u8]
//   vmovl_high_u8 widen high 8 x u8  → 8 x u16  (uint16x8_t)
//   vmovl_u16     widen low  4 x u16 → 4 x u32  (uint32x4_t)
//   vmovl_high_u16 "  high 4 x u16 → 4 x u32
//   vcvtq_f32_u32 convert   4 x u32 → 4 x f32
//   vsubq_f32     subtract offset (127.0) from 4 x f32
//   vmulq_f32     scale by 1/128.0
//   vst1q_f32     store 4 x f32
//
// One vld1q_u8 gives 16 u8 samples.
// We need 4 vst1q_f32 passes (4 samples each) to drain all 16.
// So one outer iteration converts 16 input bytes → 16 output f32s.

#[cfg(target_arch = "aarch64")]
pub struct NeonConverter;

#[cfg(target_arch = "aarch64")]
impl Converter for NeonConverter {
    #[inline]
    fn convert(&self, src: &[u8], dst: &mut [f32]) {
        debug_assert_eq!(src.len(), dst.len(), "src and dst must be same length");
        // SAFETY: all pointer arithmetic stays within the proven-equal slice bounds.
        unsafe { neon_convert(src, dst) }
    }
}

#[cfg(target_arch = "aarch64")]
#[target_feature(enable = "neon")]
unsafe fn neon_convert(src: &[u8], dst: &mut [f32]) {
    use std::arch::aarch64::*;

    let len = src.len();
    let simd_len = len - (len % 16); // round down to multiple of 16

    // Broadcast constants into NEON registers once
    let v_offset = vdupq_n_f32(127.0_f32);
    let v_scale  = vdupq_n_f32(1.0_f32 / 128.0_f32);

    let mut i = 0usize;

    while i < simd_len {
        // ── Load 16 x u8 ────────────────────────────────────────────────
        let v_u8: uint8x16_t = vld1q_u8(src.as_ptr().add(i));

        // ── Widen to u16 ────────────────────────────────────────────────
        // low 8 bytes → uint16x8_t
        let v_u16_lo: uint16x8_t = vmovl_u8(vget_low_u8(v_u8));
        // high 8 bytes → uint16x8_t
        let v_u16_hi: uint16x8_t = vmovl_high_u8(v_u8);

        // ── Widen u16 → u32 (4 groups of 4) ─────────────────────────────
        let v_u32_0: uint32x4_t = vmovl_u16(vget_low_u16(v_u16_lo));
        let v_u32_1: uint32x4_t = vmovl_high_u16(v_u16_lo);
        let v_u32_2: uint32x4_t = vmovl_u16(vget_low_u16(v_u16_hi));
        let v_u32_3: uint32x4_t = vmovl_high_u16(v_u16_hi);

        // ── Convert u32 → f32 ────────────────────────────────────────────
        let v_f32_0: float32x4_t = vcvtq_f32_u32(v_u32_0);
        let v_f32_1: float32x4_t = vcvtq_f32_u32(v_u32_1);
        let v_f32_2: float32x4_t = vcvtq_f32_u32(v_u32_2);
        let v_f32_3: float32x4_t = vcvtq_f32_u32(v_u32_3);

        // ── Subtract offset (127.0) ──────────────────────────────────────
        let v_f32_0 = vsubq_f32(v_f32_0, v_offset);
        let v_f32_1 = vsubq_f32(v_f32_1, v_offset);
        let v_f32_2 = vsubq_f32(v_f32_2, v_offset);
        let v_f32_3 = vsubq_f32(v_f32_3, v_offset);

        // ── Scale by 1/128.0 ────────────────────────────────────────────
        let v_f32_0 = vmulq_f32(v_f32_0, v_scale);
        let v_f32_1 = vmulq_f32(v_f32_1, v_scale);
        let v_f32_2 = vmulq_f32(v_f32_2, v_scale);
        let v_f32_3 = vmulq_f32(v_f32_3, v_scale);

        // ── Store 16 x f32 ──────────────────────────────────────────────
        let out = dst.as_mut_ptr().add(i);
        vst1q_f32(out,      v_f32_0);
        vst1q_f32(out.add(4),  v_f32_1);
        vst1q_f32(out.add(8),  v_f32_2);
        vst1q_f32(out.add(12), v_f32_3);

        i += 16;
    }

    // ── Scalar tail (0..15 remaining samples) ────────────────────────────
    for j in i..len {
        dst[j] = (src[j] as f32 - 127.0) / 128.0;
    }
}

// ============================================================
// Runtime-dispatched public convenience function
// ============================================================

/// Convert RTL-SDR u8 offset-binary samples to f32.
/// Automatically uses NEON on aarch64, scalar everywhere else.
#[inline]
pub fn convert(src: &[u8], dst: &mut [f32]) {
    debug_assert_eq!(src.len(), dst.len());

    #[cfg(target_arch = "aarch64")]
    {
        // is_aarch64_feature_detected! checks the CPU at runtime.
        // On the Pi 5 (Cortex-A76) this is always true but the check
        // costs nothing after the first call — it reads a cached atomic.
        if std::arch::is_aarch64_feature_detected!("neon") {
            let c = NeonConverter;
            return c.convert(src, dst);
        }
    }

    ScalarConverter.convert(src, dst);
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Tolerance for f32 comparisons
    const EPS: f32 = 1e-6;

    fn check(input: &[u8], expected: &[f32]) {
        let scalar = ScalarConverter;
        let mut out_scalar = vec![0.0f32; input.len()];
        scalar.convert(input, &mut out_scalar);
        for (i, (&a, &e)) in out_scalar.iter().zip(expected.iter()).enumerate() {
            assert!((a - e).abs() < EPS, "scalar[{}]: got {}, expected {}", i, a, e);
        }

        #[cfg(target_arch = "aarch64")]
        if std::arch::is_aarch64_feature_detected!("neon") {
            let neon = NeonConverter;
            let mut out_neon = vec![0.0f32; input.len()];
            neon.convert(input, &mut out_neon);
            for (i, (&a, &e)) in out_neon.iter().zip(expected.iter()).enumerate() {
                assert!((a - e).abs() < EPS, "neon[{}]: got {}, expected {}", i, a, e);
            }
        }
    }

    #[test]
    fn test_center_value() {
        // 127 → (127-127)/128 = 0.0
        check(&[127], &[0.0]);
    }

    #[test]
    fn test_max_value() {
        // 255 → (255-127)/128 = 1.0
        check(&[255], &[1.0]);
    }

    #[test]
    fn test_min_value() {
        // 0 → (0-127)/128 = -0.9921875
        check(&[0], &[-127.0 / 128.0]);
    }

    #[test]
    fn test_batch_boundary_values() {
        check(
            &[127, 255, 0, 64],
            &[0.0, 1.0, -127.0 / 128.0, (64.0 - 127.0) / 128.0],
        );
    }

    #[test]
    fn test_16_sample_simd_batch() {
        // Exactly 16 samples — exercises the full NEON path with no tail
        let input: Vec<u8> = (0u8..16).map(|i| 127u8.wrapping_add(i * 8)).collect();
        let expected: Vec<f32> = input.iter().map(|&v| (v as f32 - 127.0) / 128.0).collect();
        check(&input, &expected);
    }

    #[test]
    fn test_17_sample_simd_plus_tail() {
        // 17 samples — 16 via NEON + 1 scalar tail
        let input: Vec<u8> = (0u8..17).map(|i| i.wrapping_mul(15)).collect();
        let expected: Vec<f32> = input.iter().map(|&v| (v as f32 - 127.0) / 128.0).collect();
        check(&input, &expected);
    }

    #[test]
    fn test_large_buffer_neon_scalar_agree() {
        // 256KB buffer — realistic RTL-SDR chunk size
        // Verifies NEON and scalar produce identical results
        let len = 256 * 1024;
        let input: Vec<u8> = (0..len).map(|i| (i % 256) as u8).collect();

        let scalar = ScalarConverter;
        let mut out_scalar = vec![0.0f32; len];
        scalar.convert(&input, &mut out_scalar);

        #[cfg(target_arch = "aarch64")]
        if std::arch::is_aarch64_feature_detected!("neon") {
            let neon = NeonConverter;
            let mut out_neon = vec![0.0f32; len];
            neon.convert(&input, &mut out_neon);
            for (i, (&a, &b)) in out_scalar.iter().zip(out_neon.iter()).enumerate() {
                assert!(
                    (a - b).abs() < EPS,
                    "Mismatch at index {}: scalar={} neon={}",
                    i, a, b
                );
            }
        }
    }

    #[test]
    fn test_dispatch_function() {
        let input = vec![127u8, 255, 0, 200, 50, 127, 100, 180];
        let expected: Vec<f32> = input.iter().map(|&v| (v as f32 - 127.0) / 128.0).collect();
        let mut output = vec![0.0f32; input.len()];
        convert(&input, &mut output);
        for (i, (&a, &e)) in output.iter().zip(expected.iter()).enumerate() {
            assert!((a - e).abs() < EPS, "dispatch[{}]: got {}, expected {}", i, a, e);
        }
    }
}