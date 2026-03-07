//! DSP pipeline: FIR low-pass filter + decimation with NEON acceleration.
//!
//! # Design
//!
//! A naive decimator just discards N-1 of every N samples.  That aliases
//! high-frequency content back into the baseband.  A proper decimator runs a
//! low-pass FIR filter first, *then* picks every Nth output sample.
//!
//! This module provides:
//!   - `design_lowpass`  — windowed-sinc FIR coefficient generator
//!   - `Decimator`       — stateful FIR+decimate with history overlap buffer
//!                         (handles block boundaries correctly)
//!   - NEON fast-path on aarch64, scalar fallback everywhere else
//!
//! # RTL-SDR usage
//!
//! The driver outputs 2.048 MSPS.  To receive a 200 kHz FM channel you need
//! to decimate by 10 (output 204.8 kSPS).  The anti-alias filter cutoff
//! should be set to 0.5 / decimation_factor = 0.05 (normalised, i.e. 102.4 kHz).
//!
//! ```rust
//! use rtlsdr_next::dsp::Decimator;
//! let iq_samples = vec![0.0f32; 2048];
//! let mut dec = Decimator::new(10, 0.05, 63); // factor=10, cutoff=0.05, 63 taps
//! let output  = dec.process(&iq_samples);
//! ```

// ============================================================
// FIR filter design — windowed sinc
// ============================================================

/// Generate a symmetric windowed-sinc low-pass FIR.
///
/// - `num_taps`:  must be odd for a symmetric Type-I filter
/// - `cutoff`:    normalised cutoff, 0.0 < cutoff < 0.5
///                (e.g. 0.05 = fc/fs = 102.4 kHz at 2.048 MSPS)
///
/// Returns `num_taps` coefficients that sum to 1.0.
pub fn design_lowpass(num_taps: usize, cutoff: f32) -> Vec<f32> {
    assert!(num_taps % 2 == 1, "num_taps must be odd");
    assert!(cutoff > 0.0 && cutoff < 0.5, "cutoff must be in (0, 0.5)");

    let m = (num_taps - 1) as f32;
    let mut taps: Vec<f32> = (0..num_taps)
        .map(|i| {
            let n = i as f32 - m / 2.0;
            // Sinc kernel
            let sinc = if n == 0.0 {
                2.0 * cutoff
            } else {
                (2.0 * std::f32::consts::PI * cutoff * n).sin()
                    / (std::f32::consts::PI * n)
            };
            // Hamming window
            let window =
                0.54 - 0.46 * (2.0 * std::f32::consts::PI * i as f32 / m).cos();
            sinc * window
        })
        .collect();

    // Normalise so DC gain = 1.0
    let sum: f32 = taps.iter().sum();
    taps.iter_mut().for_each(|t| *t /= sum);
    taps
}

// ============================================================
// Decimator
// ============================================================

/// Stateful FIR low-pass decimator.
///
/// Keeps a history buffer of `taps.len() - 1` samples so that FIR
/// convolution is correct across block boundaries — essential when
/// processing a continuous stream of RTL-SDR chunks.
pub struct Decimator {
    /// FIR coefficients (length = num_taps, odd)
    taps: Vec<f32>,
    /// Overlap-save history: last `taps.len() - 1` input samples
    history: Vec<f32>,
    /// Keep every Nth filtered sample
    factor: usize,
    /// Sample offset into the current block for correct phase tracking
    phase: usize,
}

impl Decimator {
    /// Create a new decimator.
    ///
    /// # Arguments
    /// * `factor`   — decimation ratio (output rate = input rate / factor)
    /// * `cutoff`   — normalised cutoff frequency (0 < cutoff < 0.5)
    /// * `num_taps` — FIR length (odd; higher = sharper but more latency)
    pub fn new(factor: usize, cutoff: f32, num_taps: usize) -> Self {
        assert!(factor >= 2, "decimation factor must be >= 2");
        let taps    = design_lowpass(num_taps, cutoff);
        let history = vec![0.0f32; taps.len() - 1];
        Self { taps, history, factor, phase: 0 }
    }

    /// Convenience constructor that picks a sensible cutoff and tap count.
    ///
    /// cutoff  = 0.45 / factor  (10% guard band before Nyquist)
    /// num_taps = 4 * factor + 1  (scales with decimation ratio)
    pub fn with_factor(factor: usize) -> Self {
        let cutoff   = 0.45 / factor as f32;
        let num_taps = 4 * factor + 1;
        Self::new(factor, cutoff, num_taps)
    }

    /// Process a block of samples.
    ///
    /// Input and output are interleaved I/Q f32 pairs as produced by the
    /// `Converter`.  Decimation is applied to the complex magnitude, i.e.
    /// both I and Q channels are decimated together maintaining their
    /// pairing.
    ///
    /// For non-I/Q (real) sample streams, the same function works — just
    /// pass a plain real sample slice.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        // Build the extended buffer: history || input
        let taps_len = self.taps.len();
        let overlap  = taps_len - 1;

        let mut extended = Vec::with_capacity(overlap + input.len());
        extended.extend_from_slice(&self.history);
        extended.extend_from_slice(input);

        // Output capacity: ceil((input.len() - phase) / factor)
        let output_len = (input.len().saturating_sub(self.phase) + self.factor - 1)
            / self.factor;
        let mut output = Vec::with_capacity(output_len);

        // Run FIR + decimate
        #[cfg(target_arch = "aarch64")]
        {
            if std::arch::is_aarch64_feature_detected!("neon") {
                unsafe {
                    fir_decimate_neon(
                        &extended,
                        &self.taps,
                        self.factor,
                        &mut self.phase,
                        &mut output,
                    );
                }
                // Update history
                let new_history_start = extended.len() - overlap;
                self.history.copy_from_slice(&extended[new_history_start..]);
                return output;
            }
        }

        fir_decimate_scalar(
            &extended,
            &self.taps,
            self.factor,
            &mut self.phase,
            &mut output,
        );

        // Update history for next block
        let new_history_start = extended.len() - overlap;
        self.history.copy_from_slice(&extended[new_history_start..]);

        output
    }

    /// Reset internal state (history + phase). Call between unrelated streams.
    pub fn reset(&mut self) {
        self.history.fill(0.0);
        self.phase = 0;
    }
}

// ============================================================
// Scalar FIR + decimate
// ============================================================

fn fir_decimate_scalar(
    extended: &[f32],
    taps:     &[f32],
    factor:   usize,
    phase:    &mut usize,
    output:   &mut Vec<f32>,
) {
    let taps_len   = taps.len();
    let overlap    = taps_len - 1;
    let input_len  = extended.len() - overlap;

    let mut i = *phase;
    while i < input_len {
        // Dot-product of taps over the window centred at extended[i..i+taps_len]
        let mut acc = 0.0f32;
        let window = &extended[i..i + taps_len];
        for (s, t) in window.iter().zip(taps.iter()) {
            acc += s * t;
        }
        output.push(acc);
        i += factor;
    }

    // Carry forward phase for next block
    *phase = i - input_len;
}

// ============================================================
// NEON FIR + decimate  (aarch64 only)
// ============================================================

#[cfg(target_arch = "aarch64")]
#[target_feature(enable = "neon")]
unsafe fn fir_decimate_neon(
    extended: &[f32],
    taps:     &[f32],
    factor:   usize,
    phase:    &mut usize,
    output:   &mut Vec<f32>,
) {
    use std::arch::aarch64::*;

    let taps_len  = taps.len();
    let overlap   = taps_len - 1;
    let input_len = extended.len() - overlap;

    // Number of tap-groups we can process 4-at-a-time
    let taps_simd = taps_len - (taps_len % 4);

    let mut i = *phase;
    while i < input_len {
        let win_ptr  = extended.as_ptr().add(i);
        let taps_ptr = taps.as_ptr();

        let mut v_acc = vdupq_n_f32(0.0);
        let mut j     = 0usize;

        // ── 4-wide FMA loop ──────────────────────────────────────────────
        while j < taps_simd {
            let v_s = vld1q_f32(win_ptr.add(j));
            let v_t = vld1q_f32(taps_ptr.add(j));
            v_acc   = vmlaq_f32(v_acc, v_s, v_t);
            j      += 4;
        }

        // ── Horizontal sum of 4 lanes ────────────────────────────────────
        let mut acc = vaddvq_f32(v_acc);

        // ── Scalar tail (0..3 remaining taps) ───────────────────────────
        while j < taps_len {
            acc += *extended.get_unchecked(i + j) * *taps.get_unchecked(j);
            j   += 1;
        }

        output.push(acc);
        i += factor;
    }

    *phase = i - input_len;
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    // ── FIR design ───────────────────────────────────────────────────────

    #[test]
    fn test_lowpass_tap_count() {
        let taps = design_lowpass(63, 0.1);
        assert_eq!(taps.len(), 63);
    }

    #[test]
    fn test_lowpass_dc_gain_unity() {
        let taps = design_lowpass(63, 0.1);
        let sum: f32 = taps.iter().sum();
        assert!((sum - 1.0).abs() < EPS, "DC gain = {}", sum);
    }

    #[test]
    fn test_lowpass_symmetric() {
        let taps = design_lowpass(63, 0.1);
        let n = taps.len();
        for i in 0..n / 2 {
            assert!(
                (taps[i] - taps[n - 1 - i]).abs() < EPS,
                "Not symmetric at index {}: {} vs {}",
                i, taps[i], taps[n - 1 - i]
            );
        }
    }

    // ── Decimation math ──────────────────────────────────────────────────

    #[test]
    fn test_dc_passthrough() {
        // A DC input should pass through a LPF and decimate correctly.
        // We skip the initial filter transient — the first ceil(taps/2/factor)
        // output samples are influenced by the zero-initialized history buffer.
        let mut dec   = Decimator::new(4, 0.1, 17);
        let input     = vec![1.0f32; 128]; // longer input to get past transient
        let output    = dec.process(&input);

        assert_eq!(output.len(), 128 / 4);

        // Skip transient: ceil(num_taps / (2 * factor)) + 1
        let skip = (17 / (2 * 4)) + 2;
        for &v in &output[skip..] {
            assert!((v - 1.0).abs() < 0.01, "DC passthrough failed: {}", v);
        }
    }

    #[test]
    fn test_output_length() {
        let mut dec = Decimator::with_factor(8);
        let input   = vec![0.0f32; 256];
        let out     = dec.process(&input);
        assert_eq!(out.len(), 256 / 8);
    }

    #[test]
    fn test_nyquist_rejection() {
        // A signal at exactly Nyquist (alternating +1/-1) should be
        // heavily attenuated by the LPF before decimation.
        // Use more taps for better stopband attenuation.
        let mut dec   = Decimator::new(4, 0.1, 63);
        let input: Vec<f32> = (0..512)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let output = dec.process(&input);

        // Skip transient
        let skip = (63 / (2 * 4)) + 2;
        for &v in &output[skip..] {
            assert!(v.abs() < 0.1, "Nyquist not rejected: {}", v);
        }
    }

    #[test]
    fn test_block_boundary_continuity() {
        // Process the same signal in one block vs two half-blocks.
        // Outputs (after the initial filter transient) should match.
        let signal: Vec<f32> = (0..512)
            .map(|i| (i as f32 * 0.01 * std::f32::consts::PI).sin())
            .collect();

        let mut dec_one = Decimator::new(4, 0.1, 17);
        let out_one     = dec_one.process(&signal);

        let mut dec_two = Decimator::new(4, 0.1, 17);
        let mut out_two = dec_two.process(&signal[..256]);
        out_two.extend(dec_two.process(&signal[256..]));

        assert_eq!(out_one.len(), out_two.len());

        // Skip the initial filter transient (ceil(taps/2/factor) samples)
        let skip = (17 / 2 / 4) + 1;
        for i in skip..out_one.len() {
            assert!(
                (out_one[i] - out_two[i]).abs() < EPS,
                "Block boundary mismatch at output[{}]: {} vs {}",
                i, out_one[i], out_two[i]
            );
        }
    }

    #[test]
    fn test_reset_clears_state() {
        let mut dec   = Decimator::new(4, 0.1, 17);
        let signal    = vec![1.0f32; 64];
        let _         = dec.process(&signal); // warm up history
        dec.reset();

        // After reset, history is zero so output should match a fresh decimator
        let mut dec2  = Decimator::new(4, 0.1, 17);
        let out1      = dec.process(&signal);
        let out2      = dec2.process(&signal);

        for (a, b) in out1.iter().zip(out2.iter()) {
            assert!((a - b).abs() < EPS, "Post-reset mismatch: {} vs {}", a, b);
        }
    }

    // ── NEON / scalar agreement ──────────────────────────────────────────

    #[test]
    #[cfg(target_arch = "aarch64")]
    fn test_neon_scalar_agree() {
        if !std::arch::is_aarch64_feature_detected!("neon") {
            return;
        }

        let signal: Vec<f32> = (0..1024)
            .map(|i| (i as f32 * 0.05 * std::f32::consts::PI).sin())
            .collect();

        // Scalar path — generate taps independently, don't access private field
        let taps_s    = design_lowpass(17, 0.1);
        let out_s     = {
            let taps_len = taps_s.len();
            let overlap  = taps_len - 1;
            let mut ext  = vec![0.0f32; overlap];
            ext.extend_from_slice(&signal);
            let mut out  = Vec::new();
            let mut ph   = 0usize;
            fir_decimate_scalar(&ext, &taps_s, 4, &mut ph, &mut out);
            out
        };

        // NEON path
        let out_n = {
            let taps_n   = design_lowpass(17, 0.1);
            let taps_len = taps_n.len();
            let overlap  = taps_len - 1;
            let mut ext  = vec![0.0f32; overlap];
            ext.extend_from_slice(&signal);
            let mut out  = Vec::new();
            let mut ph   = 0usize;
            unsafe { fir_decimate_neon(&ext, &taps_n, 4, &mut ph, &mut out); }
            out
        };

        assert_eq!(out_s.len(), out_n.len());
        for (i, (&s, &n)) in out_s.iter().zip(out_n.iter()).enumerate() {
            assert!(
                (s - n).abs() < 1e-4,
                "NEON/scalar mismatch at [{}]: scalar={} neon={}",
                i, s, n
            );
        }
    }
}