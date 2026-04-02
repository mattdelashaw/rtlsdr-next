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
//!     (handles block boundaries correctly)
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
///   (e.g. 0.05 = fc/fs = 102.4 kHz at 2.048 MSPS)
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
                (2.0 * std::f32::consts::PI * cutoff * n).sin() / (std::f32::consts::PI * n)
            };
            // Hamming window
            let window = 0.54 - 0.46 * (2.0 * std::f32::consts::PI * i as f32 / m).cos();
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
    pub taps: Vec<f32>,
    /// Overlap-save history: last `taps.len() - 1` input samples
    pub history: Vec<f32>,
    /// Keep every Nth filtered sample
    pub factor: usize,
    /// Sample offset into the current block for correct phase tracking
    pub phase: usize,
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
        let taps = design_lowpass(num_taps, cutoff);
        let history = vec![0.0f32; taps.len() - 1];
        Self {
            taps,
            history,
            factor,
            phase: 0,
        }
    }

    /// Convenience constructor that picks a sensible cutoff and tap count.
    ///
    /// cutoff  = 0.8 / factor   (Widened to avoid clipping FM sidebands)
    /// num_taps = 12 * factor + 1 (Higher quality rejection)
    pub fn with_factor(factor: usize) -> Self {
        let cutoff = 0.8 / factor as f32;
        let num_taps = 12 * factor + 1;
        Self::new(factor, cutoff, num_taps)
    }

    /// Process a block of samples into a provided destination buffer.
    ///
    /// This avoids re-allocating a new Vec for every block.
    pub fn process_into(&mut self, input: &[f32], output: &mut Vec<f32>) {
        // Clear destination but keep capacity
        output.clear();

        // Build the extended buffer: history || input
        let taps_len = self.taps.len();
        let overlap = taps_len - 1;

        // Note: Using a pre-allocated extended buffer would be even faster,
        // but for now let's focus on the output allocation.
        let mut extended = Vec::with_capacity(overlap + input.len());
        extended.extend_from_slice(&self.history);
        extended.extend_from_slice(input);

        // Run FIR + decimate
        #[cfg(target_arch = "aarch64")]
        {
            if std::arch::is_aarch64_feature_detected!("neon") {
                unsafe {
                    fir_decimate_neon(&extended, &self.taps, self.factor, &mut self.phase, output);
                }
                // Update history
                let new_history_start = extended.len() - overlap;
                self.history.copy_from_slice(&extended[new_history_start..]);
                return;
            }
        }

        fir_decimate_scalar(&extended, &self.taps, self.factor, &mut self.phase, output);

        // Update history for next block
        let new_history_start = extended.len() - overlap;
        self.history.copy_from_slice(&extended[new_history_start..]);
    }

    /// Process a block of samples and return a new Vec.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let output_len = input.len().saturating_sub(self.phase).div_ceil(self.factor);
        let mut output = Vec::with_capacity(output_len);
        self.process_into(input, &mut output);
        output
    }

    /// Reset internal state (history + phase). Call between unrelated streams.
    pub fn reset(&mut self) {
        self.history.fill(0.0);
        self.phase = 0;
    }

    /// Update the FIR cutoff frequency without clearing history.
    pub fn update_cutoff(&mut self, cutoff: f32) {
        self.taps = design_lowpass(self.taps.len(), cutoff);
    }
}

// ============================================================
// Scalar FIR + decimate
// ============================================================

fn fir_decimate_scalar(
    extended: &[f32],
    taps: &[f32],
    factor: usize,
    phase: &mut usize,
    output: &mut Vec<f32>,
) {
    let taps_len = taps.len();
    let overlap = taps_len - 1;
    let input_len = extended.len() - overlap;

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
    taps: &[f32],
    factor: usize,
    phase: &mut usize,
    output: &mut Vec<f32>,
) {
    use std::arch::aarch64::*;

    let taps_len = taps.len();
    let overlap = taps_len - 1;
    let input_len = extended.len() - overlap;

    // Number of tap-groups we can process 4-at-a-time
    let taps_simd = taps_len - (taps_len % 4);

    let mut i = *phase;
    while i < input_len {
        // Bounds check to prevent out-of-bounds access
        if i + taps_len > extended.len() {
            break;
        }

        let win_ptr = unsafe { extended.as_ptr().add(i) };
        let taps_ptr = taps.as_ptr();

        let mut v_acc = vdupq_n_f32(0.0);
        let mut j = 0usize;

        // ── 4-wide FMA loop ──────────────────────────────────────────────
        while j < taps_simd {
            // Bounds check to prevent out-of-bounds access
            if i + j + 3 >= extended.len() || j + 3 >= taps.len() {
                break;
            }
            unsafe {
                let v_s = vld1q_f32(win_ptr.add(j));
                let v_t = vld1q_f32(taps_ptr.add(j));
                v_acc = vmlaq_f32(v_acc, v_s, v_t);
            }
            j += 4;
        }

        // ── Horizontal sum of 4 lanes ────────────────────────────────────
        let mut acc = vaddvq_f32(v_acc);

        // ── Scalar tail (0..3 remaining taps) ───────────────────────────
        while j < taps_len {
            // Safe access with bounds checking
            if i + j < extended.len() && j < taps.len() {
                acc += extended[i + j] * taps[j];
            }
            j += 1;
        }

        output.push(acc);
        i += factor;
    }

    *phase = i - input_len;
}

// ============================================================
// DC Removal
// ============================================================

/// A one-pole IIR high-pass filter for removing DC offset.
///
/// Keeps a running average of the signal and subtracts it.
pub struct DcRemover {
    avg_i: f32,
    avg_q: f32,
    alpha: f32,
}

impl DcRemover {
    pub fn new(alpha: f32) -> Self {
        Self {
            avg_i: 0.0,
            avg_q: 0.0,
            alpha,
        }
    }

    /// Process a block of interleaved I/Q samples.
    pub fn process(&mut self, data: &mut [f32]) {
        for i in (0..data.len()).step_by(2) {
            let i_val = data[i];
            let q_val = data[i + 1];

            self.avg_i = (1.0 - self.alpha) * self.avg_i + self.alpha * i_val;
            self.avg_q = (1.0 - self.alpha) * self.avg_q + self.alpha * q_val;

            data[i] = i_val - self.avg_i;
            data[i + 1] = q_val - self.avg_q;
        }
    }

    /// Process a block of mono samples.
    pub fn process_mono(&mut self, data: &mut [f32]) {
        for val in data.iter_mut() {
            self.avg_i = (1.0 - self.alpha) * self.avg_i + self.alpha * *val;
            *val -= self.avg_i;
        }
    }

    /// Process separate I and Q arrays (avoids interleave/deinterleave overhead).
    pub fn process_split(&mut self, i_buf: &mut [f32], q_buf: &mut [f32]) {
        let n = i_buf.len().min(q_buf.len());
        let a = self.alpha;
        let b = 1.0 - a;
        let mut si = self.avg_i;
        let mut sq = self.avg_q;
        for k in 0..n {
            si = a * i_buf[k] + b * si;
            i_buf[k] -= si;
            sq = a * q_buf[k] + b * sq;
            q_buf[k] -= sq;
        }
        self.avg_i = si;
        self.avg_q = sq;
    }
}

// ============================================================
// Automatic Gain Control (AGC)
// ============================================================

/// A simple feedback-loop AGC to maintain a target signal level.
pub struct Agc {
    gain: f32,
    target: f32,
    attack: f32,
    decay: f32,
}

impl Agc {
    pub fn new(target: f32, attack: f32, decay: f32) -> Self {
        Self {
            gain: 1.0,
            target,
            attack,
            decay,
        }
    }

    /// Process a block of interleaved I/Q samples.
    pub fn process(&mut self, data: &mut [f32]) {
        for i in (0..data.len()).step_by(2) {
            let i_val = data[i];
            let q_val = data[i + 1];

            let mag = (i_val * i_val + q_val * q_val).sqrt();
            let error = self.target / (mag + 1e-6);

            // Simple attack/decay logic
            let coeff = if error < self.gain {
                self.attack
            } else {
                self.decay
            };
            self.gain = (1.0 - coeff) * self.gain + coeff * error;

            data[i] *= self.gain;
            data[i + 1] *= self.gain;
        }
    }
}

/// A post-demodulation Audio AGC with "Hang Time" and Noise Floor Suppression.
///
/// Designed for SSB and AM where signal-burstiness causes standard AGC to "pump".
/// Holds gain steady for `hang_time_samples` before starting to decay.
/// If signal magnitude is below `min_magnitude`, gain remains frozen to avoid
/// amplifying the noise floor.
pub struct AudioAgc {
    gain: f32,
    target: f32,
    attack: f32,
    decay: f32,
    min_magnitude: f32,
    hang_time_samples: usize,
    hang_counter: usize,
}

impl AudioAgc {
    pub fn new(
        target: f32,
        attack: f32,
        decay: f32,
        hang_time_ms: f32,
        sample_rate: f32,
        min_magnitude: f32,
    ) -> Self {
        Self {
            gain: 1.0,
            target,
            attack,
            decay,
            min_magnitude,
            hang_time_samples: (hang_time_ms * sample_rate / 1000.0) as usize,
            hang_counter: 0,
        }
    }

    pub fn process(&mut self, data: &mut [f32]) {
        for val in data.iter_mut() {
            let mag = val.abs();

            if mag < self.min_magnitude {
                // Signal is likely dead air / noise floor. Freeze gain to avoid pumping.
                *val *= self.gain;
                *val = val.clamp(-1.0, 1.0);
                continue;
            }

            let error = self.target / (mag + 1e-6);

            if error < self.gain {
                // Attack: signal got louder, reduce gain immediately
                self.gain = (1.0 - self.attack) * self.gain + self.attack * error;
                self.hang_counter = self.hang_time_samples;
            } else {
                // Decay: signal got quieter
                if self.hang_counter > 0 {
                    self.hang_counter -= 1;
                } else {
                    self.gain = (1.0 - self.decay) * self.gain + self.decay * error;
                }
            }

            *val *= self.gain;
            // Clamp to avoid digital clipping
            *val = val.clamp(-1.0, 1.0);
        }
    }
}

// ============================================================
// FM Demodulation
// ============================================================

/// A quadrature FM demodulator.
///
/// Produces real samples representing the instantaneous frequency
/// (derivative of phase) of the complex input.
pub struct FmDemodulator {
    last_phase: f32,
    deemph_alpha: f32,
    deemph_state: f32,
}

impl FmDemodulator {
    pub fn new() -> Self {
        Self {
            last_phase: 0.0,
            deemph_alpha: 1.0, // Default: no filtering
            deemph_state: 0.0,
        }
    }

    /// Set de-emphasis time constant (e.g., 75e-6 for US, 50e-6 for EU)
    pub fn with_deemphasis(mut self, sample_rate: f32, tau: f32) -> Self {
        let dt = 1.0 / sample_rate;
        self.deemph_alpha = dt / (dt + tau);
        self
    }

    /// Process a block of interleaved I/Q samples.
    /// Returns a vector of real (f32) demodulated samples.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() / 2);
        for i in (0..input.len()).step_by(2) {
            let i_val = input[i];
            let q_val = input[i + 1];

            let phase = q_val.atan2(i_val);
            let mut diff = phase - self.last_phase;

            // Phase wrap-around (-PI to PI)
            if diff > std::f32::consts::PI {
                diff -= 2.0 * std::f32::consts::PI;
            } else if diff < -std::f32::consts::PI {
                diff += 2.0 * std::f32::consts::PI;
            }

            // Apply de-emphasis (simple IIR low-pass)
            self.deemph_state =
                (1.0 - self.deemph_alpha) * self.deemph_state + self.deemph_alpha * diff;

            output.push(self.deemph_state);
            self.last_phase = phase;
        }
        output
    }
}

impl Default for FmDemodulator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// SSB Demodulation (USB/LSB)
// ============================================================

/// A FIR Hilbert transformer.
///
/// Shifts the phase of a signal by exactly 90 degrees.
/// Used in the phasing method for SSB demodulation.
pub struct HilbertFilter {
    taps: Vec<f32>,
    history: Vec<f32>,
}

impl HilbertFilter {
    /// Create a new Hilbert filter with N taps (must be odd).
    pub fn new(num_taps: usize) -> Self {
        assert!(num_taps % 2 == 1, "Hilbert taps must be odd");
        let mut taps = vec![0.0f32; num_taps];
        let mid = num_taps / 2;

        for (i, tap) in taps.iter_mut().enumerate() {
            let n = i as i32 - mid as i32;
            if n % 2 != 0 {
                // Ideal Hilbert kernel: 2 / (pi * n)
                let val = 2.0 / (std::f32::consts::PI * n as f32);
                // Blackman-Harris window for high sideband rejection (>90dB)
                let a0 = 0.35875;
                let a1 = 0.48829;
                let a2 = 0.14128;
                let a3 = 0.01168;
                let w = a0
                    - a1 * (2.0 * std::f32::consts::PI * i as f32 / (num_taps - 1) as f32).cos()
                    + a2 * (4.0 * std::f32::consts::PI * i as f32 / (num_taps - 1) as f32).cos()
                    - a3 * (6.0 * std::f32::consts::PI * i as f32 / (num_taps - 1) as f32).cos();
                *tap = val * w;
            } else {
                *tap = 0.0;
            }
        }

        Self {
            taps,
            history: vec![0.0f32; num_taps - 1],
        }
    }

    /// Process input and return Hilbert-transformed (90° shifted) output.
    pub fn process_into(&mut self, input: &[f32], output: &mut Vec<f32>) {
        output.clear();
        let taps_len = self.taps.len();
        let overlap = taps_len - 1;

        let mut extended = Vec::with_capacity(overlap + input.len());
        extended.extend_from_slice(&self.history);
        extended.extend_from_slice(input);

        // Standard FIR convolution
        for i in 0..input.len() {
            let mut acc = 0.0f32;
            let window = &extended[i..i + taps_len];
            for (s, t) in window.iter().zip(self.taps.iter()) {
                acc += s * t;
            }
            output.push(acc);
        }

        let new_history_start = extended.len() - overlap;
        self.history.copy_from_slice(&extended[new_history_start..]);
    }
}

/// A Single Sideband (SSB) demodulator using the phasing method.
pub struct SsbDemodulator {
    hilbert: HilbertFilter,
    q_shifted: Vec<f32>,
    is_usb: bool,
}

impl SsbDemodulator {
    pub fn new(is_usb: bool) -> Self {
        let num_taps = 65; // Good balance for 48kHz audio
        Self {
            hilbert: HilbertFilter::new(num_taps),
            q_shifted: Vec::with_capacity(1024),
            is_usb,
        }
    }

    /// Process interleaved I/Q samples and return real audio.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let n = input.len() / 2;
        let mut i_branch = Vec::with_capacity(n);
        let mut q_branch = Vec::with_capacity(n);

        for k in 0..n {
            i_branch.push(input[k * 2]);
            q_branch.push(input[k * 2 + 1]);
        }

        // 1. Transform Q branch (90 deg shift)
        self.hilbert.process_into(&q_branch, &mut self.q_shifted);

        // 2. Combine with I branch
        // Hilbert filter's q_shifted[k] is the 90-degree shift of input[k].
        // They are already time-aligned at the same index.
        let mut output = Vec::with_capacity(n);
        for (&i_val, &q_hat) in i_branch.iter().zip(self.q_shifted.iter()) {
            // Phasing formula: USB = I - Q_hat, LSB = I + Q_hat
            if self.is_usb {
                output.push(i_val - q_hat);
            } else {
                output.push(i_val + q_hat);
            }
        }
        output
    }
}

// ============================================================

/// A magnitude-based AM demodulator.
pub struct AmDemodulator {
    dc_alpha: f32,
    dc_mean: f32,
    hp_alpha: f32,
    hp_state: f32,
}

impl AmDemodulator {
    pub fn new() -> Self {
        Self {
            dc_alpha: 0.005, // Slower EMA for base envelope
            dc_mean: 0.0,
            hp_alpha: 0.05, // Faster HP filter for audio centering
            hp_state: 0.0,
        }
    }

    /// Process a block of interleaved I/Q samples.
    /// Returns a vector of real (f32) demodulated samples.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() / 2);
        for i in (0..input.len()).step_by(2) {
            let i_val = input[i];
            let q_val = input[i + 1];

            // Magnitude
            let mag = (i_val * i_val + q_val * q_val).sqrt();

            // 1. EMA DC removal to extract audio envelope
            if self.dc_mean == 0.0 {
                self.dc_mean = mag;
            } else {
                self.dc_mean = (1.0 - self.dc_alpha) * self.dc_mean + self.dc_alpha * mag;
            }
            let envelope = mag - self.dc_mean;

            // 2. Second-stage HP filter to center audio perfectly at 0.0
            self.hp_state = (1.0 - self.hp_alpha) * self.hp_state + self.hp_alpha * envelope;
            output.push(envelope - self.hp_state);
        }
        output
    }
}

impl Default for AmDemodulator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    #[test]
    fn test_dc_remover() {
        let mut dc = DcRemover::new(0.01);
        // Input with a 0.5 offset on both I and Q
        let mut data = vec![1.5f32, 1.5, 1.5, 1.5, 1.5, 1.5];
        // Process many samples to let the average settle
        for _ in 0..1000 {
            let mut block = vec![1.5f32, 1.5];
            dc.process(&mut block);
        }
        dc.process(&mut data);
        // After settling, it should be near zero
        assert!(data[0].abs() < 0.1);
        assert!(data[1].abs() < 0.1);
    }

    #[test]
    fn test_dc_remover_mono() {
        let mut dc = DcRemover::new(0.01);
        let mut data = vec![1.5f32; 1000];
        dc.process_mono(&mut data);
        // Last sample should be near zero
        assert!(data[999].abs() < 0.1);
    }

    #[test]
    fn test_agc() {
        let mut agc = Agc::new(1.0, 0.01, 0.01);
        // Input with small magnitude (0.1)
        let mut data = vec![0.1f32, 0.0];
        // Process many samples to let the gain settle
        for _ in 0..1000 {
            let mut block = vec![0.1f32, 0.0];
            agc.process(&mut block);
        }
        agc.process(&mut data);
        // Output magnitude should be close to target (1.0)
        assert!((data[0].abs() - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_fm_demod() {
        let mut fm = FmDemodulator::new();
        // A complex sine wave with constant frequency
        // phase(t) = omega * t
        // freq = d(phase)/dt = omega
        let omega = 0.5f32;
        let mut input = Vec::new();
        for t in 0..100 {
            let phase = omega * t as f32;
            input.push(phase.cos());
            input.push(phase.sin());
        }
        let output = fm.process(&input);
        assert_eq!(output.len(), 100);
        // Skip first sample (no last_phase)
        for &v in &output[1..] {
            assert!((v - omega).abs() < 1e-4);
        }
    }

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
                i,
                taps[i],
                taps[n - 1 - i]
            );
        }
    }

    // ── Decimation math ──────────────────────────────────────────────────

    #[test]
    fn test_dc_passthrough() {
        // A DC input should pass through a LPF and decimate correctly.
        // We skip the initial filter transient — the first ceil(taps/2/factor)
        // output samples are influenced by the zero-initialized history buffer.
        let mut dec = Decimator::new(4, 0.1, 17);
        let input = vec![1.0f32; 128]; // longer input to get past transient
        let output = dec.process(&input);

        assert_eq!(output.len(), 128 / 4);

        // Skip transient: ceil(num_taps / (2 * 4)) + 1
        let skip = (17 / (2 * 4)) + 2;
        for &v in &output[skip..] {
            assert!((v - 1.0).abs() < 0.01, "DC passthrough failed: {}", v);
        }
    }

    #[test]
    fn test_output_length() {
        let mut dec = Decimator::with_factor(8);
        let input = vec![0.0f32; 256];
        let out = dec.process(&input);
        assert_eq!(out.len(), 256 / 8);
    }

    #[test]
    fn test_nyquist_rejection() {
        // A signal at exactly Nyquist (alternating +1/-1) should be
        // heavily attenuated by the LPF before decimation.
        // Use more taps for better stopband attenuation.
        let mut dec = Decimator::new(4, 0.1, 63);
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
        let out_one = dec_one.process(&signal);

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
                i,
                out_one[i],
                out_two[i]
            );
        }
    }

    #[test]
    fn test_reset_clears_state() {
        let mut dec = Decimator::new(4, 0.1, 17);
        let signal = vec![1.0f32; 64];
        let _ = dec.process(&signal); // warm up history
        dec.reset();

        // After reset, history is zero so output should match a fresh decimator
        let mut dec2 = Decimator::new(4, 0.1, 17);
        let out1 = dec.process(&signal);
        let out2 = dec2.process(&signal);

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
        let taps_s = design_lowpass(17, 0.1);
        let out_s = {
            let taps_len = taps_s.len();
            let overlap = taps_len - 1;
            let mut ext = vec![0.0f32; overlap];
            ext.extend_from_slice(&signal);
            let mut out = Vec::new();
            let mut ph = 0usize;
            fir_decimate_scalar(&ext, &taps_s, 4, &mut ph, &mut out);
            out
        };

        // NEON path
        let out_n = {
            let taps_n = design_lowpass(17, 0.1);
            let taps_len = taps_n.len();
            let overlap = taps_len - 1;
            let mut ext = vec![0.0f32; overlap];
            ext.extend_from_slice(&signal);
            let mut out = Vec::new();
            let mut ph = 0usize;
            unsafe {
                fir_decimate_neon(&ext, &taps_n, 4, &mut ph, &mut out);
            }
            out
        };

        assert_eq!(out_s.len(), out_n.len());
        for (i, (&s, &n)) in out_s.iter().zip(out_n.iter()).enumerate() {
            assert!(
                (s - n).abs() < 1e-4,
                "NEON/scalar mismatch at [{}]: scalar={} neon={}",
                i,
                s,
                n
            );
        }
    }

    #[test]
    fn test_am_demod() {
        let mut am = AmDemodulator::new();
        // Constant amplitude 1.0 -> 0.5 -> 1.0 -> 0.5
        let input = vec![1.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0];
        let output = am.process(&input);
        assert_eq!(output.len(), 4);
        // The first sample is always 0.0 due to DC initialization logic
        assert_eq!(output[0], 0.0);

        // Check that the demodulator is actually reacting to the 1.0 -> 0.5 drop
        // Since it's a high-pass/DC-removed signal, a drop in magnitude
        // should result in a negative-going value.
        assert!(
            output[1] < 0.0,
            "Output should drop when magnitude decreases"
        );
    }

    #[test]
    fn test_audio_agc_hang_time() {
        let mut agc = AudioAgc::new(1.0, 1.0, 0.01, 10.0, 1000.0, 0.01); // 10 samples hang

        // 1. Signal at target
        let mut data = vec![1.0f32; 1];
        agc.process(&mut data);
        assert!((data[0] - 1.0).abs() < 0.1);

        // 2. Signal drops to zero (below min_magnitude 0.01)
        let mut silence = vec![0.0f32; 5];
        agc.process(&mut silence);
        // Gain should stay near 1.0 (frozen)
        assert!((agc.gain - 1.0).abs() < 1e-5);

        // 3. Signal just above noise floor
        let mut noise = vec![0.02f32; 10];
        agc.process(&mut noise);
        // Decay should finally start after hang time
        assert!(agc.gain > 1.0);
    }

    #[test]
    fn test_decimator_update_cutoff() {
        let mut dec = Decimator::new(4, 0.1, 17);
        let taps_orig = dec.taps.clone();
        dec.update_cutoff(0.2);
        assert_ne!(dec.taps, taps_orig);
        assert_eq!(dec.taps.len(), 17);
    }
}
