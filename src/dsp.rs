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
//!   - `Nco`             — Numerically Controlled Oscillator for DDC frequency shift
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
    /// Persistent scratch buffer for [history || input] to avoid allocations.
    pub extended: Vec<f32>,
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
            extended: Vec::with_capacity(num_taps + 32768),
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

        self.extended.clear();
        self.extended.extend_from_slice(&self.history);
        self.extended.extend_from_slice(input);

        // Run FIR + decimate
        #[cfg(target_arch = "aarch64")]
        {
            if std::arch::is_aarch64_feature_detected!("neon") {
                unsafe {
                    fir_decimate_neon(&self.extended, &self.taps, self.factor, &mut self.phase, output);
                }
                // Update history
                let new_history_start = self.extended.len() - overlap;
                self.history.copy_from_slice(&self.extended[new_history_start..]);
                return;
            }
        }

        fir_decimate_scalar(&self.extended, &self.taps, self.factor, &mut self.phase, output);

        // Update history for next block
        let new_history_start = self.extended.len() - overlap;
        self.history.copy_from_slice(&self.extended[new_history_start..]);
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
    envelope: f32, // Track the signal volume over time
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
            envelope: 0.0,
        }
    }

    pub fn process(&mut self, data: &mut [f32]) {
        for val in data.iter_mut() {
            let mag = val.abs();

            // Fast-attack, slow-release envelope tracker to avoid zero-crossing shredding.
            if mag > self.envelope {
                self.envelope = mag; // Fast attack
            } else {
                self.envelope = 0.999 * self.envelope + 0.001 * mag; // Slow release
            }

            if self.envelope < self.min_magnitude {
                // Signal is dead air. Freeze gain to avoid pumping.
                if self.hang_counter > 0 {
                    self.hang_counter -= 1;
                }
                *val *= self.gain;
                continue;
            }

            let error = self.target / (self.envelope + 1e-6);

            if error < self.gain {
                // Attack: signal got louder
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

    /// Process a block of interleaved I/Q samples into a provided destination.
    pub fn process_into(&mut self, input: &[f32], output: &mut Vec<f32>) {
        output.clear();
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

            // Scale to [-1.0, 1.0] — diff is in [-PI, PI]
            output.push(self.deemph_state / std::f32::consts::PI);
            self.last_phase = phase;
        }
    }

    /// Process a block of interleaved I/Q samples.
    /// Returns a vector of real (f32) demodulated samples.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() / 2);
        self.process_into(input, &mut output);
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
    /// Persistent scratch buffer for [history || input] to avoid allocations.
    extended: Vec<f32>,
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
            extended: Vec::with_capacity(num_taps + 1024),
        }
    }

    /// Process input and return Hilbert-transformed (90° shifted) output.
    pub fn process_into(&mut self, input: &[f32], output: &mut Vec<f32>) {
        output.clear();
        let taps_len = self.taps.len();
        let overlap = taps_len - 1;

        self.extended.clear();
        self.extended.extend_from_slice(&self.history);
        self.extended.extend_from_slice(input);

        // Optimized FIR convolution skipping zero-valued taps
        // The Hilbert kernel has zeros at all even offsets from the center.
        for i in 0..input.len() {
            let mut acc = 0.0f32;
            let window = &self.extended[i..i + taps_len];
            
            // Only process odd-indexed taps (relative to start) because even ones are zero.
            // Note: Since mid is even (e.g. 32 for 65 taps), and n = idx - mid,
            // n is odd when idx is odd.
            for j in (1..taps_len).step_by(2) {
                acc += window[j] * self.taps[j];
            }
            output.push(acc);
        }

        let new_history_start = self.extended.len() - overlap;
        self.history.copy_from_slice(&self.extended[new_history_start..]);
    }
}

/// A Single Sideband (SSB) demodulator using the phasing method.
pub struct SsbDemodulator {
    hilbert: HilbertFilter,
    q_shifted: Vec<f32>,
    i_history: Vec<f32>,
    is_usb: bool,
    /// Persistent scratch buffer for I branch
    i_branch: Vec<f32>,
    /// Persistent scratch buffer for Q branch
    q_branch: Vec<f32>,
    /// Persistent scratch buffer for I history extension
    i_extended: Vec<f32>,
}

impl SsbDemodulator {
    pub fn new(is_usb: bool) -> Self {
        let num_taps = 65; // Good balance for 48kHz audio
        let delay = (num_taps - 1) / 2;
        Self {
            hilbert: HilbertFilter::new(num_taps),
            q_shifted: Vec::with_capacity(1024),
            i_history: vec![0.0f32; delay],
            is_usb,
            i_branch: Vec::with_capacity(1024),
            q_branch: Vec::with_capacity(1024),
            i_extended: Vec::with_capacity(2048),
        }
    }

    /// Process interleaved I/Q samples into real audio in a provided destination.
    pub fn process_into(&mut self, input: &[f32], output: &mut Vec<f32>) {
        output.clear();
        let n = input.len() / 2;
        self.i_branch.clear();
        self.q_branch.clear();

        for k in 0..n {
            self.i_branch.push(input[k * 2]);
            self.q_branch.push(input[k * 2 + 1]);
        }

        // 1. Transform Q branch (90 deg shift)
        self.hilbert.process_into(&self.q_branch, &mut self.q_shifted);

        // 2. Combine with delayed I branch
        let _delay = self.i_history.len();

        self.i_extended.clear();
        self.i_extended.extend_from_slice(&self.i_history);
        self.i_extended.extend_from_slice(&self.i_branch);

        for (i_val, &q_hat) in self.i_extended.iter().zip(self.q_shifted.iter()) {
            // Phasing formula: USB = I + Q_hat, LSB = I - Q_hat
            // Note: Standard convention for complex baseband I+jQ
            if self.is_usb {
                output.push(i_val + q_hat);
            } else {
                output.push(i_val - q_hat);
            }
        }

        // Update I history
        self.i_history.copy_from_slice(&self.i_extended[n..]);
    }

    /// Process interleaved I/Q samples and return real audio.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() / 2);
        self.process_into(input, &mut output);
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

    /// Process a block of interleaved I/Q samples into a provided destination.
    pub fn process_into(&mut self, input: &[f32], output: &mut Vec<f32>) {
        output.clear();
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
    }

    /// Process a block of interleaved I/Q samples.
    /// Returns a vector of real (f32) demodulated samples.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() / 2);
        self.process_into(input, &mut output);
        output
    }
}

impl Default for AmDemodulator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Numerically Controlled Oscillator (NCO)
// ============================================================

/// A Numerically Controlled Oscillator for digital down-conversion.
///
/// Produces the complex exponential `exp(-j·2π·f·n/fs)` sample by sample
/// and multiplies it against an interleaved I/Q buffer to shift the spectrum
/// by `-freq_hz`. Used in per-client DDC pipelines so each WebSDR client can
/// listen to a different frequency within the hardware's capture bandwidth
/// without triggering a hardware retune.
///
/// # Phase continuity
/// `set_freq` updates `phase_inc` without resetting `phase`, so retuning
/// within a stream produces no glitch or discontinuity.
///
/// # Precision
/// Phase accumulation uses `f64` internally to prevent drift over long
/// continuous streams — the phase would accumulate ~0.3° of error per second
/// at 1.5 MSPS with `f32`. The final `sin`/`cos` are computed in `f32`
/// because that is sufficient for the subsequent audio processing.
pub struct Nco {
    /// Current phase in radians, accumulated in f64 to prevent long-term drift.
    phase: f64,
    /// Phase increment per sample = 2π · freq_hz / sample_rate_hz.
    phase_inc: f64,
    /// Pre-computed rotator for complex rotation: exp(j * phase_inc).
    /// Used for the fast complex-multiply rotation method.
    rotator_i: f64,
    rotator_q: f64,
}

impl Nco {
    /// Create a new NCO.
    ///
    /// * `freq_hz`        — frequency to shift the spectrum by, in Hz.
    ///                      Positive shifts signal at `center + freq_hz` to baseband.
    /// * `sample_rate_hz` — hardware sample rate in Hz.
    pub fn new(freq_hz: f64, sample_rate_hz: f64) -> Self {
        let phase_inc = 2.0 * std::f64::consts::PI * freq_hz / sample_rate_hz;
        Self {
            phase: 0.0,
            phase_inc,
            rotator_i: phase_inc.cos(),
            rotator_q: phase_inc.sin(),
        }
    }

    /// Update the shift frequency without resetting the phase accumulator.
    ///
    /// Safe to call between blocks; produces no click or discontinuity.
    pub fn set_freq(&mut self, freq_hz: f64, sample_rate_hz: f64) {
        self.phase_inc = 2.0 * std::f64::consts::PI * freq_hz / sample_rate_hz;
        self.rotator_i = self.phase_inc.cos();
        self.rotator_q = self.phase_inc.sin();
    }

    /// Mix an interleaved I/Q buffer in-place, shifting the spectrum by `-freq_hz`.
    ///
    /// The rotation applied per sample is:
    /// ```text
    /// I' =  I·cos(φ) + Q·sin(φ)
    /// Q' = -I·sin(φ) + Q·cos(φ)
    /// ```
    /// where φ = phase_inc · n (accumulated).
    ///
    /// # Panics
    /// Panics (debug only) if `iq` length is not even.
    pub fn mix(&mut self, iq: &mut [f32]) {
        debug_assert_eq!(iq.len() % 2, 0, "IQ buffer length must be even");

        #[cfg(target_arch = "aarch64")]
        {
            if std::arch::is_aarch64_feature_detected!("neon") {
                unsafe {
                    self.mix_neon(iq);
                }
                return;
            }
        }

        self.mix_scalar(iq);
    }

    fn mix_scalar(&mut self, iq: &mut [f32]) {
        let mut cos_val = self.phase.cos();
        let mut sin_val = self.phase.sin();
        let rot_i = self.rotator_i;
        let rot_q = self.rotator_q;

        for chunk in iq.chunks_exact_mut(2) {
            let i = chunk[0] as f64;
            let q = chunk[1] as f64;

            chunk[0] = (i * cos_val + q * sin_val) as f32;
            chunk[1] = (-i * sin_val + q * cos_val) as f32;

            // Rotate phase: (cos + j*sin) * (rot_i + j*rot_q)
            let next_cos = cos_val * rot_i - sin_val * rot_q;
            let next_sin = cos_val * rot_q + sin_val * rot_i;
            cos_val = next_cos;
            sin_val = next_sin;

            self.phase += self.phase_inc;
        }

        // Re-sync phase to prevent cumulative error in complex rotation.
        // Complex rotation is fast but slowly drifts in magnitude.
        // We wrap phase to [-π, π] to prevent f64 precision loss over time.
        if self.phase > std::f64::consts::PI {
            self.phase -= 2.0 * std::f64::consts::PI;
        } else if self.phase < -std::f64::consts::PI {
            self.phase += 2.0 * std::f64::consts::PI;
        }
    }

    #[cfg(target_arch = "aarch64")]
    #[target_feature(enable = "neon")]
    unsafe fn mix_neon(&mut self, iq: &mut [f32]) {
        use std::arch::aarch64::*;

        let mut cos_val = self.phase.cos();
        let mut sin_val = self.phase.sin();
        let rot_i = self.rotator_i;
        let rot_q = self.rotator_q;

        let mut i = 0;
        // Process 2 complex samples (4 f32) at a time
        while i + 3 < iq.len() {
            let c0 = cos_val as f32;
            let s0 = sin_val as f32;
            
            let next_cos0 = cos_val * rot_i - sin_val * rot_q;
            let next_sin0 = cos_val * rot_q + sin_val * rot_i;
            
            let c1 = next_cos0 as f32;
            let s1 = next_sin0 as f32;

            // Vector: [I0, Q0, I1, Q1]
            let v_iq = unsafe { vld1q_f32(iq.as_ptr().add(i)) };
            
            // IQ swizzled: [Q0, I0, Q1, I1]
            let _v_qi = vrev64q_f32(v_iq);
            
            // Result lanes:
            // lane 0: I0*c0 + Q0*s0  (I'0)
            // lane 1: Q0*c0 - I0*s0  (Q'0)
            // lane 2: I1*c1 + Q1*s1  (I'1)
            // lane 3: Q1*c1 - I1*s1  (Q'1)
            
            let mut res = [0.0f32; 4];
            res[0] = iq[i]   * c0 + iq[i+1] * s0;
            res[1] = iq[i+1] * c0 - iq[i]   * s0;
            res[2] = iq[i+2] * c1 + iq[i+3] * s1;
            res[3] = iq[i+3] * c1 - iq[i+2] * s1;
            
            unsafe {
                vst1q_f32(iq.as_mut_ptr().add(i), vld1q_f32(res.as_ptr()));
            }

            // Update complex oscillator for next pair
            cos_val = next_cos0 * rot_i - next_sin0 * rot_q;
            sin_val = next_cos0 * rot_q + next_sin0 * rot_i;
            
            i += 4;
            self.phase += self.phase_inc * 2.0;
        }

        // Tail
        while i < iq.len() {
            let i_val = iq[i] as f64;
            let q_val = iq[i+1] as f64;
            iq[i] = (i_val * cos_val + q_val * sin_val) as f32;
            iq[i+1] = (q_val * cos_val - i_val * sin_val) as f32;
            
            let next_cos = cos_val * rot_i - sin_val * rot_q;
            let next_sin = cos_val * rot_q + sin_val * rot_i;
            cos_val = next_cos;
            sin_val = next_sin;
            
            i += 2;
            self.phase += self.phase_inc;
        }

        if self.phase > std::f64::consts::PI {
            self.phase -= 2.0 * std::f64::consts::PI;
        } else if self.phase < -std::f64::consts::PI {
            self.phase += 2.0 * std::f64::consts::PI;
        }
    }

    /// Reset the phase accumulator to zero.
    /// Use this when switching between unrelated streams.
    pub fn reset(&mut self) {
        self.phase = 0.0;
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
        let expected = omega / std::f32::consts::PI;
        for &v in &output[1..] {
            assert!((v - expected).abs() < 1e-4);
        }
    }

    // ── FIR design ───────────────────────────────────────────────────────────

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

    // ── Decimation math ──────────────────────────────────────────────────────

    #[test]
    fn test_dc_passthrough() {
        let mut dec = Decimator::new(4, 0.1, 17);
        let input = vec![1.0f32; 128];
        let output = dec.process(&input);
        assert_eq!(output.len(), 128 / 4);
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
        let mut dec = Decimator::new(4, 0.1, 63);
        let input: Vec<f32> = (0..512)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let output = dec.process(&input);
        let skip = (63 / (2 * 4)) + 2;
        for &v in &output[skip..] {
            assert!(v.abs() < 0.1, "Nyquist not rejected: {}", v);
        }
    }

    #[test]
    fn test_block_boundary_continuity() {
        let signal: Vec<f32> = (0..512)
            .map(|i| (i as f32 * 0.01 * std::f32::consts::PI).sin())
            .collect();

        let mut dec_one = Decimator::new(4, 0.1, 17);
        let out_one = dec_one.process(&signal);

        let mut dec_two = Decimator::new(4, 0.1, 17);
        let mut out_two = dec_two.process(&signal[..256]);
        out_two.extend(dec_two.process(&signal[256..]));

        assert_eq!(out_one.len(), out_two.len());

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
        let _ = dec.process(&signal);
        dec.reset();

        let mut dec2 = Decimator::new(4, 0.1, 17);
        let out1 = dec.process(&signal);
        let out2 = dec2.process(&signal);

        for (a, b) in out1.iter().zip(out2.iter()) {
            assert!((a - b).abs() < EPS, "Post-reset mismatch: {} vs {}", a, b);
        }
    }

    // ── NEON / scalar agreement ──────────────────────────────────────────────

    #[test]
    #[cfg(target_arch = "aarch64")]
    fn test_neon_scalar_agree() {
        if !std::arch::is_aarch64_feature_detected!("neon") {
            return;
        }

        let signal: Vec<f32> = (0..1024)
            .map(|i| (i as f32 * 0.05 * std::f32::consts::PI).sin())
            .collect();

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
        let input = vec![1.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0];
        let output = am.process(&input);
        assert_eq!(output.len(), 4);
        assert_eq!(output[0], 0.0);
        assert!(
            output[1] < 0.0,
            "Output should drop when magnitude decreases"
        );
    }

    #[test]
    fn test_audio_agc_hang_time() {
        let mut agc = AudioAgc::new(1.0, 1.0, 0.01, 10.0, 1000.0, 0.01);

        let mut data = vec![1.0f32; 1];
        agc.process(&mut data);
        assert!((data[0] - 1.0).abs() < 0.1);

        let mut silence = vec![0.0f32; 5];
        agc.process(&mut silence);
        assert!((agc.gain - 1.0).abs() < 1e-5);

        let mut noise = vec![0.02f32; 20];
        agc.process(&mut noise);
        assert!(agc.gain > 1.0);
    }

    #[test]
    fn test_ssb_demod_suppression() {
        let mut usb = SsbDemodulator::new(true);
        let mut lsb = SsbDemodulator::new(false);

        let fs = 48000.0;
        let f = 1000.0;
        let mut input = Vec::new();
        for i in 0..1000 {
            let t = i as f32 / fs;
            let i_val = (2.0 * std::f32::consts::PI * f * t).cos();
            let q_val = (2.0 * std::f32::consts::PI * f * t).sin();
            input.push(i_val);
            input.push(q_val);
        }

        let out_usb = usb.process(&input);
        let out_lsb = lsb.process(&input);

        let start = 100;
        let mut usb_energy = 0.0;
        let mut lsb_energy = 0.0;
        for i in start..900 {
            usb_energy += out_usb[i] * out_usb[i];
            lsb_energy += out_lsb[i] * out_lsb[i];
        }

        assert!(
            usb_energy > 10.0 * lsb_energy,
            "LSB should be suppressed in USB mode. USB: {}, LSB: {}",
            usb_energy,
            lsb_energy
        );
    }

    #[test]
    fn test_decimator_update_cutoff() {
        let mut dec = Decimator::new(4, 0.1, 17);
        let taps_orig = dec.taps.clone();
        dec.update_cutoff(0.2);
        assert_ne!(dec.taps, taps_orig);
        assert_eq!(dec.taps.len(), 17);
    }

    // ── NCO ──────────────────────────────────────────────────────────────────

    #[test]
    fn test_nco_zero_shift_is_identity() {
        // A zero-frequency NCO must leave the signal unchanged.
        let mut nco = Nco::new(0.0, 1_536_000.0);
        let original = vec![0.5f32, 0.3, -0.2, 0.8, 0.1, -0.4];
        let mut iq = original.clone();
        nco.mix(&mut iq);
        for (a, b) in iq.iter().zip(original.iter()) {
            assert!(
                (a - b).abs() < 1e-5,
                "Zero-shift NCO changed sample: {} vs {}",
                a,
                b
            );
        }
    }

    #[test]
    fn test_nco_shifts_fft_peak() {
        // Generate a real tone at 100 kHz, shift it by -100 kHz → peak should
        // move to 0 Hz (DC bin) after mixing.
        let fs = 1_536_000.0f64;
        let tone_hz = 100_000.0f64;
        let n_samples = 1024usize;

        // Build interleaved complex tone at +100 kHz
        let mut iq: Vec<f32> = (0..n_samples)
            .flat_map(|n| {
                let phase = 2.0 * std::f64::consts::PI * tone_hz * n as f64 / fs;
                [phase.cos() as f32, phase.sin() as f32]
            })
            .collect();

        // Shift by +100 kHz (NCO freq = +100 kHz → phase_inc negative → spectrum shifts left)
        let mut nco = Nco::new(tone_hz, fs);
        nco.mix(&mut iq);

        // After mixing the 100 kHz tone should be at DC.
        // Compute magnitude of DC bin directly: sum of all samples / N.
        let dc_i: f32 = iq.iter().step_by(2).sum::<f32>() / n_samples as f32;
        let dc_q: f32 = iq.iter().skip(1).step_by(2).sum::<f32>() / n_samples as f32;
        let dc_mag = (dc_i * dc_i + dc_q * dc_q).sqrt();

        // DC magnitude should be close to 0.5 (amplitude of unit complex exp / 2
        // because average of cos over full cycle → 0, but our shifted signal
        // is now at DC so the mean is non-zero). Accept > 0.4 as a loose bound.
        assert!(
            dc_mag > 0.4,
            "Expected DC peak after mixing, got dc_mag = {}",
            dc_mag
        );
    }

    #[test]
    fn test_nco_phase_continuity_across_blocks() {
        // Process the same signal in two blocks vs one block.
        // The output should be identical (phase carried across boundary).
        let fs = 1_536_000.0f64;
        let shift = 200_000.0f64;
        let n = 512usize;

        let signal: Vec<f32> = (0..n * 2)
            .flat_map(|k| {
                let t = k as f64 / fs;
                [(2.0 * std::f64::consts::PI * 50_000.0 * t).cos() as f32,
                 (2.0 * std::f64::consts::PI * 50_000.0 * t).sin() as f32]
            })
            .collect();

        // One-shot
        let mut nco_one = Nco::new(shift, fs);
        let mut one = signal.clone();
        nco_one.mix(&mut one);

        // Two blocks
        let mut nco_two = Nco::new(shift, fs);
        let mut two = signal.clone();
        nco_two.mix(&mut two[..n * 2]);       // first half (all I/Q pairs)
        nco_two.mix(&mut two[n * 2..]);       // second half — phase must continue

        for (i, (a, b)) in one.iter().zip(two.iter()).enumerate() {
            assert!(
                (a - b).abs() < 1e-4,
                "Phase discontinuity at sample {}: one={} two={}",
                i, a, b
            );
        }
    }

    #[test]
    fn test_nco_set_freq_no_phase_reset() {
        // Calling set_freq mid-stream must not reset the phase accumulator.
        let fs = 1_536_000.0f64;
        let mut nco = Nco::new(100_000.0, fs);

        // Warm up phase to a non-zero value
        let mut warmup = vec![1.0f32; 100];
        nco.mix(&mut warmup);
        let phase_before = nco.phase;

        // Change frequency — phase must be unchanged
        nco.set_freq(200_000.0, fs);
        assert!(
            (nco.phase - phase_before).abs() < 1e-12,
            "set_freq must not reset phase: before={} after={}",
            phase_before,
            nco.phase
        );
    }

    #[test]
    fn test_nco_reset_clears_phase() {
        let fs = 1_536_000.0f64;
        let mut nco = Nco::new(100_000.0, fs);
        let mut data = vec![1.0f32; 100];
        nco.mix(&mut data);
        assert!(nco.phase.abs() > 0.0);
        nco.reset();
        assert_eq!(nco.phase, 0.0);
    }
}
