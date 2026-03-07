//! DSP and Decimation Logic with SIMD Acceleration

pub struct Decimator {
    decimation_factor: usize,
    filter_taps: Vec<f32>,
    _history: Vec<f32>,
}

impl Decimator {
    pub fn new(factor: usize) -> Self {
        // Simple low-pass filter taps (normalized)
        let taps = vec![1.0 / factor as f32; factor];
        
        Self {
            decimation_factor: factor,
            filter_taps: taps,
            _history: Vec::new(),
        }
    }

    /// Process a block of f32 samples and return the decimated version.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() / self.decimation_factor);
        
        #[cfg(target_arch = "aarch64")]
        {
            // Use the NEON optimized path if on ARM64
            self.process_neon(input, &mut output);
        }

        #[cfg(not(target_arch = "aarch64"))]
        {
            // Scalar fallback for x86/macOS
            self.process_scalar(input, &mut output);
        }
        
        output
    }

    fn process_scalar(&self, input: &[f32], output: &mut Vec<f32>) {
        for i in (0..input.len()).step_by(self.decimation_factor) {
            if i + self.decimation_factor <= input.len() {
                let mut sum = 0.0;
                for j in 0..self.decimation_factor {
                    sum += input[i + j] * self.filter_taps[j];
                }
                output.push(sum);
            }
        }
    }

    #[cfg(target_arch = "aarch64")]
    fn process_neon(&self, input: &[f32], output: &mut Vec<f32>) {
        use std::arch::aarch64::*;
        
        let taps_ptr = self.filter_taps.as_ptr();
        let factor = self.decimation_factor;

        for i in (0..input.len()).step_by(factor) {
            if i + factor <= input.len() {
                let mut j = 0;
                unsafe {
                    // Accumulator register initialized to zero
                    let mut v_acc = vdupq_n_f32(0.0);
                    
                    // Process 4 taps at a time
                    while j + 4 <= factor {
                        let v_samples = vld1q_f32(input.as_ptr().add(i + j));
                        let v_taps = vld1q_f32(taps_ptr.add(j));
                        
                        // Multiply and Accumulate: acc = acc + (samples * taps)
                        v_acc = vmlaq_f32(v_acc, v_samples, v_taps);
                        j += 4;
                    }
                    
                    // Collapse 4 lanes into 1
                    let mut sum = vaddvq_f32(v_acc);
                    
                    // Handle remaining taps (if factor is not multiple of 4)
                    while j < factor {
                        sum += input[i + j] * self.filter_taps[j];
                        j += 1;
                    }
                    
                    output.push(sum);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decimation_math() {
        let mut decimator = Decimator::new(4);
        // [1,1,1,1, 2,2,2,2] -> [1.0, 2.0]
        let input = vec![1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0];
        let output = decimator.process(&input);
        
        assert_eq!(output.len(), 2);
        assert!((output[0] - 1.0).abs() < f32::EPSILON);
        assert!((output[1] - 2.0).abs() < f32::EPSILON);
    }
}
