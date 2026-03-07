use crate::device::Device;
use rusb::UsbContext;
use std::sync::Arc;
use tokio::sync::mpsc;
use std::time::Duration;
use tokio_util::sync::CancellationToken;
use log::error;

use crate::converter;
use crate::dsp::Decimator;

const BULK_ENDPOINT: u8 = 0x81;
const BUFFER_SIZE: usize = 256 * 1024; // 256KB buffers
const NUM_BUFFERS: usize = 8; // Double/Triple buffering

/// A stream of raw interleaved U8 samples (I, Q, I, Q...).
pub struct SampleStream {
    receiver: mpsc::Receiver<Result<Vec<u8>, crate::Error>>,
    cancel_token: CancellationToken,
}

impl SampleStream {
    pub fn new<T: UsbContext + 'static>(device: Arc<Device<T>>) -> Self {
        let (tx, rx) = mpsc::channel(NUM_BUFFERS);
        let cancel_token = CancellationToken::new();
        let cancel_clone = cancel_token.clone();
        
        // Use a dedicated thread for high-throughput USB reads
        std::thread::spawn(move || {
            let mut buf = vec![0u8; BUFFER_SIZE];
            
            loop {
                // Check for cancellation
                if cancel_clone.is_cancelled() {
                    break;
                }

                // Synchronous bulk read with a 100ms timeout
                match device.read_bulk(BULK_ENDPOINT, &mut buf, Duration::from_millis(100)) {
                    Ok(n) => {
                        if n > 0 {
                            if tx.blocking_send(Ok(buf[..n].to_vec())).is_err() {
                                break; // Receiver dropped
                            }
                        }
                    }
                    Err(crate::Error::Usb(rusb::Error::Timeout)) => {
                        continue;
                    }
                    Err(e) => {
                        error!("USB Bulk Read Error: {:?}", e);
                        // Surface the error to the async consumer before exiting
                        let _ = tx.blocking_send(Err(e));
                        break;
                    }
                }
            }
        });

        Self {
            receiver: rx,
            cancel_token,
        }
    }

    /// Asynchronously receive the next chunk of samples.
    pub async fn next(&mut self) -> Option<Result<Vec<u8>, crate::Error>> {
        self.receiver.recv().await
    }

    pub fn close(&self) {
        self.cancel_token.cancel();
    }
}

impl Drop for SampleStream {
    fn drop(&mut self) {
        self.close();
    }
}

/// A high-level DSP stream that produces interleaved F32 samples.
///
/// Automatically handles:
/// 1. Hardware U8 -> F32 conversion (NEON accelerated)
/// 2. FIR Low-pass filtering (NEON accelerated)
/// 3. Decimation (Downsampling)
pub struct F32Stream {
    raw_stream: SampleStream,
    decimator:  Option<Decimator>,
    dc_remover: Option<crate::dsp::DcRemover>,
    agc:        Option<crate::dsp::Agc>,
    // Reusable buffers to avoid allocations in the hot loop
    f32_buf:    Vec<f32>,
}

impl F32Stream {
    pub fn new(raw_stream: SampleStream, decimation_factor: usize) -> Self {
        let decimator = if decimation_factor > 1 {
            Some(Decimator::with_factor(decimation_factor))
        } else {
            None
        };

        Self {
            raw_stream,
            decimator,
            dc_remover: None,
            agc:        None,
            f32_buf:    Vec::with_capacity(BUFFER_SIZE),
        }
    }

    /// Enable DC removal on the stream.
    pub fn with_dc_removal(mut self, alpha: f32) -> Self {
        self.dc_remover = Some(crate::dsp::DcRemover::new(alpha));
        self
    }

    /// Enable Automatic Gain Control (AGC) on the stream.
    pub fn with_agc(mut self, target: f32, attack: f32, decay: f32) -> Self {
        self.agc = Some(crate::dsp::Agc::new(target, attack, decay));
        self
    }

    /// Asynchronously receive the next chunk of processed F32 samples.
    pub async fn next(&mut self) -> Option<Result<Vec<f32>, crate::Error>> {
        // 1. Get raw bytes from hardware
        let raw_res = self.raw_stream.next().await?;
        
        let u8_data = match raw_res {
            Ok(data) => data,
            Err(e) => return Some(Err(e)),
        };

        // 2. Convert U8 -> F32 (interleaved I/Q)
        // Ensure buffer is the right size
        if self.f32_buf.len() != u8_data.len() {
            self.f32_buf.resize(u8_data.len(), 0.0);
        }
        converter::convert(&u8_data, &mut self.f32_buf);

        // 3. DC Removal (optional)
        if let Some(dc) = &mut self.dc_remover {
            dc.process(&mut self.f32_buf);
        }

        // 4. AGC (optional)
        if let Some(agc) = &mut self.agc {
            agc.process(&mut self.f32_buf);
        }

        // 5. Apply Decimation if requested
        if let Some(dec) = &mut self.decimator {
            let decimated = dec.process(&self.f32_buf);
            Some(Ok(decimated))
        } else {
            // No decimation, return the full-rate F32 buffer
            Some(Ok(self.f32_buf.clone()))
        }
    }

    pub fn close(&self) {
        self.raw_stream.close();
    }
}
