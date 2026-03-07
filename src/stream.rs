use crate::device::Device;
use rusb::UsbContext;
use std::sync::Arc;
use tokio::sync::mpsc;
use std::time::Duration;
use tokio_util::sync::CancellationToken;
use log::error;
use std::ops::{Deref, DerefMut};

use crate::converter;
use crate::dsp::Decimator;

const BULK_ENDPOINT: u8 = 0x81;
const BUFFER_SIZE: usize = 256 * 1024; // 256KB buffers
const NUM_BUFFERS: usize = 16; // Increased for smoother pooling

/// A buffer that automatically returns itself to a pool when dropped.
pub struct Pooled<T> {
    inner: Option<T>,
    pool_tx: Option<mpsc::Sender<T>>,
}

impl<T> Pooled<T> {
    pub fn new(data: T, pool_tx: Option<mpsc::Sender<T>>) -> Self {
        Self {
            inner: Some(data),
            pool_tx,
        }
    }
}

impl<T> Deref for Pooled<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        self.inner.as_ref().unwrap()
    }
}

impl<T> DerefMut for Pooled<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.as_mut().unwrap()
    }
}

impl<T> Drop for Pooled<T> {
    fn drop(&mut self) {
        if let (Some(data), Some(pool_tx)) = (self.inner.take(), &self.pool_tx) {
            // Return to pool. If receiver dropped, the buffer is just freed.
            let _ = pool_tx.try_send(data);
        }
    }
}

/// A stream of raw interleaved U8 samples (I, Q, I, Q...).
pub struct SampleStream {
    receiver: mpsc::Receiver<Result<Pooled<Vec<u8>>, crate::Error>>,
    cancel_token: CancellationToken,
}

impl SampleStream {
    pub fn new<T: UsbContext + 'static>(device: Arc<Device<T>>) -> Self {
        let (tx, rx) = mpsc::channel(NUM_BUFFERS);
        let (pool_tx, mut pool_rx) = mpsc::channel::<Vec<u8>>(NUM_BUFFERS);
        
        // Pre-fill the pool
        for _ in 0..NUM_BUFFERS {
            let _ = pool_tx.try_send(vec![0u8; BUFFER_SIZE]);
        }

        let cancel_token = CancellationToken::new();
        let cancel_clone = cancel_token.clone();
        
        // Use a dedicated thread for high-throughput USB reads
        std::thread::spawn(move || {
            loop {
                if cancel_clone.is_cancelled() {
                    break;
                }

                // 1. Get a buffer from the pool
                let mut buf = match pool_rx.blocking_recv() {
                    Some(b) => b,
                    None => break, // Pool shut down
                };

                // 2. Synchronous bulk read
                match device.read_bulk(BULK_ENDPOINT, &mut buf, Duration::from_millis(100)) {
                    Ok(n) => {
                        if n > 0 {
                            // Trim buffer to actual read size before sending
                            // Note: for a true zero-alloc pool, we'd keep the full size 
                            // but send the length separately. For now, we'll just 
                            // ensure we don't reallocate.
                            let pooled = Pooled::new(buf, Some(pool_tx.clone()));
                            if tx.blocking_send(Ok(pooled)).is_err() {
                                break;
                            }
                        } else {
                            // Put empty buffer back
                            let _ = pool_tx.try_send(buf);
                        }
                    }
                    Err(crate::Error::Usb(rusb::Error::Timeout)) => {
                        let _ = pool_tx.try_send(buf);
                        continue;
                    }
                    Err(e) => {
                        error!("USB Bulk Read Error: {:?}", e);
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
    pub async fn next(&mut self) -> Option<Result<Pooled<Vec<u8>>, crate::Error>> {
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
pub struct F32Stream {
    raw_stream: SampleStream,
    decimator:  Option<Decimator>,
    dc_remover: Option<crate::dsp::DcRemover>,
    agc:        Option<crate::dsp::Agc>,
    
    // F32 output pool
    pool_tx:    mpsc::Sender<Vec<f32>>,
    pool_rx:    mpsc::Receiver<Vec<f32>>,
}

impl F32Stream {
    pub fn new(raw_stream: SampleStream, decimation_factor: usize) -> Self {
        let decimator = if decimation_factor > 1 {
            Some(Decimator::with_factor(decimation_factor))
        } else {
            None
        };

        let (pool_tx, pool_rx) = mpsc::channel(NUM_BUFFERS);
        for _ in 0..NUM_BUFFERS {
            let _ = pool_tx.try_send(vec![0.0f32; BUFFER_SIZE]);
        }

        Self {
            raw_stream,
            decimator,
            dc_remover: None,
            agc:        None,
            pool_tx,
            pool_rx,
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
    pub async fn next(&mut self) -> Option<Result<Pooled<Vec<f32>>, crate::Error>> {
        // 1. Get raw bytes from hardware (pooled)
        let raw_res = self.raw_stream.next().await?;
        let u8_data = match raw_res {
            Ok(data) => data,
            Err(e) => return Some(Err(e)),
        };

        // 2. Get an F32 buffer from our pool
        let mut f32_buf = self.pool_rx.recv().await.unwrap();
        if f32_buf.len() != u8_data.len() {
            f32_buf.resize(u8_data.len(), 0.0);
        }

        // 3. Convert U8 -> F32 (interleaved I/Q)
        converter::convert(&u8_data, &mut f32_buf);

        // 4. DC Removal (optional)
        if let Some(dc) = &mut self.dc_remover {
            dc.process(&mut f32_buf);
        }

        // 5. AGC (optional)
        if let Some(agc) = &mut self.agc {
            agc.process(&mut f32_buf);
        }

        // 6. Apply Decimation if requested
        if let Some(dec) = &mut self.decimator {
            // Note: Decimator::process currently returns a new Vec.
            // In a future refactor, it should process into a destination buffer.
            let decimated = dec.process(&f32_buf);
            // Return f32_buf to pool early since we have the decimated copy
            let _ = self.pool_tx.try_send(f32_buf);
            
            // We can't pool the decimated Vec easily without a more complex pool 
            // that handles various sizes, but decimation reduces the data volume 
            // by 'factor', so the impact is smaller.
            // We pass None for pool_tx because this is not a pooled buffer.
            Some(Ok(Pooled::new(decimated, None)))
        } else {
            Some(Ok(Pooled::new(f32_buf, Some(self.pool_tx.clone()))))
        }
    }

    pub fn close(&self) {
        self.raw_stream.close();
    }
}
