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
const NUM_BUFFERS: usize = 16;

/// A buffer that automatically returns itself to a pool when dropped.
pub struct PooledVec<T> {
    inner: Option<Vec<T>>,
    pool_tx: Option<mpsc::Sender<Vec<T>>>,
}

impl<T> PooledVec<T> {
    pub fn new(vec: Vec<T>, pool_tx: Option<mpsc::Sender<Vec<T>>>) -> Self {
        Self { inner: Some(vec), pool_tx }
    }
}

impl<T> Deref for PooledVec<T> {
    type Target = Vec<T>;
    fn deref(&self) -> &Self::Target { self.inner.as_ref().unwrap() }
}

impl<T> DerefMut for PooledVec<T> {
    fn deref_mut(&mut self) -> &mut Self::Target { self.inner.as_mut().unwrap() }
}

impl<T> Drop for PooledVec<T> {
    fn drop(&mut self) {
        if let (Some(vec), Some(tx)) = (self.inner.take(), &self.pool_tx) {
            let _ = tx.try_send(vec);
        }
    }
}

/// A stream of raw interleaved U8 samples (I, Q, I, Q...).
pub struct SampleStream {
    receiver: mpsc::Receiver<Result<PooledVec<u8>, crate::Error>>,
    cancel_token: CancellationToken,
}

impl SampleStream {
    pub fn new<T: UsbContext + 'static>(device: Arc<Device<T>>) -> Self {
        let (tx, rx) = mpsc::channel(NUM_BUFFERS);
        let (pool_tx, mut pool_rx) = mpsc::channel::<Vec<u8>>(NUM_BUFFERS);
        
        for _ in 0..NUM_BUFFERS {
            let _ = pool_tx.try_send(vec![0u8; BUFFER_SIZE]);
        }

        let cancel_token = CancellationToken::new();
        let cancel_clone = cancel_token.clone();
        
        std::thread::spawn(move || {
            loop {
                if cancel_clone.is_cancelled() { break; }
                let mut buf = match pool_rx.blocking_recv() {
                    Some(b) => b,
                    None => break,
                };

                match device.read_bulk(BULK_ENDPOINT, &mut buf, Duration::from_millis(100)) {
                    Ok(n) => {
                        if n > 0 {
                            let pooled = PooledVec::new(buf, Some(pool_tx.clone()));
                            if tx.blocking_send(Ok(pooled)).is_err() { break; }
                        } else {
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

        Self { receiver: rx, cancel_token }
    }

    pub async fn next(&mut self) -> Option<Result<PooledVec<u8>, crate::Error>> {
        self.receiver.recv().await
    }

    pub fn close(&self) { self.cancel_token.cancel(); }
}

impl Drop for SampleStream {
    fn drop(&mut self) { self.close(); }
}

/// A high-level DSP stream that produces interleaved F32 samples.
pub struct F32Stream {
    raw_stream: SampleStream,
    decimator:  Option<Decimator>,
    dc_remover: Option<crate::dsp::DcRemover>,
    agc:        Option<crate::dsp::Agc>,
    
    // Output Pools
    pool_f32_tx:    mpsc::Sender<Vec<f32>>,
    pool_f32_rx:    mpsc::Receiver<Vec<f32>>,
    
    pool_dec_tx:    mpsc::Sender<Vec<f32>>,
    pool_dec_rx:    mpsc::Receiver<Vec<f32>>,
}

impl F32Stream {
    pub fn new(raw_stream: SampleStream, decimation_factor: usize) -> Self {
        let decimator = if decimation_factor > 1 {
            Some(Decimator::with_factor(decimation_factor))
        } else {
            None
        };

        let (p1_tx, p1_rx) = mpsc::channel(NUM_BUFFERS);
        for _ in 0..NUM_BUFFERS { p1_tx.try_send(vec![0.0f32; BUFFER_SIZE]).unwrap(); }

        let (p2_tx, p2_rx) = mpsc::channel(NUM_BUFFERS);
        if decimation_factor > 1 {
            let decimated_size = BUFFER_SIZE / decimation_factor + 16;
            for _ in 0..NUM_BUFFERS { p2_tx.try_send(vec![0.0f32; decimated_size]).unwrap(); }
        }

        Self {
            raw_stream,
            decimator,
            dc_remover: None,
            agc:        None,
            pool_f32_tx: p1_tx,
            pool_f32_rx: p1_rx,
            pool_dec_tx: p2_tx,
            pool_dec_rx: p2_rx,
        }
    }

    pub fn with_dc_removal(mut self, alpha: f32) -> Self {
        self.dc_remover = Some(crate::dsp::DcRemover::new(alpha));
        self
    }

    pub fn with_agc(mut self, target: f32, attack: f32, decay: f32) -> Self {
        self.agc = Some(crate::dsp::Agc::new(target, attack, decay));
        self
    }

    pub async fn next(&mut self) -> Option<Result<PooledVec<f32>, crate::Error>> {
        let raw_res = self.raw_stream.next().await?;
        let u8_data = match raw_res {
            Ok(data) => data,
            Err(e) => return Some(Err(e)),
        };

        let mut f32_buf = self.pool_f32_rx.recv().await.unwrap();
        if f32_buf.len() != u8_data.len() { f32_buf.resize(u8_data.len(), 0.0); }

        converter::convert(&u8_data, &mut f32_buf);

        if let Some(dc) = &mut self.dc_remover { dc.process(&mut f32_buf); }
        if let Some(agc) = &mut self.agc { agc.process(&mut f32_buf); }

        if let Some(dec) = &mut self.decimator {
            let mut dec_buf = self.pool_dec_rx.recv().await.unwrap();
            dec.process_into(&f32_buf, &mut dec_buf);
            
            // Return intermediate buffer to pool
            let _ = self.pool_f32_tx.try_send(f32_buf);
            
            Some(Ok(PooledVec::new(dec_buf, Some(self.pool_dec_tx.clone()))))
        } else {
            Some(Ok(PooledVec::new(f32_buf, Some(self.pool_f32_tx.clone()))))
        }
    }

    pub fn close(&self) { self.raw_stream.close(); }
}
