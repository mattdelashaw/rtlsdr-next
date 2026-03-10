use crate::device::{Device, TransportBuffer, HardwareInterface};
use crate::error::{Error, Result};
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

/// A generic buffer that automatically returns itself to a pool when dropped.
pub struct PooledBuffer<B> {
    inner: Option<B>,
    pool_tx: Option<mpsc::Sender<B>>,
}

impl<B> PooledBuffer<B> {
    pub fn new(buffer: B, pool_tx: Option<mpsc::Sender<B>>) -> Self {
        Self { inner: Some(buffer), pool_tx }
    }
}

impl<B> Deref for PooledBuffer<B> {
    type Target = B;
    fn deref(&self) -> &Self::Target { self.inner.as_ref().unwrap() }
}

impl<B> DerefMut for PooledBuffer<B> {
    fn deref_mut(&mut self) -> &mut Self::Target { self.inner.as_mut().unwrap() }
}

impl<B> Drop for PooledBuffer<B> {
    fn drop(&mut self) {
        if let (Some(buffer), Some(tx)) = (self.inner.take(), &self.pool_tx) {
            let _ = tx.try_send(buffer);
        }
    }
}

/// A stream of raw interleaved U8 samples (I, Q, I, Q...).
pub struct SampleStream<T: UsbContext + 'static> {
    receiver: mpsc::Receiver<crate::Result<PooledBuffer<TransportBuffer<T>>>>,
    pub(crate) cancel_token: CancellationToken,
}

impl<T: UsbContext + 'static> SampleStream<T> {
    pub fn new(device: Arc<Device<T>>) -> Self {
        let (tx, rx) = mpsc::channel(NUM_BUFFERS);
        let (pool_tx, mut pool_rx) = mpsc::channel::<TransportBuffer<T>>(NUM_BUFFERS);
        
        // Pre-fill the pool with DMA-capable TransportBuffers
        for _ in 0..NUM_BUFFERS {
            // Each buffer holds an Arc to the device
            let buf = TransportBuffer::new(device.clone(), BUFFER_SIZE);
            let _ = pool_tx.try_send(buf);
        }

        let cancel_token = CancellationToken::new();
        let cancel_clone = cancel_token.clone();
        
        std::thread::spawn(move || {
            loop {
                if cancel_clone.is_cancelled() { break; }
                
                // Get a TransportBuffer from the pool
                let mut buf = match pool_rx.blocking_recv() {
                    Some(b) => b,
                    None => break,
                };

                // Read into the buffer (buf derefs to &mut [u8])
                match device.read_bulk(BULK_ENDPOINT, &mut buf, Duration::from_millis(100)) {
                    Ok(n) => {
                        if n > 0 {
                            let pooled = PooledBuffer::new(buf, Some(pool_tx.clone()));
                            if tx.blocking_send(Ok(pooled)).is_err() { break; }
                        } else {
                            // Zero-byte read: the DMA buffer may have been partially
                            // written by the host controller. We recycle it anyway —
                            // n == 0 means no useful data was committed, so the stale
                            // bytes will be overwritten on the next successful read.
                            let _ = pool_tx.try_send(buf);
                        }
                    }
                    Err(crate::Error::Usb(rusb::Error::Timeout)) => {
                        // Timeout: same situation — buffer is dirty but will be
                        // overwritten before any caller sees it.
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

    pub async fn next(&mut self) -> Option<crate::Result<PooledBuffer<TransportBuffer<T>>>> {
        self.receiver.recv().await
    }

    pub fn close(&self) { self.cancel_token.cancel(); }
}

impl<T: UsbContext> Drop for SampleStream<T> {
    fn drop(&mut self) { self.cancel_token.cancel(); }
}

/// A high-level DSP stream that produces interleaved F32 samples.
pub struct F32Stream<T: UsbContext + 'static> {
    raw_stream: SampleStream<T>,
    decimator:  Option<Decimator>,
    dc_remover: Option<crate::dsp::DcRemover>,
    agc:        Option<crate::dsp::Agc>,
    
    // Output Pools (Vec<f32> is sufficient here, no DMA needed for DSP output)
    pool_f32_tx:    mpsc::Sender<Vec<f32>>,
    pool_f32_rx:    mpsc::Receiver<Vec<f32>>,
    
    pool_dec_tx:    mpsc::Sender<Vec<f32>>,
    pool_dec_rx:    mpsc::Receiver<Vec<f32>>,
}

impl<T: UsbContext + 'static> F32Stream<T> {
    pub fn new(raw_stream: SampleStream<T>, decimation_factor: usize) -> Self {
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

    pub async fn next(&mut self) -> Option<crate::Result<PooledBuffer<Vec<f32>>>> {
        // raw_res is PooledBuffer<TransportBuffer<T>>
        let raw_res = self.raw_stream.next().await?;
        let u8_data_buffer = match raw_res {
            Ok(data) => data,
            Err(e) => return Some(Err(e)),
        };
        
        // Deref the PooledBuffer to get TransportBuffer, then deref that to get &[u8]
        let u8_data = &*u8_data_buffer;

        let mut f32_buf = self.pool_f32_rx.recv().await.unwrap();
        if f32_buf.len() != u8_data.len() { f32_buf.resize(u8_data.len(), 0.0); }

        converter::convert(u8_data, &mut f32_buf);

        if let Some(dc) = &mut self.dc_remover { dc.process(&mut f32_buf); }
        if let Some(agc) = &mut self.agc { agc.process(&mut f32_buf); }

        if let Some(dec) = &mut self.decimator {
            let mut dec_buf = self.pool_dec_rx.recv().await.unwrap();
            dec.process_into(&f32_buf, &mut dec_buf);
            
            // Return intermediate buffer to pool
            let _ = self.pool_f32_tx.try_send(f32_buf);
            
            Some(Ok(PooledBuffer::new(dec_buf, Some(self.pool_dec_tx.clone()))))
        } else {
            Some(Ok(PooledBuffer::new(f32_buf, Some(self.pool_f32_tx.clone()))))
        }
    }

    pub fn close(&self) { self.raw_stream.close(); }
}

impl<T: UsbContext + 'static> Drop for F32Stream<T> {
    fn drop(&mut self) {
        self.raw_stream.cancel_token.cancel();
    }
}