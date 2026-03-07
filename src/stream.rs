use crate::device::Device;
use crate::error::Result;
use rusb::UsbContext;
use std::sync::Arc;
use tokio::sync::mpsc;
use std::time::Duration;

const BULK_ENDPOINT: u8 = 0x81;
const BUFFER_SIZE: usize = 256 * 1024; // 256KB buffers
const NUM_BUFFERS: usize = 8; // Double/Triple buffering

pub struct SampleStream {
    receiver: mpsc::Receiver<Vec<u8>>,
    _abort_handle: tokio::task::AbortHandle,
}

impl SampleStream {
    pub fn new<T: UsbContext + 'static>(device: Arc<Device<T>>) -> Self {
        let (tx, rx) = mpsc::channel(NUM_BUFFERS);
        
        let task = tokio::task::spawn_blocking(move || {
            let mut buf = vec![0u8; BUFFER_SIZE];
            
            loop {
                match device.read_bulk(BULK_ENDPOINT, &mut buf, Duration::from_secs(1)) {
                    Ok(n) => {
                        if tx.blocking_send(buf[..n].to_vec()).is_err() {
                            break;
                        }
                    }
                    Err(e) => {
                        eprintln!("USB Bulk Read Error: {:?}", e);
                        break;
                    }
                }
            }
        });

        Self {
            receiver: rx,
            _abort_handle: task.abort_handle(),
        }
    }

    pub async fn next(&mut self) -> Option<Vec<u8>> {
        self.receiver.recv().await
    }
}
