use tokio::net::UnixListener;
use tokio::io::{AsyncWriteExt, Result as IoResult};
use tokio::sync::broadcast;
use std::sync::Arc;
use std::path::Path;
use tokio::task::JoinHandle;

pub struct SharingServer {
    _broadcast_tx: broadcast::Sender<Arc<Vec<u8>>>,
    _handle: JoinHandle<()>,
}

impl SharingServer {
    /// Start a new sharing server on the specified Unix Domain Socket path.
    pub async fn start<P: AsRef<Path>>(path: P, mut sample_rx: broadcast::Receiver<Arc<Vec<u8>>>) -> IoResult<Self> {
        let path = path.as_ref().to_owned();
        
        // Clean up existing socket file if it exists
        if path.exists() {
            let _ = std::fs::remove_file(&path);
        }

        let listener = UnixListener::bind(&path)?;
        let (tx, _rx) = broadcast::channel::<Arc<Vec<u8>>>(16); 
        let tx_clone = tx.clone();

        // Task 1: Accept connections and pipe the broadcast to them
        let handle = tokio::spawn(async move {
            loop {
                match listener.accept().await {
                    Ok((mut socket, _)) => {
                        let mut client_rx = tx_clone.subscribe();
                        tokio::spawn(async move {
                            while let Ok(samples) = client_rx.recv().await {
                                if socket.write_all(&samples).await.is_err() {
                                    break; // Client disconnected
                                }
                            }
                        });
                    }
                    Err(e) => {
                        eprintln!("Socket accept error: {:?}", e);
                        break;
                    }
                }
            }
        });

        // Task 2: Relay hardware samples into our broadcast channel
        let tx_relay = tx.clone();
        tokio::spawn(async move {
            while let Ok(samples) = sample_rx.recv().await {
                let _ = tx_relay.send(samples);
            }
        });

        Ok(Self {
            _broadcast_tx: tx,
            _handle: handle,
        })
    }
}
