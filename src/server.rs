use log::error;
use std::path::Path;
use std::sync::Arc;
use tokio::io::{AsyncWriteExt, Result as IoResult};
use tokio::net::UnixListener;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

pub struct SharingServer {
    _broadcast_tx: broadcast::Sender<Arc<Vec<u8>>>,
    _handle: JoinHandle<()>,
    cancel_token: CancellationToken,
}

impl SharingServer {
    /// Start a new sharing server on the specified Unix Domain Socket path.
    pub async fn start<P: AsRef<Path>>(
        path: P,
        mut sample_rx: broadcast::Receiver<Arc<Vec<u8>>>,
    ) -> IoResult<Self> {
        let path = path.as_ref().to_owned();
        let cancel_token = CancellationToken::new();
        let cancel_clone = cancel_token.clone();

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
                tokio::select! {
                    _ = cancel_clone.cancelled() => break,
                    accept_res = listener.accept() => {
                        match accept_res {
                            Ok((socket, _)) => {
                                let mut socket: tokio::net::UnixStream = socket;
                                let mut client_rx = tx_clone.subscribe();
                                let cancel_client = cancel_clone.clone();
                                tokio::spawn(async move {
                                    loop {
                                        tokio::select! {
                                            _ = cancel_client.cancelled() => break,
                                            recv_res = client_rx.recv() => {
                                                match recv_res {
                                                    Ok(samples) => {
                                                        if socket.write_all(&samples).await.is_err() {
                                                            break; // Client disconnected
                                                        }
                                                    }
                                                    Err(_) => break, // Broadcast closed
                                                }
                                            }
                                        }
                                    }
                                });
                            }
                            Err(e) => {
                                error!("Socket accept error: {:?}", e);
                                break;
                            }
                        }
                    }
                }
            }
            // Cleanup socket file on exit
            let _ = std::fs::remove_file(&path);
        });

        // Task 2: Relay hardware samples into our broadcast channel
        let tx_relay = tx.clone();
        let cancel_relay = cancel_token.clone();
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = cancel_relay.cancelled() => break,
                    res = sample_rx.recv() => {
                        match res {
                            Ok(samples) => {
                                let _ = tx_relay.send(samples);
                            }
                            Err(_) => break, // Upstream closed
                        }
                    }
                }
            }
        });

        Ok(Self {
            _broadcast_tx: tx,
            _handle: handle,
            cancel_token,
        })
    }

    /// Stop the server and clean up the socket file.
    pub fn stop(&self) {
        self.cancel_token.cancel();
    }
}

impl Drop for SharingServer {
    fn drop(&mut self) {
        self.stop();
    }
}
