use byteorder::{BigEndian, ByteOrder};
use log::{error, info, trace, warn};
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tokio_util::sync::CancellationToken;

use crate::Driver;

// rtl_tcp protocol constants
const RTL_TCP_MAGIC: &[u8; 4] = b"RTL0";

pub struct TcpServer {
    cancel_token: CancellationToken,
}

impl TcpServer {
    /// Start an rtl_tcp compatible server.
    /// This allows apps like OpenWebRX, SDR#, or GQRX to connect over the network.
    pub async fn start(driver: Driver, addr: &str) -> std::io::Result<Self> {
        let listener = TcpListener::bind(addr).await?;
        let addr = listener.local_addr()?;
        info!("rtl_tcp server listening on {}", addr);

        let cancel_token = CancellationToken::new();
        let cancel_accept = cancel_token.clone();

        // Create a broadcast channel for raw samples
        let (tx, _) = broadcast::channel::<Arc<Vec<u8>>>(128);
        let tx_clone = tx.clone();

        // Task 1: Sample Relay
        let mut raw_stream = driver.stream();
        tokio::spawn(async move {
            while let Some(res) = raw_stream.next().await {
                match res {
                    Ok(samples) => {
                        // Clone once into an Arc for broadcasting to multiple clients
                        let _ = tx_clone.send(Arc::new(samples.to_vec()));
                    }
                    Err(e) => {
                        error!("Hardware stream error: {:?}", e);
                        break;
                    }
                }
            }
        });

        // Task 2: Accept Loop
        let driver = Arc::new(tokio::sync::Mutex::new(driver));

        tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = cancel_accept.cancelled() => break,
                    accept_res = listener.accept() => {
                        match accept_res {
                            Ok((socket, client_addr)) => {
                                info!("New client connected: {}", client_addr);
                                let client_tx = tx.clone();
                                let client_driver = driver.clone();
                                let client_cancel = cancel_accept.clone();
                                tokio::spawn(async move {
                                    if let Err(e) = handle_client(socket, client_tx, client_driver, client_cancel).await {
                                        warn!("Client {} disconnected with error: {:?}", client_addr, e);
                                    } else {
                                        info!("Client {} disconnected", client_addr);
                                    }
                                });
                            }
                            Err(e) => {
                                error!("TCP accept error: {:?}", e);
                                break;
                            }
                        }
                    }
                }
            }
        });

        Ok(Self { cancel_token })
    }

    pub fn stop(&self) {
        self.cancel_token.cancel();
    }
}

async fn handle_client(
    mut socket: TcpStream,
    tx: broadcast::Sender<Arc<Vec<u8>>>,
    driver: Arc<tokio::sync::Mutex<Driver>>,
    cancel_token: CancellationToken,
) -> anyhow::Result<()> {
    // 1. Send Handshake Header (12 bytes)
    let mut header = [0u8; 12];
    header[0..4].copy_from_slice(RTL_TCP_MAGIC);
    BigEndian::write_u32(&mut header[4..8], 5); // tuner_type
    BigEndian::write_u32(&mut header[8..12], 29); // gain_count

    socket.write_all(&header).await?;

    let mut client_rx = tx.subscribe();
    // Use into_split to get owned halves for 'static tasks
    let (mut reader, mut writer) = socket.into_split();

    // Task: Process commands from client
    let cmd_driver = driver.clone();
    let mut cmd_task = tokio::spawn(async move {
        let mut buf = [0u8; 5];
        loop {
            reader.read_exact(&mut buf).await?;
            let cmd = buf[0];
            let arg = BigEndian::read_u32(&buf[1..5]);

            let mut d = cmd_driver.lock().await;
            trace!("Received command: {:?}, arg: {:?}", cmd, arg);
            match cmd {
                0x01 => {
                    let r = d.set_frequency(arg as u64);
                    trace!("set_frequency({}) = {:?}", arg, r);
                }
                0x02 => {
                    let _ = d.set_sample_rate(arg);
                }
                // 0x03: set gain mode — 0=auto(AGC), 1=manual
                // When auto, set a reasonable default gain; manual gain comes via 0x04
                0x03 => {
                    if arg == 0 {
                        let _ = d.tuner.set_gain(30.0); // auto: use mid gain
                    }
                }
                0x04 => {
                    let _ = d.tuner.set_gain(arg as f32 / 10.0);
                }
                0x05 => {
                    let _ = d.set_ppm(arg as i32);
                }
                // 0x08: set RTL AGC mode (demod AGC) — ignore for now
                // 0x09: set direct sampling — ignore (V4 handles this internally)
                // 0x0a: set offset tuning — ignore
                0x08..=0x0a => {}
                0x0e => {
                    let _ = d.set_bias_t(arg != 0);
                }
                _ => {
                    warn!("Unsupported rtl_tcp command: 0x{:02x}", cmd);
                }
            }
        }
        #[allow(unreachable_code)]
        Ok::<(), anyhow::Error>(())
    });

    // Task: Push samples to client
    let mut data_task = tokio::spawn(async move {
        loop {
            match client_rx.recv().await {
                Ok(samples) => {
                    writer.write_all(&samples).await?;
                }
                Err(broadcast::error::RecvError::Lagged(n)) => {
                    warn!("Client lagging, dropped {} blocks", n);
                }
                Err(_) => break,
            }
        }
        Ok::<(), anyhow::Error>(())
    });

    tokio::select! {
        _ = cancel_token.cancelled() => {},
        _ = &mut cmd_task => {},
        _ = &mut data_task => {},
    }

    cmd_task.abort();
    data_task.abort();
    Ok(())
}
