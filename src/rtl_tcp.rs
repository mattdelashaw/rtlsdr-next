use crate::Driver;
use anyhow::Result;
use byteorder::{BigEndian, ByteOrder};
use log::{info, trace, warn};
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::sync::{Mutex, broadcast, mpsc};
use tokio_util::sync::CancellationToken;

const RTL_TCP_MAGIC: &[u8] = b"RTL0";

pub struct TcpServer {
    driver: Arc<Mutex<Driver>>,
    addr: String,
    cancel_token: CancellationToken,
}

impl TcpServer {
    pub async fn start(driver: Driver, addr: &str) -> Result<Self> {
        let driver = Arc::new(Mutex::new(driver));
        let cancel_token = CancellationToken::new();

        let server = Self {
            driver,
            addr: addr.to_string(),
            cancel_token,
        };

        let s_driver = server.driver.clone();
        let s_addr = server.addr.clone();
        let s_token = server.cancel_token.clone();

        tokio::spawn(async move {
            if let Err(e) = run_server(s_driver, &s_addr, s_token).await {
                warn!("rtl_tcp server error: {:?}", e);
            }
        });

        Ok(server)
    }

    pub fn stop(&self) {
        self.cancel_token.cancel();
    }
}

async fn run_server(
    driver: Arc<Mutex<Driver>>,
    addr: &str,
    cancel_token: CancellationToken,
) -> Result<()> {
    let listener = TcpListener::bind(addr).await?;
    info!("rtl_tcp server listening on {}", addr);

    let (tx, _) = broadcast::channel::<Arc<Vec<u8>>>(16);

    // Stream task: pulls from driver and broadcasts to all clients
    let stream_driver = driver.clone();
    let stream_tx = tx.clone();
    let stream_token = cancel_token.clone();

    tokio::spawn(async move {
        loop {
            if stream_token.is_cancelled() {
                break;
            }

            let mut stream = {
                let d = stream_driver.lock().await;
                d.stream()
            };

            loop {
                tokio::select! {
                    _ = stream_token.cancelled() => break,
                    res = stream.next() => {
                        match res {
                            Some(Ok(samples)) => {
                                let _ = stream_tx.send(Arc::new(samples.to_vec()));
                            }
                            Some(Err(e)) => {
                                warn!("Stream error: {:?}. Attempting to restart...", e);
                                break;
                            }
                            None => {
                                warn!("Stream ended unexpectedly. Attempting to restart...");
                                break;
                            }
                        }
                    }
                }
            }

            if stream_token.is_cancelled() {
                break;
            }
            // Wait before retrying to avoid spamming on persistent failure
            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
        }
        info!("rtl_tcp stream task stopped");
    });

    loop {
        tokio::select! {
            _ = cancel_token.cancelled() => break,
            res = listener.accept() => {
                match res {
                    Ok((socket, addr)) => {
                        info!("rtl_tcp client connected: {}", addr);
                        let client_driver = driver.clone();
                        let client_tx = tx.clone();
                        let client_token = cancel_token.clone();
                        tokio::spawn(async move {
                            if let Err(e) = handle_client(client_driver, socket, client_tx, client_token).await {
                                warn!("Client {} error: {:?}", addr, e);
                            }
                            info!("rtl_tcp client disconnected: {}", addr);
                        });
                    }
                    Err(e) => warn!("Accept error: {:?}", e),
                }
            }
        }
    }

    Ok(())
}

async fn handle_client(
    driver: Arc<Mutex<Driver>>,
    mut socket: tokio::net::TcpStream,
    tx: broadcast::Sender<Arc<Vec<u8>>>,
    cancel_token: CancellationToken,
) -> anyhow::Result<()> {
    // 1. Send Handshake Header (12 bytes)
    let tuner_id = {
        let d = driver.lock().await;
        d.tuner_type.id()
    };

    let mut header = [0u8; 12];
    header[0..4].copy_from_slice(RTL_TCP_MAGIC);
    BigEndian::write_u32(&mut header[4..8], tuner_id); // tuner_type
    BigEndian::write_u32(&mut header[8..12], 29); // gain_count

    socket.write_all(&header).await?;

    socket.set_nodelay(true)?;
    let (mut reader, mut writer) = socket.into_split();

    // Channel for unified writer task
    let (writer_tx, mut writer_rx) = mpsc::channel::<Arc<Vec<u8>>>(128);

    // Task: Unified Writer
    let mut writer_task = tokio::spawn(async move {
        while let Some(data) = writer_rx.recv().await {
            if let Err(e) = writer.write_all(&data).await {
                trace!("Writer task error: {:?}", e);
                break;
            }
        }
        Ok::<(), anyhow::Error>(())
    });

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
                0x03 => {
                    let current = d.tuner.get_gain().unwrap_or(0.0);
                    if arg == 0 || (arg == 1 && current < 1.0) {
                        let _ = d.tuner.set_gain(30.0);
                    }
                }
                0x04 => {
                    let _ = d.tuner.set_gain(arg as f32 / 10.0);
                }
                0x05 => {
                    let _ = d.set_ppm(arg as i32);
                }
                0x08..=0x0a => {}
                // 0x13: set tuner gain by index — SDR++ gain slider
                0x13 => {
                    let _ = d.tuner.set_gain_by_index(arg as usize);
                }
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
    let data_writer_tx = writer_tx.clone();
    let mut client_rx = tx.subscribe();
    let mut data_task = tokio::spawn(async move {
        loop {
            match client_rx.recv().await {
                Ok(samples) => {
                    if data_writer_tx.send(samples).await.is_err() {
                        break;
                    }
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
        _ = &mut writer_task => {},
    }
    cmd_task.abort();
    data_task.abort();
    writer_task.abort();

    Ok(())
}
