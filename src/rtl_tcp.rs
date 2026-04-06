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
    /// Standalone entry point — owns the `Driver` and starts its own stream loop.
    /// Used by `src/bin/rtl_tcp.rs`. Not for daemon use.
    pub async fn start(driver: Driver, addr: &str) -> Result<Self> {
        let driver = Arc::new(Mutex::new(driver));
        let cancel_token = CancellationToken::new();

        let server = Self {
            driver: driver.clone(),
            addr: addr.to_string(),
            cancel_token: cancel_token.clone(),
        };

        // Standalone mode: create an internal broadcast channel and pump the
        // driver stream into it. This mirrors the daemon pattern so client
        // handling is identical in both modes.
        let (tx, _) = broadcast::channel::<Arc<Vec<u8>>>(32);
        let pump_driver = driver.clone();
        let pump_tx = tx.clone();
        let pump_token = cancel_token.clone();

        tokio::spawn(async move {
            loop {
                if pump_token.is_cancelled() {
                    break;
                }
                let mut stream = {
                    let d = pump_driver.lock().await;
                    d.stream()
                };
                loop {
                    tokio::select! {
                        _ = pump_token.cancelled() => break,
                        res = stream.next() => match res {
                            Some(Ok(samples)) => {
                                let _ = pump_tx.send(Arc::new(samples.to_vec()));
                            }
                            Some(Err(e)) => {
                                warn!("Stream error: {:?}. Restarting...", e);
                                break;
                            }
                            None => {
                                warn!("Stream ended unexpectedly. Restarting...");
                                break;
                            }
                        }
                    }
                }
                if pump_token.is_cancelled() {
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            }
            info!("rtl_tcp stream pump stopped");
        });

        tokio::spawn(run_listener(driver, tx, addr.to_string(), cancel_token));

        Ok(server)
    }

    /// Daemon entry point — receives a shared `Driver` handle and a pre-existing
    /// broadcast receiver from the `Daemon` broadcast pump.
    ///
    /// The server does not own the stream and does not call `Driver::stream()`.
    /// Hardware configuration (frequency, gain, sample rate) is performed by
    /// `Daemon::start` before this is called.
    pub async fn start_shared(
        driver: Arc<Mutex<Driver>>,
        sample_rx: broadcast::Receiver<Arc<Vec<u8>>>,
        addr: &str,
    ) -> Result<()> {
        // Re-broadcast the injected receiver into a fresh sender so the
        // listener task can subscribe additional per-client receivers.
        let (tx, _) = broadcast::channel::<Arc<Vec<u8>>>(32);
        let relay_tx = tx.clone();

        tokio::spawn(async move {
            let mut rx = sample_rx;
            loop {
                match rx.recv().await {
                    Ok(block) => {
                        let _ = relay_tx.send(block);
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        warn!("rtl_tcp relay lagged, dropped {} blocks", n);
                    }
                    Err(_) => break,
                }
            }
        });

        let cancel_token = CancellationToken::new();
        run_listener(driver, tx, addr.to_string(), cancel_token).await
    }

    pub fn stop(&self) {
        self.cancel_token.cancel();
    }
}

// ── Listener ──────────────────────────────────────────────────────────────────

/// Accepts TCP connections and spawns a `handle_client` task for each.
/// Shared by both `start` (standalone) and `start_shared` (daemon).
async fn run_listener(
    driver: Arc<Mutex<Driver>>,
    tx: broadcast::Sender<Arc<Vec<u8>>>,
    addr: String,
    cancel_token: CancellationToken,
) -> Result<()> {
    let listener = TcpListener::bind(&addr).await?;
    info!("rtl_tcp listening on {}", addr);

    loop {
        tokio::select! {
            _ = cancel_token.cancelled() => break,
            res = listener.accept() => {
                match res {
                    Ok((socket, peer)) => {
                        info!("rtl_tcp client connected: {}", peer);
                        let client_driver  = driver.clone();
                        let client_tx      = tx.clone();
                        let client_token   = cancel_token.clone();
                        tokio::spawn(async move {
                            if let Err(e) = handle_client(
                                client_driver, socket, client_tx, client_token,
                            ).await {
                                warn!("Client {} error: {:?}", peer, e);
                            }
                            info!("rtl_tcp client disconnected: {}", peer);
                        });
                    }
                    Err(e) => warn!("Accept error: {:?}", e),
                }
            }
        }
    }

    Ok(())
}

// ── Client handler ────────────────────────────────────────────────────────────

async fn handle_client(
    driver: Arc<Mutex<Driver>>,
    mut socket: tokio::net::TcpStream,
    tx: broadcast::Sender<Arc<Vec<u8>>>,
    cancel_token: CancellationToken,
) -> anyhow::Result<()> {
    // 1. Send handshake header (12 bytes)
    let tuner_id = {
        let d = driver.lock().await;
        d.tuner_type.id()
    };

    let mut header = [0u8; 12];
    header[0..4].copy_from_slice(RTL_TCP_MAGIC);
    BigEndian::write_u32(&mut header[4..8], tuner_id);
    BigEndian::write_u32(&mut header[8..12], 29); // gain_count

    socket.write_all(&header).await?;
    socket.set_nodelay(true)?;

    let (mut reader, mut writer) = socket.into_split();

    // Unified writer channel — all tasks funnel outgoing bytes through here
    // so command responses and sample data never interleave on the socket.
    let (writer_tx, mut writer_rx) = mpsc::channel::<Arc<Vec<u8>>>(128);

    let mut writer_task = tokio::spawn(async move {
        while let Some(data) = writer_rx.recv().await {
            if let Err(e) = writer.write_all(&data).await {
                trace!("Writer task closed: {:?}", e);
                break;
            }
        }
        Ok::<(), anyhow::Error>(())
    });

    // Command task — reads 5-byte rtl_tcp commands and dispatches to Driver
    let cmd_driver = driver.clone();
    let mut cmd_task = tokio::spawn(async move {
        let mut buf = [0u8; 5];
        loop {
            reader.read_exact(&mut buf).await?;
            let cmd = buf[0];
            let arg = BigEndian::read_u32(&buf[1..5]);
            let mut d = cmd_driver.lock().await;
            trace!("rtl_tcp cmd 0x{:02x} arg={}", cmd, arg);
            match cmd {
                0x01 => { let _ = d.set_frequency(arg as u64); }
                0x02 => { let _ = d.set_sample_rate(arg); }
                // 0x03: gain mode — apply 30 dB for both auto and manual
                // (SDR++ doesn't reliably follow with 0x04)
                0x03 => {
                    let current = d.tuner.get_gain().unwrap_or(0.0);
                    if arg == 0 || (arg == 1 && current < 1.0) {
                        let _ = d.tuner.set_gain(30.0);
                    }
                }
                0x04 => { let _ = d.tuner.set_gain(arg as f32 / 10.0); }
                0x05 => { let _ = d.set_ppm(arg as i32); }
                0x08..=0x0a => {} // AGC / direct sampling / offset tuning — ignored
                0x0d => {} // SDR++ confirmation request — silently acknowledged
                0x0e => { let _ = d.set_bias_t(arg != 0); }
                // 0x13: SDR++ gain-by-index slider
                0x13 => { let _ = d.tuner.set_gain_by_index(arg as usize); }
                _ => warn!("Unsupported rtl_tcp command: 0x{:02x}", cmd),
            }
        }
        #[allow(unreachable_code)]
        Ok::<(), anyhow::Error>(())
    });

    // Data task — forwards broadcast blocks to the unified writer
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
                    warn!("rtl_tcp client lagging, dropped {} blocks", n);
                }
                Err(_) => break,
            }
        }
        Ok::<(), anyhow::Error>(())
    });

    tokio::select! {
        _ = cancel_token.cancelled() => {}
        _ = &mut cmd_task    => {}
        _ = &mut data_task   => {}
        _ = &mut writer_task => {}
    }
    cmd_task.abort();
    data_task.abort();
    writer_task.abort();

    Ok(())
}
