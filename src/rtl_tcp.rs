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
    _driver:      Arc<Mutex<Driver>>,
    _addr:        String,
    cancel_token: CancellationToken,
}

impl TcpServer {
    /// Standalone entry point — owns the `Driver`, runs its own broadcast pump.
    pub async fn start(driver: Driver, addr: &str) -> Result<Self> {
        let driver       = Arc::new(Mutex::new(driver));
        let cancel_token = CancellationToken::new();

        let (tx, _) = broadcast::channel::<Arc<Vec<u8>>>(64);
        {
            let pump_driver = driver.clone();
            let pump_tx     = tx.clone();
            let pump_token  = cancel_token.clone();
            tokio::spawn(async move {
                loop {
                    if pump_token.is_cancelled() { break; }
                    let mut stream = { let d = pump_driver.lock().await; d.stream() };
                    loop {
                        tokio::select! {
                            _ = pump_token.cancelled() => break,
                            res = stream.next() => match res {
                                Some(Ok(s)) => { let _ = pump_tx.send(Arc::new(s.to_vec())); }
                                Some(Err(e)) => { warn!("Stream error: {:?}", e); break; }
                                None         => { warn!("Stream ended."); break; }
                            }
                        }
                    }
                    if pump_token.is_cancelled() { break; }
                    tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                }
                info!("rtl_tcp stream pump stopped");
            });
        }

        let initial_freq = { let d = driver.lock().await; d.frequency };
        let band = Arc::new(parking_lot::RwLock::new(crate::daemon::HardwareBand {
            center_hz: initial_freq,
            span_hz:   2_048_000, // Default for standalone
            spectral_inv: false,
        }));
        let (retune_tx, _retune_rx) = mpsc::channel::<crate::daemon::RetuneRequest>(8);

        let server = Self { _driver: driver.clone(), _addr: addr.to_string(), cancel_token: cancel_token.clone() };
        tokio::spawn(run_listener(driver, tx, band, retune_tx, addr.to_string(), cancel_token, false));
        Ok(server)
    }

    /// Daemon entry point — accepts a `Sender` clone from the daemon pump.
    ///
    /// Each incoming TCP client calls `tx.subscribe()` to get its own fresh
    /// receiver. No relay task — one less async hop, one less place to lag.
    pub async fn start_shared(
        driver:    Arc<Mutex<Driver>>,
        sample_tx: broadcast::Sender<Arc<Vec<u8>>>,
        band:      Arc<parking_lot::RwLock<crate::daemon::HardwareBand>>,
        retune_tx: mpsc::Sender<crate::daemon::RetuneRequest>,
        addr:      &str,
    ) -> Result<()> {
        let cancel_token = CancellationToken::new();
        run_listener(driver, sample_tx, band, retune_tx, addr.to_string(), cancel_token, true).await
    }

    pub fn stop(&self) { self.cancel_token.cancel(); }
}

// ── Listener ──────────────────────────────────────────────────────────────────

async fn run_listener(
    driver:       Arc<Mutex<Driver>>,
    tx:           broadcast::Sender<Arc<Vec<u8>>>,
    band:         Arc<parking_lot::RwLock<crate::daemon::HardwareBand>>,
    retune_tx:    mpsc::Sender<crate::daemon::RetuneRequest>,
    addr:         String,
    cancel_token: CancellationToken,
    is_shared:    bool,
) -> Result<()> {
    let listener = TcpListener::bind(&addr).await?;
    info!("rtl_tcp listening on {}", addr);

    loop {
        tokio::select! {
            _ = cancel_token.cancelled() => break,
            res = listener.accept() => match res {
                Ok((socket, peer)) => {
                    info!("rtl_tcp client connected: {}", peer);
                    let client_driver = driver.clone();
                    let client_rx     = tx.subscribe();
                    let client_token  = cancel_token.clone();
                    let client_band   = band.clone();
                    let client_retune = retune_tx.clone();
                    tokio::spawn(async move {
                        if let Err(e) = handle_client(client_driver, socket, client_rx, client_token, client_band, client_retune, is_shared).await {
                            warn!("Client {} error: {:?}", peer, e);
                        }
                        info!("rtl_tcp client disconnected: {}", peer);
                    });
                }
                Err(e) => warn!("Accept error: {:?}", e),
            }
        }
    }
    Ok(())
}

// ── Client handler ────────────────────────────────────────────────────────────

async fn handle_client(
    driver:       Arc<Mutex<Driver>>,
    mut socket:   tokio::net::TcpStream,
    mut client_rx: broadcast::Receiver<Arc<Vec<u8>>>,
    cancel_token: CancellationToken,
    _band:        Arc<parking_lot::RwLock<crate::daemon::HardwareBand>>,
    retune_tx:    mpsc::Sender<crate::daemon::RetuneRequest>,
    is_shared:    bool,
) -> anyhow::Result<()> {
    // 1. Send handshake header
    let (tuner_id, gains) = { 
        let d = driver.lock().await; 
        (d.tuner_type.id(), d.tuner.get_gain_table())
    };
    
    let mut header = [0u8; 12];
    header[0..4].copy_from_slice(RTL_TCP_MAGIC);
    BigEndian::write_u32(&mut header[4..8],  tuner_id);
    BigEndian::write_u32(&mut header[8..12], gains.len() as u32);
    socket.write_all(&header).await?;

    // 2. Send gain table (MANDATORY for protocol sync in SDR++, GQRX, etc.)
    let mut gain_buf = vec![0u8; gains.len() * 4];
    for (i, &gain) in gains.iter().enumerate() {
        BigEndian::write_i32(&mut gain_buf[i * 4..(i + 1) * 4], gain);
    }
    socket.write_all(&gain_buf).await?;
    socket.set_nodelay(true)?;

    let (mut reader, mut writer) = socket.into_split();
    let (writer_tx, mut writer_rx) = mpsc::channel::<Arc<Vec<u8>>>(1024);

    // Unified writer
    let mut writer_task = tokio::spawn(async move {
        while let Some(data) = writer_rx.recv().await {
            if let Err(e) = writer.write_all(&data).await {
                trace!("Writer closed: {:?}", e);
                break;
            }
        }
        Ok::<(), anyhow::Error>(())
    });

    // Command task
    let cmd_driver = driver.clone();
    let mut cmd_task = tokio::spawn(async move {
        let mut buf = [0u8; 5];
        loop {
            reader.read_exact(&mut buf).await?;
            let cmd = buf[0];
            let arg = BigEndian::read_u32(&buf[1..5]);
            let mut d = cmd_driver.lock().await;
            info!("rtl_tcp cmd 0x{:02x} arg={} (freq {} Hz)", cmd, arg, arg);
            match cmd {
                0x01 => {
                    // Try to use the shared retune channel first (for daemon mode sync)
                    if retune_tx.send(crate::daemon::RetuneRequest { center_hz: arg as u64 }).await.is_err() {
                        // Fallback to direct tuning (for standalone mode)
                        let _ = d.set_frequency(arg as u64, None);
                    }
                }
                0x02 => { 
                    if is_shared {
                        warn!("rtl_tcp: ignoring sample rate change request to {} Hz (daemon mode)", arg);
                    } else {
                        let _ = d.set_sample_rate(arg);
                    }
                }
                0x03 => {
                    // 0 = Auto Gain (Enable AGC), 1 = Manual Gain
                    if arg == 0 {
                        let _ = d.set_agc(true);
                    } else {
                        let _ = d.set_agc(false);
                        // Default to a sensible manual gain if none set
                        let cur = d.tuner.get_gain().unwrap_or(0.0);
                        if cur < 1.0 { let _ = d.tuner.set_gain(30.0); }
                    }
                }
                0x04 => { 
                    let _ = d.set_agc(false);
                    let db = arg as f32 / 10.0;
                    trace!("rtl_tcp: setting manual gain to {:.1} dB", db);
                    let _ = d.tuner.set_gain(db); 
                }
                0x05 => { let _ = d.set_ppm(arg as i32); }
                0x08 => { let _ = d.set_agc(arg != 0); }
                0x09..=0x0a => {}
                0x0d => {} // SDR++ confirmation — silently ack
                0x0e => { let _ = d.set_bias_t(arg != 0); }
                0x13 => { 
                    let _ = d.set_agc(false);
                    trace!("rtl_tcp: setting gain by index {}", arg);
                    let _ = d.tuner.set_gain_by_index(arg as usize); 
                }
                _    => warn!("Unsupported rtl_tcp cmd: 0x{:02x}", cmd),
            }
        }
        #[allow(unreachable_code)]
        Ok::<(), anyhow::Error>(())
    });

    // Data task — forwards broadcast blocks to writer
    let data_writer_tx = writer_tx.clone();
    let mut data_task = tokio::spawn(async move {
        loop {
            match client_rx.recv().await {
                Ok(samples) => {
                    // Use non-blocking try_send to avoid backpressuring the global broadcast pump.
                    // If the TCP client is too slow, we drop the block.
                    if let Err(e) = data_writer_tx.try_send(samples) {
                        match e {
                            mpsc::error::TrySendError::Full(_) => {
                                // Drop block silently to avoid log spam, but protect the system.
                                trace!("rtl_tcp: buffer full, dropping block");
                            }
                            mpsc::error::TrySendError::Closed(_) => break,
                        }
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
