use crate::Driver;
use crate::dsp::{AmDemodulator, Decimator, FmDemodulator};
use axum::{
    Router,
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::IntoResponse,
    routing::get,
};
use axum_server::tls_rustls::RustlsConfig;
use futures_util::{SinkExt, StreamExt};
use log::{error, info};
use rustfft::{FftPlanner, num_complex::Complex};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::{Mutex, broadcast};

// ── Constants ────────────────────────────────────────────────────────────────

/// Hardware sample rate for the WebSDR pipeline.
/// 1_536_000 / 32 = 48_000 exactly — all decimation chains target 48 kHz audio.
pub const PIPELINE_SAMPLE_RATE: u32 = 1_536_000;

/// Audio sample rate delivered to browser clients.
pub const AUDIO_SAMPLE_RATE: u32 = 48_000;

/// FFT size for spectrum / waterfall.
const FFT_SIZE: usize = 1024;

/// Target display frame rate for waterfall / spectrum updates.
const DISPLAY_FPS: f32 = 10.0;

// ── WebSocket protocol ────────────────────────────────────────────────────────

/// Demod modes — lowercase JSON tags match the iOS client `DemodMode` enum.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(rename_all = "lowercase")]
enum DemodMode {
    Wfm, // Wideband FM — broadcast, ~200 kHz
    Nfm, // Narrowband FM — GMRS/ham, ~12.5 kHz
    Am,  // Amplitude modulation — aircraft, AM broadcast
    Usb, // Upper sideband
    Lsb, // Lower sideband
    Off,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type", rename_all = "lowercase")]
enum Command {
    Frequency { hz: u64 },
    Gain { db: f32 },
    Demod { mode: DemodMode },
    Bandwidth { hz: u32 },
}

#[derive(Serialize, Debug)]
#[serde(tag = "type", rename_all = "lowercase")]
#[allow(dead_code)]
enum WebEvent {
    HardwareInfo {
        manufacturer: String,
        product: String,
        is_v4: bool,
    },
    /// Sent once on connect — iOS uses this to configure AVAudioEngine sample rate.
    AudioConfig {
        sample_rate: u32,
        fft_size: usize,
    },
    FrequencyChange { hz: u64 },
    GainChange { db: f32 },
}

// ── Server state ──────────────────────────────────────────────────────────────

pub struct WebSdrServer {
    driver: Arc<Mutex<Driver>>,
    /// Waterfall row: FFT_SIZE magnitude bytes per message (u8, 0–255).
    waterfall_tx: broadcast::Sender<Vec<u8>>,
    /// Demodulated audio at AUDIO_SAMPLE_RATE, Arc-wrapped to avoid copies.
    audio_tx: broadcast::Sender<Arc<Vec<f32>>>,
    demod_tx: broadcast::Sender<DemodMode>,
    bandwidth_tx: broadcast::Sender<u32>,
}

impl WebSdrServer {
    /// Standalone entry point — takes ownership of the `Driver`, applies
    /// hardware config, and blocks until the HTTP server exits.
    /// Used by `src/bin/websdr.rs`. Not for daemon use.
    pub async fn start(
        driver: Driver,
        addr: &str,
        tls: Option<(PathBuf, PathBuf)>,
    ) -> anyhow::Result<()> {
        let driver = Arc::new(Mutex::new(driver));

        // Standalone: apply hardware config before starting the pipeline.
        {
            let mut d = driver.lock().await;
            info!("WebSDR: setting sample rate to {} Hz", PIPELINE_SAMPLE_RATE);
            if let Err(e) = d.set_sample_rate(PIPELINE_SAMPLE_RATE) {
                error!("CRITICAL: Failed to set sample rate: {:?}", e);
            }
            if d.frequency == 0 {
                let _ = d.set_frequency(101_100_000);
                let _ = d.tuner.set_gain(30.0);
            }
        }

        // Standalone: create a private broadcast channel and pump the driver stream.
        let (raw_tx, _) = broadcast::channel::<Arc<Vec<u8>>>(32);
        {
            let pump_driver = driver.clone();
            let pump_tx = raw_tx.clone();
            tokio::spawn(async move {
                let mut stream = {
                    let d = pump_driver.lock().await;
                    d.stream()
                };
                while let Some(res) = stream.next().await {
                    match res {
                        Ok(buf) => { let _ = pump_tx.send(Arc::new(buf.to_vec())); }
                        Err(e) => { error!("WebSDR stream error: {:?}", e); break; }
                    }
                }
            });
        }

        Self::serve(driver, raw_tx.subscribe(), addr, tls).await
    }

    /// Daemon entry point — receives a shared `Driver` handle and a broadcast
    /// receiver from the `Daemon` pump. Hardware config is already applied.
    ///
    /// The server does not own the stream and does not call `Driver::stream()`.
    pub async fn start_shared(
        driver: Arc<Mutex<Driver>>,
        sample_rx: broadcast::Receiver<Arc<Vec<u8>>>,
        addr: &str,
        tls: Option<(PathBuf, PathBuf)>,
    ) -> anyhow::Result<()> {
        Self::serve(driver, sample_rx, addr, tls).await
    }

    /// Internal: build the Axum app, start the pipeline task, and bind.
    /// Shared by both `start` and `start_shared`.
    async fn serve(
        driver: Arc<Mutex<Driver>>,
        sample_rx: broadcast::Receiver<Arc<Vec<u8>>>,
        addr: &str,
        tls: Option<(PathBuf, PathBuf)>,
    ) -> anyhow::Result<()> {
        let (waterfall_tx, _) = broadcast::channel(16);
        let (audio_tx, _) = broadcast::channel(16);
        let (demod_tx, _) = broadcast::channel(16);
        let (bandwidth_tx, _) = broadcast::channel(16);

        let state = Arc::new(Self {
            driver,
            waterfall_tx,
            audio_tx,
            demod_tx,
            bandwidth_tx,
        });

        let pipeline_state = state.clone();
        tokio::spawn(async move {
            if let Err(e) = run_pipeline(pipeline_state, sample_rx).await {
                error!("WebSDR pipeline error: {:?}", e);
            }
        });

        let app = Router::new()
            .route("/", get(index_handler))
            .route("/ws", get(ws_handler))
            .route("/favicon.ico", get(favicon_handler))
            .with_state(state);

        if let Some((cert, key)) = tls {
            info!("WebSDR listening on https://{} (wss://)", addr);
            let config = RustlsConfig::from_pem_file(cert, key).await?;
            axum_server::bind_rustls(addr.parse()?, config)
                .serve(app.into_make_service())
                .await?;
        } else {
            let listener = tokio::net::TcpListener::bind(addr).await?;
            info!("WebSDR listening on http://{} (ws://)", addr);
            axum::serve(listener, app).await?;
        }

        Ok(())
    }
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────

async fn index_handler() -> impl IntoResponse {
    axum::response::Html(include_str!("../assets/websdr_ui.html"))
}

async fn favicon_handler() -> impl IntoResponse {
    const ICON: &[u8] = include_bytes!("../assets/favicon.ico");
    (
        [(axum::http::header::CONTENT_TYPE, "image/x-icon")],
        axum::response::Response::new(axum::body::Body::from(ICON)),
    )
}

// ── WebSocket upgrade ─────────────────────────────────────────────────────────

async fn ws_handler(
    ws: WebSocketUpgrade,
    axum::extract::State(state): axum::extract::State<Arc<WebSdrServer>>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_socket(socket, state))
}

async fn handle_socket(mut socket: WebSocket, state: Arc<WebSdrServer>) {
    let (info, freq, gain) = {
        let d = state.driver.lock().await;
        (
            d.info.clone(),
            d.frequency,
            d.tuner.get_gain().unwrap_or(0.0),
        )
    };

    // Handshake: hardware info + audio config + current tuning state
    let _ = socket
        .send(Message::Text(
            serde_json::to_string(&WebEvent::HardwareInfo {
                manufacturer: info.manufacturer,
                product: info.product,
                is_v4: info.is_v4,
            })
            .unwrap()
            .into(),
        ))
        .await;

    let _ = socket
        .send(Message::Text(
            serde_json::to_string(&WebEvent::AudioConfig {
                sample_rate: AUDIO_SAMPLE_RATE,
                fft_size: FFT_SIZE,
            })
            .unwrap()
            .into(),
        ))
        .await;

    let _ = socket
        .send(Message::Text(
            serde_json::json!({"type":"freqconfirm","requested":freq,"actual":freq})
                .to_string()
                .into(),
        ))
        .await;

    let _ = socket
        .send(Message::Text(
            serde_json::json!({"type":"gainconfirm","actual":gain})
                .to_string()
                .into(),
        ))
        .await;

    let mut waterfall_rx = state.waterfall_tx.subscribe();
    let mut audio_rx = state.audio_tx.subscribe();
    let (mut sink, mut receiver) = socket.split();
    let (ws_tx, mut ws_rx) = tokio::sync::mpsc::channel::<Message>(128);

    // Unified writer — all tasks push to ws_tx; this task drains to the socket.
    let mut sink_task = tokio::spawn(async move {
        while let Some(msg) = ws_rx.recv().await {
            if sink.send(msg).await.is_err() {
                break;
            }
        }
    });

    // Waterfall: binary 'W' + FFT_SIZE bytes
    let wf_tx = ws_tx.clone();
    let mut wf_task = tokio::spawn(async move {
        while let Ok(data) = waterfall_rx.recv().await {
            let mut msg = Vec::with_capacity(1 + data.len());
            msg.push(b'W');
            msg.extend_from_slice(&data);
            if wf_tx.send(Message::Binary(msg.into())).await.is_err() {
                break;
            }
        }
    });

    // Audio: binary 'A' + 3-byte padding + f32 LE PCM
    // Padding aligns the float data to a 4-byte boundary for Float32Array in JS.
    let au_tx = ws_tx.clone();
    let mut au_task = tokio::spawn(async move {
        while let Ok(data) = audio_rx.recv().await {
            let mut msg = Vec::with_capacity(4 + data.len() * 4);
            msg.extend_from_slice(&[b'A', 0, 0, 0]);
            for &s in data.iter() {
                msg.extend_from_slice(&s.to_le_bytes());
            }
            if au_tx.send(Message::Binary(msg.into())).await.is_err() {
                break;
            }
        }
    });

    // Command handler
    let cmd_driver = state.driver.clone();
    let cmd_demod_tx = state.demod_tx.clone();
    let cmd_reply_tx = ws_tx.clone();
    let state_clone = state.clone();
    let mut cmd_task = tokio::spawn(async move {
        while let Some(Ok(Message::Text(text))) = receiver.next().await {
            let Ok(cmd) = serde_json::from_str::<Command>(&text) else {
                continue;
            };
            match cmd {
                Command::Frequency { hz } => {
                    let mut d = cmd_driver.lock().await;
                    let reply = match d.set_frequency(hz) {
                        Ok(actual) => {
                            serde_json::json!({"type":"freqconfirm","requested":hz,"actual":actual})
                        }
                        Err(e) => {
                            serde_json::json!({"type":"error","cmd":"setfrequency","msg":e.to_string()})
                        }
                    };
                    let _ = cmd_reply_tx
                        .send(Message::Text(reply.to_string().into()))
                        .await;
                }
                Command::Gain { db } => {
                    let d = cmd_driver.lock().await;
                    let reply = match d.tuner.set_gain(db) {
                        Ok(actual) => serde_json::json!({"type":"gainconfirm","actual":actual}),
                        Err(e) => {
                            serde_json::json!({"type":"error","cmd":"setgain","msg":e.to_string()})
                        }
                    };
                    let _ = cmd_reply_tx
                        .send(Message::Text(reply.to_string().into()))
                        .await;
                }
                Command::Demod { mode } => {
                    let _ = cmd_demod_tx.send(mode);
                    let reply = serde_json::json!({"type":"demodconfirm","mode":mode});
                    let _ = cmd_reply_tx
                        .send(Message::Text(reply.to_string().into()))
                        .await;
                }
                Command::Bandwidth { hz } => {
                    let clamped = hz.clamp(500, 380_000);
                    let _ = state_clone.bandwidth_tx.send(clamped);
                    let reply = serde_json::json!({"type":"bandwidthconfirm","actual":clamped});
                    let _ = cmd_reply_tx
                        .send(Message::Text(reply.to_string().into()))
                        .await;
                }
            }
        }
    });

    tokio::select! {
        _ = &mut sink_task => {}
        _ = &mut wf_task   => {}
        _ = &mut au_task   => {}
        _ = &mut cmd_task  => {}
    }
    sink_task.abort();
    wf_task.abort();
    au_task.abort();
    cmd_task.abort();
}

// ── DSP pipeline ──────────────────────────────────────────────────────────────

/// Receives raw IQ from `sample_rx`, demodulates, and pushes waterfall + audio
/// to the broadcast channels in `state`.
///
/// In standalone mode the caller provides a receiver from an internal pump.
/// In daemon mode the receiver comes directly from `Daemon`'s broadcast pump —
/// no hardware interaction happens here.
async fn run_pipeline(
    state: Arc<WebSdrServer>,
    mut sample_rx: broadcast::Receiver<Arc<Vec<u8>>>,
) -> anyhow::Result<()> {
    let mut current_mode = DemodMode::Wfm;
    let mut demod_rx = state.demod_tx.subscribe();
    let mut bandwidth_rx = state.bandwidth_tx.subscribe();

    // ── Demodulators ─────────────────────────────────────────────────────────
    let intermediate_rate_wfm = PIPELINE_SAMPLE_RATE as f32 / 8.0; // 192 kHz
    let mut fm_wfm = FmDemodulator::new().with_deemphasis(intermediate_rate_wfm, 75e-6);
    let mut fm_nfm = FmDemodulator::new();
    let mut am = AmDemodulator::new();
    let mut ssb_usb = crate::dsp::SsbDemodulator::new(true);
    let mut ssb_lsb = crate::dsp::SsbDemodulator::new(false);

    // ── Audio AGC ────────────────────────────────────────────────────────────
    // target = -15 dBFS (0.17), attack = 0.1, decay = 0.001,
    // hang = 500 ms, min_magnitude = 0.02 (-34 dBFS)
    let mut audio_agc =
        crate::dsp::AudioAgc::new(0.17, 0.1, 0.001, 500.0, AUDIO_SAMPLE_RATE as f32, 0.02);

    // ── Decimation chains ─────────────────────────────────────────────────────
    // WFM : ÷8  → 192 kHz → FM discriminate → ÷4 → 48 kHz
    // NFM : ÷32 → 48 kHz (direct)
    // AM  : ÷32 → 48 kHz → envelope detect
    // SSB : ÷32 → 48 kHz → phasing method
    let mut pre_i_wfm  = Decimator::new(8,  0.45 / 8.0,  31);
    let mut pre_q_wfm  = Decimator::new(8,  0.45 / 8.0,  31);
    let mut post_wfm   = Decimator::new(4,  0.45 / 4.0,  65);

    let mut pre_i_nfm  = Decimator::new(32, 0.45 / 32.0, 31);
    let mut pre_q_nfm  = Decimator::new(32, 0.45 / 32.0, 31);

    let mut pre_i_am   = Decimator::new(32, 6000.0 / PIPELINE_SAMPLE_RATE as f32, 63);
    let mut pre_q_am   = Decimator::new(32, 6000.0 / PIPELINE_SAMPLE_RATE as f32, 63);

    // 6.4 kHz total bandwidth (±3.2 kHz) — standard for high-fidelity SSB
    let mut pre_i_ssb  = Decimator::new(32, 6400.0 / PIPELINE_SAMPLE_RATE as f32, 127);
    let mut pre_q_ssb  = Decimator::new(32, 6400.0 / PIPELINE_SAMPLE_RATE as f32, 127);

    let mut dc_iq      = crate::dsp::DcRemover::new(0.01);
    let mut dc_audio   = crate::dsp::DcRemover::new(0.01);

    // ── Scratch buffers (allocated once, grown as needed) ────────────────────
    let mut i_buf: Vec<f32>          = Vec::new();
    let mut q_buf: Vec<f32>          = Vec::new();
    let mut i_dec: Vec<f32>          = Vec::new();
    let mut q_dec: Vec<f32>          = Vec::new();
    let mut iq_interleaved: Vec<f32> = Vec::new();
    let mut audio_out: Vec<f32>      = Vec::new();
    // Pre-allocated FFT buffer — avoids 8 KB heap alloc every frame
    let mut fft_buf: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); FFT_SIZE];

    // ── FFT ──────────────────────────────────────────────────────────────────
    let fft = FftPlanner::new().plan_fft_forward(FFT_SIZE);

    // ── Waterfall throttle ───────────────────────────────────────────────────
    let frame_interval = std::time::Duration::from_secs_f32(1.0 / DISPLAY_FPS);
    let mut last_frame = std::time::Instant::now() - frame_interval;
    let mut avg_pwr    = 0.0f32;
    let ema_alpha      = 0.05f32;

    loop {
        // Drain control messages before processing the next IQ block.
        while let Ok(mode) = demod_rx.try_recv() {
            current_mode = mode;
        }
        while let Ok(bw) = bandwidth_rx.try_recv() {
            let iq_bw      = bw as f32 * 2.0;
            let normalized = (iq_bw / PIPELINE_SAMPLE_RATE as f32).clamp(0.001, 0.499);
            pre_i_ssb.update_cutoff(normalized);
            pre_q_ssb.update_cutoff(normalized);
            pre_i_am.update_cutoff(normalized);
            pre_q_am.update_cutoff(normalized);
            info!("WebSDR filter BW → {} Hz (norm {:.4})", bw, normalized);
        }

        // Idle path — skip DSP when nobody is listening.
        if state.audio_tx.receiver_count() == 0 && state.waterfall_tx.receiver_count() == 0 {
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            // Still need to drain the broadcast so we don't fall far behind.
            let _ = sample_rx.try_recv();
            continue;
        }

        if current_mode == DemodMode::Off {
            let _ = sample_rx.try_recv();
            tokio::task::yield_now().await;
            continue;
        }

        // Receive next IQ block from broadcast pump.
        let raw_arc = match sample_rx.recv().await {
            Ok(b) => b,
            Err(broadcast::error::RecvError::Lagged(n)) => {
                log::warn!("WebSDR pipeline lagged, dropped {} blocks", n);
                continue;
            }
            Err(_) => break,
        };

        let raw: &[u8] = &raw_arc;
        let n = raw.len() / 2;
        if n == 0 {
            continue;
        }

        // ── u8 IQ → split f32 I/Q ────────────────────────────────────────────
        if iq_interleaved.len() != n * 2 {
            iq_interleaved.resize(n * 2, 0.0);
        }
        crate::converter::convert(raw, &mut iq_interleaved);

        if i_buf.len() != n { i_buf.resize(n, 0.0); }
        if q_buf.len() != n { q_buf.resize(n, 0.0); }
        for k in 0..n {
            i_buf[k] = iq_interleaved[k * 2];
            q_buf[k] = iq_interleaved[k * 2 + 1];
        }

        // DC removal — skip for AM which needs the carrier intact.
        if current_mode != DemodMode::Am {
            dc_iq.process_split(&mut i_buf, &mut q_buf);
        }

        // ── Waterfall / spectrum ──────────────────────────────────────────────
        let now = std::time::Instant::now();
        if now.duration_since(last_frame) >= frame_interval
            && n >= FFT_SIZE
            && state.waterfall_tx.receiver_count() > 0
        {
            last_frame = now;
            let offset = n - FFT_SIZE;
            for k in 0..FFT_SIZE {
                fft_buf[k] = Complex::new(i_buf[offset + k], q_buf[offset + k]);
            }
            fft.process(&mut fft_buf);

            let pwr_sum  = fft_buf.iter().map(|c| c.norm_sqr()).sum::<f32>();
            let inst_pwr = (pwr_sum / FFT_SIZE as f32).max(1e-12).log10() * 10.0;
            avg_pwr = if avg_pwr == 0.0 {
                inst_pwr
            } else {
                (1.0 - ema_alpha) * avg_pwr + ema_alpha * inst_pwr
            };

            let mut mag = vec![0u8; FFT_SIZE];
            for (i, m) in mag.iter_mut().enumerate() {
                let shifted  = (i + FFT_SIZE / 2) % FFT_SIZE;
                let bin_pwr  = fft_buf[shifted].norm_sqr().max(1e-12).log10() * 10.0;
                *m = ((bin_pwr - avg_pwr + 5.0) * 8.0).clamp(0.0, 255.0) as u8;
            }
            let _ = state.waterfall_tx.send(mag);
        }

        // ── Demodulation ──────────────────────────────────────────────────────
        // Helper: decimate I and Q, then re-interleave into iq_interleaved.
        macro_rules! decimate_iq {
            ($pre_i:expr, $pre_q:expr) => {{
                $pre_i.process_into(&i_buf, &mut i_dec);
                $pre_q.process_into(&q_buf, &mut q_dec);
                let len = i_dec.len().min(q_dec.len());
                iq_interleaved.resize(len * 2, 0.0);
                for k in 0..len {
                    iq_interleaved[k * 2]     = i_dec[k];
                    iq_interleaved[k * 2 + 1] = q_dec[k];
                }
                len
            }};
        }

        let mut audio: Vec<f32> = match current_mode {
            DemodMode::Wfm => {
                let len = decimate_iq!(pre_i_wfm, pre_q_wfm);
                let disc = fm_wfm.process(&iq_interleaved[..len * 2]);
                post_wfm.process_into(&disc, &mut audio_out);
                audio_out.clone()
            }
            DemodMode::Nfm => {
                let len = decimate_iq!(pre_i_nfm, pre_q_nfm);
                fm_nfm.process(&iq_interleaved[..len * 2])
            }
            DemodMode::Am => {
                let len = decimate_iq!(pre_i_am, pre_q_am);
                am.process(&iq_interleaved[..len * 2])
            }
            DemodMode::Usb => {
                let len = decimate_iq!(pre_i_ssb, pre_q_ssb);
                ssb_usb.process(&iq_interleaved[..len * 2])
            }
            DemodMode::Lsb => {
                let len = decimate_iq!(pre_i_ssb, pre_q_ssb);
                ssb_lsb.process(&iq_interleaved[..len * 2])
            }
            DemodMode::Off => continue,
        };

        // ── Post-demod processing ─────────────────────────────────────────────
        if !audio.is_empty() && state.audio_tx.receiver_count() > 0 {
            dc_audio.process_mono(&mut audio);
            audio_agc.process(&mut audio);
            let _ = state.audio_tx.send(Arc::new(audio));
        }
    }

    Ok(())
}
