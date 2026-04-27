use crate::Driver;
use crate::daemon::{HardwareBand, RetuneRequest};
use crate::dsp::{AmDemodulator, Decimator, FmDemodulator, Nco};
use axum::{
    Router,
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::IntoResponse,
    routing::get,
};
use axum_server::tls_rustls::RustlsConfig;
use futures_util::{SinkExt, StreamExt};
use log::{error, info, warn};
use parking_lot::RwLock;
use rustfft::{FftPlanner, num_complex::Complex};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::{Mutex, broadcast, mpsc};

// ── Constants ─────────────────────────────────────────────────────────────────

/// Hardware sample rate for the WebSDR pipeline.
/// 1_536_000 / 32 = 48_000 exactly — all decimation chains target 48 kHz audio.
pub const PIPELINE_SAMPLE_RATE: u32 = 1_536_000;

/// Audio sample rate delivered to browser clients.
pub const AUDIO_SAMPLE_RATE: u32 = 48_000;

const FFT_SIZE: usize = 1024;
const DISPLAY_FPS: f32 = 10.0;

// ── WebSocket protocol ────────────────────────────────────────────────────────

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum DemodMode {
    Wfm,
    Nfm,
    Am,
    Usb,
    Lsb,
    Off,
}

#[derive(serde::Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Command {
    Frequency { hz: u64 },
    Gain { db: f32 },
    Demod { mode: DemodMode },
    Bandwidth { hz: u32 },
    Squelch { db: f32 },
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
    AudioConfig {
        sample_rate: u32,
        fft_size: usize,
    },
    FrequencyChange {
        hz: u64,
    },
    GainChange {
        db: f32,
    },
}

// ── Server state ──────────────────────────────────────────────────────────────

pub struct WebSdrServer {
    driver: Arc<Mutex<Driver>>,
    /// Broadcast sender — each new WebSocket client calls `.subscribe()` for
    /// its own receiver. No relay task, no extra async hop.
    sample_tx: broadcast::Sender<Arc<Vec<u8>>>,
    band: Arc<RwLock<HardwareBand>>,
    retune_tx: mpsc::Sender<RetuneRequest>,
    sample_rate: u32,
}

impl WebSdrServer {
    /// Standalone entry point.
    pub async fn start(
        driver: Driver,
        addr: &str,
        tls: Option<(PathBuf, PathBuf)>,
    ) -> anyhow::Result<()> {
        let driver = Arc::new(Mutex::new(driver));

        {
            let mut d = driver.lock().await;
            info!("WebSDR: setting sample rate to {} Hz", PIPELINE_SAMPLE_RATE);
            if let Err(e) = d.set_sample_rate(PIPELINE_SAMPLE_RATE) {
                error!("CRITICAL: Failed to set sample rate: {:?}", e);
            }
            if d.frequency == 0 {
                let _ = d.set_frequency(101_100_000, None);
                let _ = d.tuner.set_gain(30.0);
            }
        }

        let (initial_freq, initial_inv) = {
            let d = driver.lock().await;
            let plan = d.orchestrator.plan_tuning(d.frequency);
            (d.frequency, plan.spectral_inv)
        };

        let (sample_tx, _) = broadcast::channel::<Arc<Vec<u8>>>(64);
        {
            let pump_driver = driver.clone();
            let pump_tx = sample_tx.clone();
            tokio::spawn(async move {
                let mut stream = {
                    let d = pump_driver.lock().await;
                    d.stream()
                };
                while let Some(res) = stream.next().await {
                    match res {
                        Ok(buf) => {
                            let _ = pump_tx.send(Arc::new(buf.to_vec()));
                        }
                        Err(e) => {
                            error!("WebSDR pump: {:?}", e);
                            break;
                        }
                    }
                }
            });
        }

        let band = Arc::new(RwLock::new(HardwareBand {
            center_hz: initial_freq,
            span_hz: PIPELINE_SAMPLE_RATE,
            spectral_inv: initial_inv,
        }));
        let (retune_tx, mut retune_rx) = mpsc::channel::<RetuneRequest>(8);
        {
            let retune_driver = driver.clone();
            let retune_band = band.clone();
            tokio::spawn(async move {
                while let Some(req) = retune_rx.recv().await {
                    let mut d = retune_driver.lock().await;
                    if let Ok(actual) = d.set_frequency(req.center_hz, Some(&retune_band)) {
                        info!("Standalone retune → {} Hz", actual);
                    }
                }
            });
        }

        let state = Arc::new(Self {
            driver,
            sample_tx,
            band,
            retune_tx,
            sample_rate: PIPELINE_SAMPLE_RATE,
        });
        Self::bind(state, addr, tls).await
    }

    /// Daemon entry point — accepts a `Sender` clone from the daemon pump.
    ///
    /// Each new WebSocket client calls `sample_tx.subscribe()` to get its own
    /// fresh `Receiver`. No relay task — eliminates one async hop and one
    /// broadcast ring buffer that could cause `Lagged` errors.
    pub async fn start_shared(
        driver: Arc<Mutex<Driver>>,
        sample_tx: broadcast::Sender<Arc<Vec<u8>>>,
        band: Arc<RwLock<HardwareBand>>,
        retune_tx: mpsc::Sender<RetuneRequest>,
        sample_rate: u32,
        addr: &str,
        tls: Option<(PathBuf, PathBuf)>,
    ) -> anyhow::Result<()> {
        let state = Arc::new(Self {
            driver,
            sample_tx,
            band,
            retune_tx,
            sample_rate,
        });
        Self::bind(state, addr, tls).await
    }

    async fn bind(
        state: Arc<Self>,
        addr: &str,
        tls: Option<(PathBuf, PathBuf)>,
    ) -> anyhow::Result<()> {
        let app = Router::new()
            .route("/", get(index_handler))
            .route("/ws", get(ws_handler))
            .route("/favicon.ico", get(favicon_handler))
            .with_state(state);

        if let Some((cert, key)) = tls {
            info!("WebSDR listening on https://{} (wss://)", addr);
            let cfg = RustlsConfig::from_pem_file(cert, key).await?;
            axum_server::bind_rustls(addr.parse()?, cfg)
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

// ── HTTP ──────────────────────────────────────────────────────────────────────

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

// ── Per-client connection handler ─────────────────────────────────────────────

async fn handle_socket(mut socket: WebSocket, state: Arc<WebSdrServer>) {
    let (info, hw_freq, gain) = {
        let d = state.driver.lock().await;
        (
            d.info.clone(),
            d.frequency,
            d.tuner.get_gain().unwrap_or(0.0),
        )
    };

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
            serde_json::json!({"type":"freqconfirm","requested":hw_freq,"actual":hw_freq})
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

    let (demod_tx, demod_rx) = broadcast::channel::<DemodMode>(4);
    let (bandwidth_tx, bandwidth_rx) = broadcast::channel::<u32>(4);
    let (squelch_tx, squelch_rx) = mpsc::channel::<f32>(4);
    let (freq_tx, freq_rx) = mpsc::channel::<u64>(4);

    let (mut sink, mut receiver) = socket.split();
    let (ws_tx, mut ws_rx) = mpsc::channel::<Message>(128);

    // Unified writer
    let mut sink_task = tokio::spawn(async move {
        while let Some(msg) = ws_rx.recv().await {
            if sink.send(msg).await.is_err() {
                break;
            }
        }
    });

    // Per-client pipeline — subscribes directly from the daemon/standalone pump sender
    let pipeline_ws_tx = ws_tx.clone();
    let pipeline_sample_rx = state.sample_tx.subscribe();
    let mut pipeline_task = tokio::spawn(run_client_pipeline(
        state.clone(),
        PipelineChannels {
            sample_rx: pipeline_sample_rx,
            demod_rx,
            bw_rx: bandwidth_rx,
            squelch_rx,
            freq_rx,
        },
        pipeline_ws_tx,
        hw_freq,
    ));

    // Command handler
    let cmd_driver = state.driver.clone();
    let cmd_state = state.clone();
    let cmd_reply_tx = ws_tx.clone();
    let mut cmd_task = tokio::spawn(async move {
        while let Some(Ok(Message::Text(text))) = receiver.next().await {
            let Ok(cmd) = serde_json::from_str::<Command>(&text) else {
                continue;
            };
            match cmd {
                Command::Frequency { hz } => {
                    if cmd_state.band.read().contains(hz) {
                        let _ = freq_tx.send(hz).await;
                    } else {
                        let _ = cmd_state
                            .retune_tx
                            .send(RetuneRequest { center_hz: hz })
                            .await;
                        let _ = freq_tx.send(hz).await;
                    }
                    let reply =
                        serde_json::json!({"type":"freqconfirm","requested":hz,"actual":hz});
                    let _ = cmd_reply_tx
                        .send(Message::Text(reply.to_string().into()))
                        .await;
                }
                Command::Gain { db } => {
                    let d = cmd_driver.lock().await;
                    let reply = match d.tuner.set_gain(db) {
                        Ok(a) => serde_json::json!({"type":"gainconfirm","actual":a}),
                        Err(e) => {
                            serde_json::json!({"type":"error","cmd":"setgain","msg":e.to_string()})
                        }
                    };
                    let _ = cmd_reply_tx
                        .send(Message::Text(reply.to_string().into()))
                        .await;
                }
                Command::Demod { mode } => {
                    let _ = demod_tx.send(mode);
                    let _ = cmd_reply_tx
                        .send(Message::Text(
                            serde_json::json!({"type":"demodconfirm","mode":mode})
                                .to_string()
                                .into(),
                        ))
                        .await;
                }
                Command::Bandwidth { hz } => {
                    let clamped = hz.clamp(500, 380_000);
                    let _ = bandwidth_tx.send(clamped);
                    let _ = cmd_reply_tx
                        .send(Message::Text(
                            serde_json::json!({"type":"bandwidthconfirm","actual":clamped})
                                .to_string()
                                .into(),
                        ))
                        .await;
                }
                Command::Squelch { db } => {
                    let _ = squelch_tx.send(db).await;
                    let _ = cmd_reply_tx
                        .send(Message::Text(
                            serde_json::json!({"type":"squelchconfirm","actual":db})
                                .to_string()
                                .into(),
                        ))
                        .await;
                }
            }
        }
    });

    tokio::select! {
        _ = &mut sink_task     => {}
        _ = &mut pipeline_task => {}
        _ = &mut cmd_task      => {}
    }
    sink_task.abort();
    pipeline_task.abort();
    cmd_task.abort();
}

// ── Per-client DSP state ─────────────────────────────────────────────────────

/// Consolidates all scratch buffers and stateful DSP components for one client.
pub struct DspState {
    pub nco: Nco,
    pub fm_wfm: FmDemodulator,
    pub fm_nfm: FmDemodulator,
    pub am: AmDemodulator,
    pub ssb_usb: crate::dsp::SsbDemodulator,
    pub ssb_lsb: crate::dsp::SsbDemodulator,
    pub audio_agc: crate::dsp::AudioAgc,
    pub pre_i_wfm: Decimator,
    pub pre_q_wfm: Decimator,
    pub post_wfm: Decimator,
    pub pre_i_nfm: Decimator,
    pub pre_q_nfm: Decimator,
    pub pre_i_am: Decimator,
    pub pre_q_am: Decimator,
    pub pre_i_ssb: Decimator,
    pub pre_q_ssb: Decimator,
    pub dc_iq: crate::dsp::DcRemover,
    pub dc_audio: crate::dsp::DcRemover,
    pub iq_buf: Vec<f32>,
    pub i_buf: Vec<f32>,
    pub q_buf: Vec<f32>,
    pub i_dec: Vec<f32>,
    pub q_dec: Vec<f32>,
    pub au_out: Vec<f32>,
    pub fft_buf: Vec<Complex<f32>>,
    pub squelch_db: f32,
    pub narrowband_pwr: f32,
}

impl DspState {
    pub fn new(sr: f32, fs: f64, initial_offset: f64) -> Self {
        Self {
            nco: Nco::new(initial_offset, fs),
            fm_wfm: FmDemodulator::new().with_deemphasis(sr / 8.0, 75e-6),
            fm_nfm: FmDemodulator::new(),
            am: AmDemodulator::new(),
            ssb_usb: crate::dsp::SsbDemodulator::new(true),
            ssb_lsb: crate::dsp::SsbDemodulator::new(false),
            audio_agc: crate::dsp::AudioAgc::new(
                0.17,
                0.1,
                0.001,
                500.0,
                AUDIO_SAMPLE_RATE as f32,
                0.02,
            ),
            pre_i_wfm: Decimator::new(8, 0.45 / 8.0, 31),
            pre_q_wfm: Decimator::new(8, 0.45 / 8.0, 31),
            post_wfm: Decimator::new(4, 0.45 / 4.0, 65),
            pre_i_nfm: Decimator::new(32, 0.45 / 32.0, 31),
            pre_q_nfm: Decimator::new(32, 0.45 / 32.0, 31),
            pre_i_am: Decimator::new(32, 6000.0 / sr, 63),
            pre_q_am: Decimator::new(32, 6000.0 / sr, 63),
            pre_i_ssb: Decimator::new(32, 6400.0 / sr, 127),
            pre_q_ssb: Decimator::new(32, 6400.0 / sr, 127),
            dc_iq: crate::dsp::DcRemover::new(0.01),
            dc_audio: crate::dsp::DcRemover::new(0.01),
            iq_buf: Vec::with_capacity(32768),
            i_buf: Vec::with_capacity(16384),
            q_buf: Vec::with_capacity(16384),
            i_dec: Vec::with_capacity(16384),
            q_dec: Vec::with_capacity(16384),
            au_out: Vec::with_capacity(16384),
            fft_buf: vec![Complex::new(0.0, 0.0); FFT_SIZE],
            squelch_db: -80.0,
            narrowband_pwr: -100.0,
        }
    }

    pub fn reset(&mut self) {
        self.pre_i_wfm.reset();
        self.pre_q_wfm.reset();
        self.post_wfm.reset();
        self.pre_i_nfm.reset();
        self.pre_q_nfm.reset();
        self.pre_i_am.reset();
        self.pre_q_am.reset();
        self.pre_i_ssb.reset();
        self.pre_q_ssb.reset();
        self.dc_iq = crate::dsp::DcRemover::new(0.01);
        self.nco.reset();
    }
}

struct PipelineChannels {
    sample_rx: broadcast::Receiver<Arc<Vec<u8>>>,
    demod_rx: broadcast::Receiver<DemodMode>,
    bw_rx: broadcast::Receiver<u32>,
    squelch_rx: mpsc::Receiver<f32>,
    freq_rx: mpsc::Receiver<u64>,
}

/// One instance per WebSocket connection. Owns all DSP state for this client.
///
/// # On Lagged errors
/// If the broadcast receiver falls behind and drops blocks, all FIR decimator
/// history buffers are reset. The transient is short (a few FIR tap-lengths of
/// audio, ~1–2 ms) and far less destructive than continuing with stale history
/// which produces sustained garbage output.
async fn run_client_pipeline(
    state: Arc<WebSdrServer>,
    channels: PipelineChannels,
    ws_tx: mpsc::Sender<Message>,
    initial_freq: u64,
) {
    let PipelineChannels {
        mut sample_rx,
        mut demod_rx,
        mut bw_rx,
        mut squelch_rx,
        mut freq_rx,
    } = channels;

    let fs = state.sample_rate as f64;
    let sr = state.sample_rate as f32;

    let initial_offset = state.band.read().offset_hz(initial_freq) as f64;
    let mut dsp = DspState::new(sr, fs, initial_offset);
    let mut client_freq = initial_freq;
    let mut last_confirmed = initial_freq;

    let fft = FftPlanner::new().plan_fft_forward(FFT_SIZE);
    let frame_iv = std::time::Duration::from_secs_f32(1.0 / DISPLAY_FPS);
    let mut last_frame = std::time::Instant::now() - frame_iv;
    let mut avg_pwr = 0.0f32;
    let ema = 0.05f32;
    let mut mode = DemodMode::Wfm;

    loop {
        // ── Drain control messages ────────────────────────────────────────────
        while let Ok(f) = freq_rx.try_recv() {
            client_freq = f;
        }
        while let Ok(m) = demod_rx.try_recv() {
            mode = m;
        }
        while let Ok(sq) = squelch_rx.try_recv() {
            dsp.squelch_db = sq;
        }
        while let Ok(bw) = bw_rx.try_recv() {
            let norm = (bw as f32 / sr).clamp(0.001, 0.499);
            dsp.pre_i_ssb.update_cutoff(norm);
            dsp.pre_q_ssb.update_cutoff(norm);
            dsp.pre_i_am.update_cutoff(norm);
            dsp.pre_q_am.update_cutoff(norm);
        }

        {
            let offset = state.band.read().offset_hz(client_freq) as f64;
            dsp.nco.set_freq(offset, fs);
        }

        if mode == DemodMode::Off {
            let _ = sample_rx.try_recv();
            tokio::task::yield_now().await;
            continue;
        }

        let raw = match sample_rx.recv().await {
            Ok(b) => b,
            Err(broadcast::error::RecvError::Lagged(n)) => {
                warn!("Pipeline lagged, dropped {} blocks — resetting state", n);
                dsp.reset();
                continue;
            }
            Err(_) => break,
        };

        let n = raw.len() / 2;
        if n == 0 {
            info!("Pipeline flush signal received — resetting state");
            dsp.reset();
            continue;
        }

        // ── Processing ────────────────────────────────────────────────────────
        if dsp.iq_buf.len() != n * 2 {
            dsp.iq_buf.resize(n * 2, 0.0);
        }
        crate::converter::convert(&raw, &mut dsp.iq_buf);
        dsp.nco.mix(&mut dsp.iq_buf);

        if dsp.i_buf.len() != n {
            dsp.i_buf.resize(n, 0.0);
        }
        if dsp.q_buf.len() != n {
            dsp.q_buf.resize(n, 0.0);
        }
        for k in 0..n {
            dsp.i_buf[k] = dsp.iq_buf[k * 2];
            dsp.q_buf[k] = dsp.iq_buf[k * 2 + 1];
        }
        if mode != DemodMode::Am {
            dsp.dc_iq.process_split(&mut dsp.i_buf, &mut dsp.q_buf);
        }

        // ── Waterfall ─────────────────────────────────────────────────────────
        let now = std::time::Instant::now();
        if now.duration_since(last_frame) >= frame_iv && n >= FFT_SIZE {
            last_frame = now;
            let off = n - FFT_SIZE;
            for k in 0..FFT_SIZE {
                dsp.fft_buf[k] = Complex::new(dsp.i_buf[off + k], dsp.q_buf[off + k]);
            }
            fft.process(&mut dsp.fft_buf);
            let pwr = dsp.fft_buf.iter().map(|c| c.norm_sqr()).sum::<f32>();
            let ipwr = (pwr / FFT_SIZE as f32).max(1e-12).log10() * 10.0;
            avg_pwr = if avg_pwr == 0.0 {
                ipwr
            } else {
                (1.0 - ema) * avg_pwr + ema * ipwr
            };
            let mut mag = vec![0u8; FFT_SIZE];
            let inv = state.band.read().spectral_inv;
            for (i, m) in mag.iter_mut().enumerate() {
                let s = if inv {
                    (FFT_SIZE - i + FFT_SIZE / 2) % FFT_SIZE
                } else {
                    (i + FFT_SIZE / 2) % FFT_SIZE
                };
                let bp = dsp.fft_buf[s].norm_sqr().max(1e-12).log10() * 10.0;
                *m = ((bp - avg_pwr + 5.0) * 8.0).clamp(0.0, 255.0) as u8;
            }
            let mut wf = Vec::with_capacity(1 + FFT_SIZE);
            wf.push(b'W');
            wf.extend_from_slice(&mag);
            let _ = ws_tx.send(Message::Binary(wf.into())).await;
        }

        // ── Demodulation ──────────────────────────────────────────────────────
        macro_rules! dec {
            ($pi:expr, $pq:expr) => {{
                $pi.process_into(&dsp.i_buf, &mut dsp.i_dec);
                $pq.process_into(&dsp.q_buf, &mut dsp.q_dec);
                let len = dsp.i_dec.len().min(dsp.q_dec.len());
                dsp.iq_buf.resize(len * 2, 0.0);
                for k in 0..len {
                    dsp.iq_buf[k * 2] = dsp.i_dec[k];
                    dsp.iq_buf[k * 2 + 1] = dsp.q_dec[k];
                }
                len
            }};
        }

        let mut audio: Vec<f32> = match mode {
            DemodMode::Wfm => {
                let len = dec!(dsp.pre_i_wfm, dsp.pre_q_wfm);
                let disc = dsp.fm_wfm.process(&dsp.iq_buf[..len * 2]);
                dsp.post_wfm.process_into(&disc, &mut dsp.au_out);
                dsp.au_out.clone()
            }
            DemodMode::Nfm => {
                let len = dec!(dsp.pre_i_nfm, dsp.pre_q_nfm);
                dsp.fm_nfm.process(&dsp.iq_buf[..len * 2])
            }
            DemodMode::Am => {
                let len = dec!(dsp.pre_i_am, dsp.pre_q_am);
                dsp.am.process(&dsp.iq_buf[..len * 2])
            }
            DemodMode::Usb => {
                let len = dec!(dsp.pre_i_ssb, dsp.pre_q_ssb);
                dsp.ssb_usb.process(&dsp.iq_buf[..len * 2])
            }
            DemodMode::Lsb => {
                let len = dec!(dsp.pre_i_ssb, dsp.pre_q_ssb);
                dsp.ssb_lsb.process(&dsp.iq_buf[..len * 2])
            }
            DemodMode::Off => continue,
        };

        // ── Post-demod audio ──────────────────────────────────────────────────
        if !audio.is_empty() {
            // Calculate narrowband power (dBFS)
            let pwr = audio.iter().map(|&s| s * s).sum::<f32>() / audio.len() as f32;
            let dbfs = if pwr > 1e-12 {
                10.0 * pwr.log10()
            } else {
                -100.0
            };

            // Apply hysteresis (EMA)
            dsp.narrowband_pwr = 0.05 * dbfs + 0.95 * dsp.narrowband_pwr;

            // Apply RSSI Squelch
            if dsp.narrowband_pwr < dsp.squelch_db {
                audio.fill(0.0);
            } else {
                dsp.dc_audio.process_mono(&mut audio);
                dsp.audio_agc.process(&mut audio);
            }

            let mut au_msg = Vec::with_capacity(4 + audio.len() * 4);
            au_msg.extend_from_slice(&[b'A', 0, 0, 0]);
            for &s in &audio {
                au_msg.extend_from_slice(&s.to_le_bytes());
            }
            if ws_tx.send(Message::Binary(au_msg.into())).await.is_err() {
                break;
            }
        }

        // ── Freq confirm — only on change ─────────────────────────────────────
        if client_freq != last_confirmed {
            let hw_center = state.band.read().center_hz;
            let confirm = serde_json::json!({
                "type": "freqconfirm", "requested": client_freq, "actual": hw_center,
            });
            if ws_tx
                .send(Message::Text(confirm.to_string().into()))
                .await
                .is_err()
            {
                break;
            }
            last_confirmed = client_freq;
        }
    }
}
