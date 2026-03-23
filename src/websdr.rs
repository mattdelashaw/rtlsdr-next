use crate::Driver;
use crate::dsp::{AmDemodulator, Decimator, FmDemodulator};
use axum::{
    Router,
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::IntoResponse,
    routing::get,
};
use futures_util::{SinkExt, StreamExt};
use log::{error, info};
use rustfft::{FftPlanner, num_complex::Complex};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::{Mutex, broadcast};

// ── Constants ────────────────────────────────────────────────────────────────

/// Driver sample rate — chosen so AUDIO_DECIMATION divides it exactly to 48kHz.
/// 1.536M / 32 = 48,000 exactly. All decimation chains target this.
pub const PIPELINE_SAMPLE_RATE: u32 = 1_536_000;

/// Audio sample rate delivered to clients.
pub const AUDIO_SAMPLE_RATE: u32 = 48_000;

/// FFT size for spectrum/waterfall.
const FFT_SIZE: usize = 1024;

/// Target display frame rate for waterfall/spectrum.
const DISPLAY_FPS: f32 = 10.0;

// ── WebSocket Protocol ────────────────────────────────────────────────────────

/// Demod modes — matches iOS client `DemodMode` enum (lowercase JSON tags).
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(rename_all = "lowercase")]
enum DemodMode {
    Wfm, // Wideband FM — broadcast, ~200kHz
    Nfm, // Narrowband FM — GMRS/ham, ~12.5kHz
    Am,  // Amplitude modulation — aircraft, AM broadcast
    Usb, // Upper sideband (stub — requires Hilbert SSB in dsp.rs)
    Lsb, // Lower sideband (stub — requires Hilbert SSB in dsp.rs)
    Off,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type", rename_all = "lowercase")]
enum Command {
    Frequency { hz: u64 },
    Gain { db: f32 },
    Demod { mode: DemodMode },
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
    FrequencyChange {
        hz: u64,
    },
    GainChange {
        db: f32,
    },
}

// ── Server State ──────────────────────────────────────────────────────────────

pub struct WebSdrServer {
    driver: Arc<Mutex<Driver>>,
    /// Waterfall row: FFT_SIZE magnitude bytes per message (u8, 0-255).
    waterfall_tx: broadcast::Sender<Vec<u8>>,
    /// Audio PCM at AUDIO_SAMPLE_RATE, wrapped in Arc to avoid copies on broadcast.
    audio_tx: broadcast::Sender<Arc<Vec<f32>>>,
    demod_tx: broadcast::Sender<DemodMode>,
}

impl WebSdrServer {
    pub async fn start(driver: Driver, addr: &str) -> anyhow::Result<()> {
        let driver = Arc::new(Mutex::new(driver));

        let (waterfall_tx, _) = broadcast::channel(16);
        let (audio_tx, _) = broadcast::channel(16);
        let (demod_tx, _) = broadcast::channel(16);

        let state = Arc::new(Self {
            driver: driver.clone(),
            waterfall_tx,
            audio_tx,
            demod_tx,
        });

        {
            let mut d = driver.lock().await;
            if d.frequency == 0 {
                info!(
                    "WebSDR init: 101.1 MHz / 30 dB / {} kSPS",
                    PIPELINE_SAMPLE_RATE / 1000
                );
                let _ = d.set_sample_rate(PIPELINE_SAMPLE_RATE);
                let _ = d.set_frequency(101_100_000);
                let _ = d.tuner.set_gain(30.0);
            }
        }

        let pipeline_state = state.clone();
        tokio::spawn(async move {
            if let Err(e) = run_pipeline(pipeline_state).await {
                error!("WebSDR pipeline error: {:?}", e);
            }
        });

        let app = Router::new()
            .route("/", get(index_handler))
            .route("/ws", get(ws_handler))
            .route("/favicon.ico", get(favicon_handler))
            .with_state(state);

        let listener = tokio::net::TcpListener::bind(addr).await?;
        info!("WebSDR listening on http://{}", addr);
        axum::serve(listener, app).await?;
        Ok(())
    }
}

// ── HTTP ──────────────────────────────────────────────────────────────────────

async fn index_handler() -> impl IntoResponse {
    axum::response::Html(include_str!("../assets/websdr_ui.html"))
}

async fn favicon_handler() -> impl IntoResponse {
    match tokio::fs::read("assets/favicon.ico").await {
        Ok(bytes) => (
            [(axum::http::header::CONTENT_TYPE, "image/x-icon")],
            axum::response::Response::new(axum::body::Body::from(bytes)),
        ),
        Err(_) => (
            [(axum::http::header::CONTENT_TYPE, "text/plain")],
            axum::response::Response::builder()
                .status(axum::http::StatusCode::NOT_FOUND)
                .body(axum::body::Body::from("Favicon not found"))
                .unwrap(),
        ),
    }
}

// ── WebSocket ─────────────────────────────────────────────────────────────────

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

    // Handshake: hardware info + audio config + current state
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

    // Unified writer task
    let mut sink_task = tokio::spawn(async move {
        while let Some(msg) = ws_rx.recv().await {
            if sink.send(msg).await.is_err() {
                break;
            }
        }
    });

    // Waterfall push: binary 'W' + FFT_SIZE bytes
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

    // Audio push: binary 'A' + 3-byte padding + f32 LE PCM
    let au_tx = ws_tx.clone();
    let mut au_task = tokio::spawn(async move {
        while let Ok(data) = audio_rx.recv().await {
            let mut msg = Vec::with_capacity(4 + data.len() * 4);
            msg.extend_from_slice(&[b'A', 0, 0, 0]); // header + padding for Float32Array alignment
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
            }
        }
    });

    tokio::select! {
        _ = &mut sink_task => {},
        _ = &mut wf_task   => {},
        _ = &mut au_task   => {},
        _ = &mut cmd_task  => {},
    }
    sink_task.abort();
    wf_task.abort();
    au_task.abort();
    cmd_task.abort();
}

// ── Pipeline ──────────────────────────────────────────────────────────────────

async fn run_pipeline(state: Arc<WebSdrServer>) -> anyhow::Result<()> {
    // Raw stream — all decimation is explicit here for exact rate control.
    // Driver runs at PIPELINE_SAMPLE_RATE (1.536M).
    let mut stream = {
        let d = state.driver.lock().await;
        d.stream()
    };

    let mut current_mode = DemodMode::Wfm;
    let mut demod_rx = state.demod_tx.subscribe();

    // ── Demodulators ────────────────────────────────────────────────────────
    // de-emphasis at the 192kHz intermediate rate before post-decimation
    let intermediate_rate_wfm = PIPELINE_SAMPLE_RATE as f32 / 8.0; // 192kHz
    let mut fm_wfm = FmDemodulator::new().with_deemphasis(intermediate_rate_wfm, 75e-6);
    let mut fm_nfm = FmDemodulator::new();
    let mut am = AmDemodulator::new();

    // ── Decimation chains ────────────────────────────────────────────────────
    // WFM:  ÷8 → 192kHz → discriminate → ÷4 → 48kHz
    // NFM:  ÷16 → 96kHz → discriminate → ÷2 → 48kHz
    // AM:   ÷16 → 96kHz → envelope → ÷2 → 48kHz
    // SSB:  stub (÷32 → 48kHz direct, needs Hilbert filter in dsp.rs)
    let mut pre_i_wfm = Decimator::new(8, 0.45 / 8.0, 31);
    let mut pre_q_wfm = Decimator::new(8, 0.45 / 8.0, 31);
    let mut post_wfm = Decimator::new(4, 0.45 / 4.0, 65);

    let mut pre_i_nfm = Decimator::new(16, 0.45 / 16.0, 31);
    let mut pre_q_nfm = Decimator::new(16, 0.45 / 16.0, 31);
    let mut post_nfm = Decimator::new(2, 0.45 / 2.0, 65);

    let mut pre_i_am = Decimator::new(16, 0.45 / 16.0, 31);
    let mut pre_q_am = Decimator::new(16, 0.45 / 16.0, 31);
    let mut post_am = Decimator::new(2, 0.45 / 2.0, 65);

    let mut dc = crate::dsp::DcRemover::new(0.001);

    // ── Scratch buffers (allocated once) ────────────────────────────────────
    let mut i_buf: Vec<f32> = Vec::new();
    let mut q_buf: Vec<f32> = Vec::new();
    let mut i_dec: Vec<f32> = Vec::new();
    let mut q_dec: Vec<f32> = Vec::new();
    let mut iq_interleaved: Vec<f32> = Vec::new();
    let _demod_out: Vec<f32> = Vec::new();
    let mut audio_out: Vec<f32> = Vec::new();
    // Pre-allocated FFT buffer — avoids 8KB heap alloc every frame
    let mut fft_buf: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); FFT_SIZE];

    // ── FFT ──────────────────────────────────────────────────────────────────
    let fft = FftPlanner::new().plan_fft_forward(FFT_SIZE);

    // ── Waterfall throttle ───────────────────────────────────────────────────
    let frame_interval = std::time::Duration::from_secs_f32(1.0 / DISPLAY_FPS);
    let mut last_frame = std::time::Instant::now() - frame_interval;
    let mut avg_pwr = 0.0f32;
    let ema_alpha = 0.05f32;

    while let Some(res) = stream.next().await {
        while let Ok(mode) = demod_rx.try_recv() {
            current_mode = mode;
        }
        if current_mode == DemodMode::Off {
            continue;
        }

        let raw_buf = match res {
            Ok(b) => b,
            Err(e) => {
                error!("Stream error: {:?}", e);
                break;
            }
        };
        let raw: &[u8] = &raw_buf;
        let n = raw.len() / 2;
        if n == 0 {
            continue;
        }

        // ── u8 IQ → split f32 I/Q ───────────────────────────────────────────
        // converter::convert produces interleaved f32 — split in one pass.
        if iq_interleaved.len() != n * 2 {
            iq_interleaved.resize(n * 2, 0.0);
        }
        crate::converter::convert(raw, &mut iq_interleaved);

        if i_buf.len() != n {
            i_buf.resize(n, 0.0);
        }
        if q_buf.len() != n {
            q_buf.resize(n, 0.0);
        }
        for k in 0..n {
            i_buf[k] = iq_interleaved[k * 2];
            q_buf[k] = iq_interleaved[k * 2 + 1];
        }

        // DC removal on split buffers
        dc.process_split(&mut i_buf, &mut q_buf);

        // ── Spectrum / Waterfall ────────────────────────────────────────────
        let now = std::time::Instant::now();
        if now.duration_since(last_frame) >= frame_interval && n >= FFT_SIZE {
            last_frame = now;
            let offset = n - FFT_SIZE;
            for k in 0..FFT_SIZE {
                fft_buf[k] = Complex::new(i_buf[offset + k], q_buf[offset + k]);
            }
            fft.process(&mut fft_buf);

            let pwr_sum: f32 = fft_buf.iter().map(|c| c.norm_sqr()).sum();
            let inst_pwr = (pwr_sum / FFT_SIZE as f32).max(1e-12).log10() * 10.0;
            avg_pwr = if avg_pwr == 0.0 {
                inst_pwr
            } else {
                (1.0 - ema_alpha) * avg_pwr + ema_alpha * inst_pwr
            };

            let mut mag = vec![0u8; FFT_SIZE];
            for (i, m) in mag.iter_mut().enumerate().take(FFT_SIZE) {
                let shifted = (i + FFT_SIZE / 2) % FFT_SIZE;
                let bin_pwr = fft_buf[shifted].norm_sqr().max(1e-12).log10() * 10.0;
                *m = ((bin_pwr - avg_pwr + 5.0) * 8.0).clamp(0.0, 255.0) as u8;
            }
            let _ = state.waterfall_tx.send(mag);
        }

        // ── Demodulation ────────────────────────────────────────────────────
        let audio: Vec<f32> = match current_mode {
            DemodMode::Wfm => {
                pre_i_wfm.process_into(&i_buf, &mut i_dec);
                pre_q_wfm.process_into(&q_buf, &mut q_dec);
                let len = i_dec.len().min(q_dec.len());
                iq_interleaved.resize(len * 2, 0.0);
                for k in 0..len {
                    iq_interleaved[k * 2] = i_dec[k];
                    iq_interleaved[k * 2 + 1] = q_dec[k];
                }
                let disc = fm_wfm.process(&iq_interleaved[..len * 2]);
                post_wfm.process_into(&disc, &mut audio_out);
                audio_out.clone()
            }
            DemodMode::Nfm => {
                pre_i_nfm.process_into(&i_buf, &mut i_dec);
                pre_q_nfm.process_into(&q_buf, &mut q_dec);
                let len = i_dec.len().min(q_dec.len());
                iq_interleaved.resize(len * 2, 0.0);
                for k in 0..len {
                    iq_interleaved[k * 2] = i_dec[k];
                    iq_interleaved[k * 2 + 1] = q_dec[k];
                }
                let disc = fm_nfm.process(&iq_interleaved[..len * 2]);
                post_nfm.process_into(&disc, &mut audio_out);
                audio_out.clone()
            }
            DemodMode::Am => {
                pre_i_am.process_into(&i_buf, &mut i_dec);
                pre_q_am.process_into(&q_buf, &mut q_dec);
                let len = i_dec.len().min(q_dec.len());
                iq_interleaved.resize(len * 2, 0.0);
                for k in 0..len {
                    iq_interleaved[k * 2] = i_dec[k];
                    iq_interleaved[k * 2 + 1] = q_dec[k];
                }
                let env = am.process(&iq_interleaved[..len * 2]);
                post_am.process_into(&env, &mut audio_out);
                audio_out.clone()
            }
            // TODO: USB/LSB — needs HilbertFilter in dsp.rs (port from iOS SSBDemodulator)
            DemodMode::Usb | DemodMode::Lsb => vec![],
            DemodMode::Off => continue,
        };

        if !audio.is_empty() {
            let _ = state.audio_tx.send(Arc::new(audio));
        }
    }

    Ok(())
}
