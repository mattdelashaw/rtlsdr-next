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

// ── WebSocket Protocol ──────────────────────────────────────────────────────

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(rename_all = "lowercase")]
enum DemodMode {
    Fm,
    Am,
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
    FrequencyChange {
        hz: u64,
    },
    GainChange {
        db: f32,
    },
}

// ── Server State ─────────────────────────────────────────────────────────────

pub struct WebSdrServer {
    driver: Arc<Mutex<Driver>>,
    waterfall_tx: broadcast::Sender<Vec<u8>>, // magnitude bytes (0-255)
    audio_tx: broadcast::Sender<Vec<f32>>,    // f32 audio samples
    demod_tx: broadcast::Sender<DemodMode>,
}

impl WebSdrServer {
    pub async fn start(driver: Driver, addr: &str) -> anyhow::Result<()> {
        let driver = Arc::new(Mutex::new(driver));

        // Broadcast channels for waterfall and audio
        let (waterfall_tx, _) = broadcast::channel(16);
        let (audio_tx, _) = broadcast::channel(16);
        let (demod_tx, _) = broadcast::channel(16);

        let state = Arc::new(Self {
            driver: driver.clone(),
            waterfall_tx: waterfall_tx.clone(),
            audio_tx: audio_tx.clone(),
            demod_tx: demod_tx.clone(),
        });

        // 1. Initialize Hardware (Default to 100 MHz + 30dB if uninitialized)
        {
            let mut d = driver.lock().await;
            if d.frequency == 0 {
                info!("Initializing WebSDR hardware at 100.0 MHz / 30.0 dB...");
                let _ = d.set_frequency(100_000_000);
                let _ = d.tuner.set_gain(30.0);
            }
        }

        // 2. Hardware Pipeline Task (FFT + Demod)
        let pipeline_state = state.clone();
        tokio::spawn(async move {
            if let Err(e) = run_pipeline(pipeline_state).await {
                error!("WebSDR Pipeline Error: {:?}", e);
            }
        });

        // 2. Web Server
        let app = Router::new()
            .route("/", get(index_handler))
            .route("/ws", get(ws_handler))
            .route("/favicon.ico", get(favicon_handler))
            .with_state(state);

        let listener = tokio::net::TcpListener::bind(addr).await?;
        info!("WebSDR Backend listening on http://{}", addr);
        axum::serve(listener, app).await?;

        Ok(())
    }
}

// ── HTTP Handlers ────────────────────────────────────────────────────────────

async fn index_handler() -> impl IntoResponse {
    axum::response::Html(include_str!("../assets/websdr_ui.html"))
}

async fn favicon_handler() -> impl IntoResponse {
    // We try to find the favicon in the assets directory.
    // If it doesn't exist, we'll return a 404.
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

// ── WebSocket Handler ────────────────────────────────────────────────────────

async fn ws_handler(
    ws: WebSocketUpgrade,
    axum::extract::State(state): axum::extract::State<Arc<WebSdrServer>>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_socket(socket, state))
}

async fn handle_socket(mut socket: WebSocket, state: Arc<WebSdrServer>) {
    // Send initial hardware info and state
    let (info, freq, gain) = {
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

    // Sync UI with current tuner state
    let _ = socket
        .send(Message::Text(
            serde_json::json!({
                "type": "freqconfirm",
                "requested": freq,
                "actual": freq,
            })
            .to_string()
            .into(),
        ))
        .await;

    let _ = socket
        .send(Message::Text(
            serde_json::json!({
                "type": "gainconfirm",
                "actual": gain,
            })
            .to_string()
            .into(),
        ))
        .await;

    let mut waterfall_rx = state.waterfall_tx.subscribe();
    let mut audio_rx = state.audio_tx.subscribe();

    let (mut sink, mut receiver) = socket.split();
    let (ws_send_tx, mut ws_send_rx) = tokio::sync::mpsc::channel::<Message>(128);

    // Task: Unified WebSocket Sink
    let mut sink_task = tokio::spawn(async move {
        while let Some(msg) = ws_send_rx.recv().await {
            if sink.send(msg).await.is_err() {
                break;
            }
        }
    });

    // Task: Push waterfall (binary)
    let waterfall_tx = ws_send_tx.clone();
    let mut waterfall_task = tokio::spawn(async move {
        while let Ok(data) = waterfall_rx.recv().await {
            let mut msg = vec![b'W'];
            msg.extend_from_slice(&data);
            if waterfall_tx
                .send(Message::Binary(msg.into()))
                .await
                .is_err()
            {
                break;
            }
        }
    });

    // Task: Push audio (binary)
    let audio_tx = ws_send_tx.clone();
    let mut audio_task = tokio::spawn(async move {
        while let Ok(data) = audio_rx.recv().await {
            // Use 4-byte header to maintain 4-byte alignment for Float32Array in the browser.
            // [0] = 'A', [1,2,3] = padding, [4..] = f32 samples
            let mut msg = Vec::with_capacity(4 + data.len() * 4);
            msg.push(b'A');

            msg.push(0);
            msg.push(0);
            msg.push(0);
            for &sample in data.iter() {
                msg.extend_from_slice(&sample.to_le_bytes());
            }
            if audio_tx.send(Message::Binary(msg.into())).await.is_err() {
                break;
            }
        }
    });

    // Task: Process Commands
    let cmd_driver = state.driver.clone();
    let cmd_reply_tx = ws_send_tx.clone();
    let mut cmd_task = tokio::spawn(async move {
        while let Some(Ok(Message::Text(text))) = receiver.next().await {
            if let Ok(cmd) = serde_json::from_str::<Command>(&text) {
                let mut d = cmd_driver.lock().await;
                match cmd {
                    Command::Frequency { hz } => match d.set_frequency(hz) {
                        Ok(actual) => {
                            let reply = serde_json::json!({
                                "type": "freqconfirm",
                                "requested": hz,
                                "actual": actual,
                            });
                            let _ = cmd_reply_tx
                                .send(Message::Text(reply.to_string().into()))
                                .await;
                        }
                        Err(e) => {
                            let reply = serde_json::json!({
                                "type": "error",
                                "cmd": "setfrequency",
                                "msg": e.to_string(),
                            });
                            let _ = cmd_reply_tx
                                .send(Message::Text(reply.to_string().into()))
                                .await;
                        }
                    },
                    Command::Gain { db } => match d.tuner.set_gain(db) {
                        Ok(actual) => {
                            let reply = serde_json::json!({
                                "type": "gainconfirm",
                                "actual": actual,
                            });
                            let _ = cmd_reply_tx
                                .send(Message::Text(reply.to_string().into()))
                                .await;
                        }
                        Err(e) => {
                            let reply = serde_json::json!({
                                "type": "error",
                                "cmd": "setgain",
                                "msg": e.to_string(),
                            });
                            let _ = cmd_reply_tx
                                .send(Message::Text(reply.to_string().into()))
                                .await;
                        }
                    },
                    Command::Demod { mode } => {
                        let _ = state.demod_tx.send(mode);
                        let reply = serde_json::json!({
                            "type": "demodconfirm",
                            "actual": mode,
                        });
                        let _ = cmd_reply_tx
                            .send(Message::Text(reply.to_string().into()))
                            .await;
                    }
                }
            }
        }
    });

    tokio::select! {
        _ = &mut sink_task => {},
        _ = &mut waterfall_task => {},
        _ = &mut audio_task => {},
        _ = &mut cmd_task => {},
    }
    sink_task.abort();
    waterfall_task.abort();
    audio_task.abort();
    cmd_task.abort();
}

// ── The Pipeline (FFT & Demod) ─────────────────────────────────────────────

async fn run_pipeline(state: Arc<WebSdrServer>) -> anyhow::Result<()> {
    let mut stream = {
        let d = state.driver.lock().await;
        let stream: crate::stream::F32Stream<rusb::Context> = d.stream_f32(4);
        stream.with_dc_removal(0.01).with_agc(2.0, 0.01, 0.01)
    };

    let mut fm = FmDemodulator::new().with_deemphasis(512_000.0, 75e-6);
    let mut am = AmDemodulator::new();
    let mut current_mode = DemodMode::Fm;
    let mut demod_rx = state.demod_tx.subscribe();

    let mut planner = FftPlanner::new();
    let fft_size = 1024;
    let fft = planner.plan_fft_forward(fft_size);

    // Audio decimation: 512k -> 48k (factor 10.66... not integer, use approx 11)
    // Or simpler: change baseband to 480k? No, let's just use 48k output by
    // adjusting the decimation. 512 / 10.66 = 48.
    // We'll use Decimator::new(factor, cutoff, taps).
    // To get exactly 48k from 2048k, we need factor 42.66.
    // Better: change Driver::stream_f32(4) to a factor that divides 2048 into a mult of 48.
    // 2048 / 4 = 512.
    // Let's use 2.4 MSPS instead? No, V4 is 2.048.
    // Let's keep 51.2k for now but use 100 taps for high-fidelity.
    let mut audio_decimator = Decimator::new(10, 0.05, 101); // 512k -> 51k

    // Dynamic noise floor tracking for waterfall
    let mut avg_pwr = 0.0f32;
    let alpha = 0.05f32; // EMA smoothing

    while let Some(res) = stream.next().await {
        // Check for demod mode changes
        while let Ok(mode) = demod_rx.try_recv() {
            current_mode = mode;
        }

        let iq_pooled = res?;
        let iq = &*iq_pooled;

        // 1. FFT for Waterfall
        if iq.len() >= fft_size * 2 {
            let mut buffer: Vec<Complex<f32>> = (0..fft_size)
                .map(|i| Complex::new(iq[i * 2], iq[i * 2 + 1]))
                .collect();

            fft.process(&mut buffer);

            // Calculate instantaneous power for noise floor tracking
            let mut pwr_sum = 0.0;
            for c in buffer.iter() {
                pwr_sum += c.norm_sqr();
            }
            let inst_pwr = (pwr_sum / fft_size as f32).log10() * 10.0;
            if avg_pwr == 0.0 {
                avg_pwr = inst_pwr;
            } else {
                avg_pwr = (1.0 - alpha) * avg_pwr + alpha * inst_pwr;
            }

            // Shift (center DC) and convert to magnitude bytes using relative power
            let mut mag = vec![0u8; fft_size];
            for (i, m) in mag.iter_mut().enumerate().take(fft_size) {
                let shifted = (i + fft_size / 2) % fft_size;
                let bin_pwr = (buffer[shifted].norm_sqr().max(1e-12)).log10() * 10.0;

                // Scale signal relative to moving average noise floor
                // Map [avg_pwr, avg_pwr + 30dB] -> [0, 255]
                let val = ((bin_pwr - avg_pwr + 5.0) * 8.0).clamp(0.0, 255.0);
                *m = val as u8;
            }
            let _ = state.waterfall_tx.send(mag);
        }

        // 2. Demodulation
        let audio_raw = match current_mode {
            DemodMode::Fm => fm.process(iq),
            DemodMode::Am => am.process(iq),
            DemodMode::Off => continue,
        };

        let audio_final = audio_decimator.process(&audio_raw);
        let _ = state.audio_tx.send(audio_final);
    }
    Ok(())
}
