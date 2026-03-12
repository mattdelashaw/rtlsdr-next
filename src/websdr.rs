use axum::{
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::IntoResponse,
    routing::get,
    Router,
};
use tokio::sync::{broadcast, Mutex};
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use crate::Driver;
use crate::dsp::{FmDemodulator, Decimator};
use rustfft::{FftPlanner, num_complex::Complex};
use log::{info, error};
use futures_util::{StreamExt, SinkExt};

// ── WebSocket Protocol ──────────────────────────────────────────────────────

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type", rename_all = "lowercase")]
enum Command {
    SetFrequency { hz: u64 },
    SetGain { db: f32 },
    SetDemod { mode: String }, // "fm", "am", "off"
}

#[derive(Serialize, Debug)]
#[serde(tag = "type", rename_all = "lowercase")]
enum WebEvent {
    HardwareInfo { manufacturer: String, product: String, is_v4: bool },
    FrequencyChange { hz: u64 },
    GainChange { db: f32 },
}

// ── Server State ─────────────────────────────────────────────────────────────

pub struct WebSdrServer {
    driver: Arc<Mutex<Driver>>,
    waterfall_tx: broadcast::Sender<Vec<u8>>, // magnitude bytes (0-255)
    audio_tx:     broadcast::Sender<Vec<f32>>, // f32 audio samples
}

impl WebSdrServer {
    pub async fn start(driver: Driver, addr: &str) -> anyhow::Result<()> {
        let driver = Arc::new(Mutex::new(driver));
        
        // Broadcast channels for waterfall and audio
        let (waterfall_tx, _) = broadcast::channel(16);
        let (audio_tx,     _) = broadcast::channel(16);

        let state = Arc::new(Self {
            driver: driver.clone(),
            waterfall_tx: waterfall_tx.clone(),
            audio_tx:     audio_tx.clone(),
        });

        // 1. Hardware Pipeline Task (FFT + Demod)
        let pipeline_state = state.clone();
        tokio::spawn(async move {
            if let Err(e) = run_pipeline(pipeline_state).await {
                error!("WebSDR Pipeline Error: {:?}", e);
            }
        });

        // 2. Web Server
        let app = Router::new()
            .route("/ws", get(ws_handler))
            .with_state(state);

        let listener = tokio::net::TcpListener::bind(addr).await?;
        info!("WebSDR Backend listening on http://{}", addr);
        axum::serve(listener, app).await?;
        
        Ok(())
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
    // Send initial hardware info
    let (info, _freq, _gain) = {
        let d = state.driver.lock().await;
        (d.info.clone(), d.frequency, d.tuner.get_gain().unwrap_or_else(|_| 0.0))
    };
    
    let json = serde_json::to_string(&WebEvent::HardwareInfo {
        manufacturer: info.manufacturer,
        product: info.product,
        is_v4: info.is_v4,
    }).expect("Serialization failed");
    
    let _ = socket.send(Message::Text(json.into())).await;

    let mut waterfall_rx = state.waterfall_tx.subscribe();
    let mut audio_rx     = state.audio_tx.subscribe();

    let (sender, mut receiver) = socket.split();

    // Task: Push waterfall (binary)
    let sender_shared = Arc::new(Mutex::new(sender));

    let s1 = sender_shared.clone();
    let mut waterfall_task = tokio::spawn(async move {
        while let Ok(data) = waterfall_rx.recv().await {
            let mut msg = vec![b'W'];
            msg.extend_from_slice(&data);
            if s1.lock().await.send(Message::Binary(msg.into())).await.is_err() { break; }
        }
    });

    let s2 = sender_shared.clone();
    let mut audio_task = tokio::spawn(async move {
        while let Ok(data) = audio_rx.recv().await {
            let mut msg = vec![b'A'];
            // Convert f32 slice to bytes (native endian)
            let bytes: &[u8] = unsafe {
                std::slice::from_raw_parts(
                    data.as_ptr() as *const u8,
                    data.len() * std::mem::size_of::<f32>(),
                )
            };
            msg.extend_from_slice(bytes);
            if s2.lock().await.send(Message::Binary(msg.into())).await.is_err() { break; }
        }
    });

    // Task: Process Commands
    let cmd_driver = state.driver.clone();
    let mut cmd_task = tokio::spawn(async move {
        while let Some(Ok(Message::Text(text))) = receiver.next().await {
            if let Ok(cmd) = serde_json::from_str::<Command>(&text) {
                let mut d = cmd_driver.lock().await;
                match cmd {
                    Command::SetFrequency { hz } => { let _ = d.set_frequency(hz); }
                    Command::SetGain { db } => { let _ = d.tuner.set_gain(db); }
                    Command::SetDemod { .. } => { /* TODO: Dynamic switch */ }
                }
            }
        }
    });

    tokio::select! {
        _ = &mut waterfall_task => {},
        _ = &mut audio_task => {},
        _ = &mut cmd_task => {},
    }
    waterfall_task.abort();
    audio_task.abort();
    cmd_task.abort();
}

// ── The Pipeline (FFT & Demod) ─────────────────────────────────────────────

async fn run_pipeline(state: Arc<WebSdrServer>) -> anyhow::Result<()> {
    let mut stream = {
        let d = state.driver.lock().await;
        let stream: crate::stream::F32Stream<rusb::Context> = d.stream_f32(8);
        stream.with_dc_removal(0.01)
            .with_agc(1.0, 0.01, 0.01)
    };

    let mut fm = FmDemodulator::new();
    let mut planner = FftPlanner::new();
    let fft_size = 1024;
    let fft = planner.plan_fft_forward(fft_size);
    
    let mut audio_decimator = Decimator::new(5, 0.1, 31); // 256k -> 51k (audio-ish)

    while let Some(res) = stream.next().await {
        let iq_pooled = res?;
        let iq = &*iq_pooled;
        
        // 1. FFT for Waterfall
        if iq.len() >= fft_size * 2 {
            let mut buffer: Vec<Complex<f32>> = (0..fft_size)
                .map(|i| Complex::new(iq[i*2], iq[i*2+1]))
                .collect();
            
            fft.process(&mut buffer);

            // Shift (center DC) and convert to magnitude bytes
            let mut mag = vec![0u8; fft_size];
            for i in 0..fft_size {
                let shifted = (i + fft_size / 2) % fft_size;
                let val = (buffer[shifted].norm().log10() * 20.0 + 60.0).clamp(0.0, 255.0);
                mag[i] = val as u8;
            }
            let _ = state.waterfall_tx.send(mag);
        }

        // 2. FM Demodulation
        let audio_raw = fm.process(iq);
        let audio_final = audio_decimator.process(&audio_raw);
        let _ = state.audio_tx.send(audio_final);
    }
    Ok(())
}