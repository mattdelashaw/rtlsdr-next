use anyhow::{Context, Result};
use log::info;
use rtlsdr_next::Driver;
use std::env;

struct Args {
    address: String,
    port: u16,
}

fn parse_args() -> Result<Args> {
    let args: Vec<String> = env::args().collect();
    let mut address = "0.0.0.0".to_string();
    let mut port = 1234;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "-a" | "--address" => {
                if i + 1 < args.len() {
                    address = args[i + 1].clone();
                    i += 2;
                } else {
                    anyhow::bail!("Missing value for address");
                }
            }
            "-p" | "--port" => {
                if i + 1 < args.len() {
                    port = args[i + 1]
                        .parse()
                        .with_context(|| format!("Invalid port number: {}", args[i + 1]))?;
                    i += 2;
                } else {
                    anyhow::bail!("Missing value for port");
                }
            }
            "-h" | "--help" => {
                println!("Usage: rtl_tcp [options]");
                println!("Options:");
                println!("  -a, --address <addr>  Listening address (default: 0.0.0.0)");
                println!("  -p, --port <port>     Listening port (default: 1234)");
                println!("  -h, --help            What else do you expect?");
                std::process::exit(0);
            }
            _ => {
                anyhow::bail!("Unknown argument: {}", args[i]);
            }
        }
    }

    Ok(Args { address, port })
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = parse_args()?;

    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    info!("Starting rtlsdr-next rtl_tcp server...");

    // 1. Open the device
    let driver = match Driver::new() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("Error opening device: {:?}\n(Check USB permissions)", e);
            return Ok(());
        }
    };

    info!(
        "Hardware: {} (V4: {})",
        driver.info.product, driver.info.is_v4
    );

    // 2. Start the rtl_tcp server
    let addr = format!("{}:{}", args.address, args.port);
    let server = driver.start_rtl_tcp(&addr).await?;

    info!("Server is running! You can now connect with OpenWebRX, SDR#, GQRX, etc.");
    info!("Connect to: {}", addr);

    // 3. Keep the server running until Ctrl+C
    tokio::signal::ctrl_c().await?;

    info!("Shutting down server...");
    server.stop();

    Ok(())
}
