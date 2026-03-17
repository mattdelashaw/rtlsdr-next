use anyhow::{Context, Result};
use rtlsdr_next::Driver;
use rtlsdr_next::websdr::WebSdrServer;
use std::env;

struct Args {
    address: String,
    port: u16,
}

fn parse_args() -> Result<Args> {
    let args: Vec<String> = env::args().collect();
    let mut address = "0.0.0.0".to_string();
    let mut port = 8080;

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
                println!("Usage: websdr [options]");
                println!("Options:");
                println!("  -a, --address <addr>  Listening address (default: 0.0.0.0)");
                println!("  -p, --port <port>     Listening port (default: 8080)");
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

    // 1. Open device
    let driver = Driver::new()?;

    // 2. Start the WebSdr server
    let addr = format!("{}:{}", args.address, args.port);
    WebSdrServer::start(driver, &addr).await?;

    Ok(())
}
