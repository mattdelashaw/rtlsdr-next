use rtlsdr_next::Driver;
use rtlsdr_next::websdr::WebSdrServer;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // 1. Open device
    let driver = Driver::new()?;

    // 2. Start the WebSDR backend on all interfaces, port 8080
    WebSdrServer::start(driver, "0.0.0.0:8080").await?;

    Ok(())
}
