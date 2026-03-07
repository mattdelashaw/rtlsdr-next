use rtlsdr_next::Driver;
use log::info;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
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

    info!("Hardware: {} (V4: {})", driver.info.product, driver.info.is_v4);

    // 2. Start the rtl_tcp server on port 1234
    // Standard rtl_tcp port is 1234. Listening on 0.0.0.0 makes it accessible from other machines.
    let addr = "0.0.0.0:1234";
    let server = driver.start_rtl_tcp(addr).await?;

    info!("Server is running! You can now connect with OpenWebRX, SDR#, GQRX, etc.");
    info!("Connect to: <your-pi-ip>:1234");

    // 3. Keep the server running until Ctrl+C
    tokio::signal::ctrl_c().await?;
    
    info!("Shutting down server...");
    server.stop();

    Ok(())
}
