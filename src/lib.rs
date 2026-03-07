use std::sync::Arc;
pub mod device;
pub mod error;
pub mod registers;
pub mod tuner;
pub mod tuners;
pub mod stream;
pub mod converter;
pub mod dsp;
pub mod server;

pub use device::Device;
pub use error::{Error, Result};
pub use tuner::{Tuner, FilterRange};
pub use stream::SampleStream;
pub use server::SharingServer;

/// The main entry point for the next-generation RTL-SDR driver.
///
/// # Example
///
/// ```no_run
/// use rtlsdr_next::Driver;
///
/// #[tokio::main]
/// async fn main() -> anyhow::Result<()> {
///     let driver = Driver::new()?;
///     let mut stream = driver.stream();
///     
///     while let Some(samples) = stream.next().await {
///         // Process samples...
///     }
///     Ok(())
/// }
/// ```
pub struct Driver {
    device: Arc<Device<rusb::Context>>,
    /// Access the underlying tuner to change frequency or gain.
    pub tuner: Box<dyn Tuner>,
}

impl Driver {
    /// Discovers and initializes the first available RTL-SDR device.
    ///
    /// This will automatically:
    /// 1. Detach kernel drivers (on Linux).
    /// 2. Reset the USB bus (for Pi 5 stability).
    /// 3. Identify and initialize the tuner (e.g., R828D).
    pub fn new() -> Result<Self> {
        let device = Arc::new(Device::open()?);
        
        let is_v4 = true; // Defaulting to V4 for now
        
        // Pass device as Arc<dyn HardwareInterface>
        let tuner: Box<dyn Tuner> = Box::new(tuners::r828d::R828D::new(device.clone(), is_v4));
        tuner.initialize()?;
        
        Ok(Self { device, tuner })
    }

    /// Create a new asynchronous sample stream.
    pub fn stream(&self) -> SampleStream {
        SampleStream::new(self.device.clone())
    }

    /// Start a sharing server to allow multiple local applications to receive samples.
    pub async fn start_sharing<P: AsRef<std::path::Path>>(&self, path: P) -> Result<SharingServer> {
        let mut stream = self.stream();
        let (tx, rx) = tokio::sync::broadcast::channel::<Arc<Vec<u8>>>(16);
        
        // Spawn a relay task to feed the broadcast
        tokio::spawn(async move {
            while let Some(samples) = stream.next().await {
                let _ = tx.send(Arc::new(samples));
            }
        });

        Ok(SharingServer::start(path, rx).await.map_err(|e| Error::Tuner(format!("Server error: {:?}", e)))?)
    }
}
