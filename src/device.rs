use rusb::{Context, DeviceHandle, UsbContext};
use crate::error::{Error, Result};
use crate::registers::{Block, Request};
use std::time::Duration;
use std::slice;
use libusb1_sys as ffi;

unsafe extern "C" {
    fn libusb_dev_mem_alloc(dev_handle: *mut ffi::libusb_device_handle, len: libc::size_t) -> *mut u8;
    fn libusb_dev_mem_free(dev_handle: *mut ffi::libusb_device_handle, ptr: *mut u8, len: libc::size_t) -> libc::c_int;
}

const RTL2832U_VID: u16 = 0x0bda;
const RTL2832U_PID: u16 = 0x2838; // Common PID for RTL2832U (RTL-SDR Blog V3/V4)

pub struct Device<T: UsbContext> {
    handle: DeviceHandle<T>,
}

pub enum BufferType {
    Dma(*mut u8),
    Heap(Vec<u8>),
}

/// A block of memory for USB transfers (DMA-capable if available, fallback to Heap).
pub struct TransportBuffer<'a, T: UsbContext> {
    device: &'a Device<T>,
    inner: BufferType,
    len: usize,
}

impl<'a, T: UsbContext> TransportBuffer<'a, T> {
    pub fn new(device: &'a Device<T>, len: usize) -> Self {
        let raw_handle = device.handle.as_raw();
        
        // Attempt DMA allocation
        let ptr = unsafe { libusb_dev_mem_alloc(raw_handle, len as libc::size_t) };
        
        if ptr.is_null() {
            // Fallback for Windows/macOS/Standard Linux
            Self {
                device,
                inner: BufferType::Heap(vec![0u8; len]),
                len,
            }
        } else {
            // Success for performance-tuned Linux (Pi 5)
            Self {
                device,
                inner: BufferType::Dma(ptr),
                len,
            }
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        match &self.inner {
            BufferType::Dma(ptr) => unsafe { slice::from_raw_parts(*ptr, self.len) },
            BufferType::Heap(v) => v.as_slice(),
        }
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        match &mut self.inner {
            BufferType::Dma(ptr) => unsafe { slice::from_raw_parts_mut(*ptr, self.len) },
            BufferType::Heap(v) => v.as_mut_slice(),
        }
    }
}

impl<'a, T: UsbContext> Drop for TransportBuffer<'a, T> {
    fn drop(&mut self) {
        if let BufferType::Dma(ptr) = self.inner {
            let raw_handle = self.device.handle.as_raw();
            unsafe {
                libusb_dev_mem_free(raw_handle, ptr, self.len as libc::size_t);
            }
        }
    }
}

unsafe impl<'a, T: UsbContext> Send for TransportBuffer<'a, T> {}

pub trait HardwareInterface: Send + Sync {
    fn read_reg(&self, block: Block, addr: u16) -> Result<u8>;
    fn write_reg(&self, block: Block, addr: u16, val: u8) -> Result<()>;
    fn i2c_read(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>>;
    fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()>;
    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize>;
}

impl<T: UsbContext> HardwareInterface for Device<T> {
    fn read_reg(&self, block: Block, addr: u16) -> Result<u8> {
        self.read_reg(block, addr)
    }

    fn write_reg(&self, block: Block, addr: u16, val: u8) -> Result<()> {
        self.write_reg(block, addr, val)
    }

    fn i2c_read(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        self.i2c_read(addr, reg, len)
    }

    fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.i2c_write(addr, reg, data)
    }

    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.read_bulk(endpoint, buf, timeout)
    }
}

impl<T: UsbContext> Device<T> {
    pub fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        Ok(self.handle.read_bulk(endpoint, buf, timeout)?)
    }

    pub fn read_reg(&self, block: Block, addr: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        let block_addr = block as u16;
        self.handle.read_control(0xc0, Request::RegRead as u8, addr, block_addr, &mut buf, Duration::from_millis(100))?;
        Ok(buf[0])
    }

    pub fn write_reg(&self, block: Block, addr: u16, val: u8) -> Result<()> {
        let block_addr = block as u16;
        self.handle.write_control(0x40, Request::RegWrite as u8, val as u16, block_addr | addr, &[], Duration::from_millis(100))?;
        Ok(())
    }

    pub fn i2c_read(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        self.handle.read_control(0xc0, Request::I2cRead as u8, (addr as u16) << 8 | reg as u16, 0, &mut buf, Duration::from_millis(100))?;
        Ok(buf)
    }

    pub fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.handle.write_control(0x40, Request::I2cWrite as u8, (addr as u16) << 8 | reg as u16, 0, data, Duration::from_millis(100))?;
        Ok(())
    }
}

impl Device<Context> {
    pub fn open() -> Result<Self> {
        let context = Context::new()?;
        let devices = context.devices()?;
        for device in devices.iter() {
            let descriptor = device.device_descriptor()?;
            if descriptor.vendor_id() == RTL2832U_VID && descriptor.product_id() == RTL2832U_PID {
                let handle = device.open()?;
                
                #[cfg(target_os = "linux")]
                {
                    // Pi 5 / Linux Hacks: reset and detach (fail gracefully)
                    let _ = handle.reset();
                    if handle.kernel_driver_active(0).unwrap_or(false) {
                        let _ = handle.detach_kernel_driver(0);
                    }
                }

                handle.claim_interface(0)?;
                return Ok(Self { handle });
            }
        }
        Err(Error::NotFound)
    }
}
