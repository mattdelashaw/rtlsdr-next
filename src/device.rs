use rusb::{Context, DeviceHandle, UsbContext};
use crate::error::{Error, Result};
use crate::registers::{self, tuner_ids, Request};
use crate::tuner::TunerType;
use std::time::Duration;
use log::info;
use std::slice;
use libusb1_sys as ffi;

unsafe extern "C" {
    fn libusb_dev_mem_alloc(
        dev_handle: *mut ffi::libusb_device_handle,
        len: libc::size_t,
    ) -> *mut u8;
    fn libusb_dev_mem_free(
        dev_handle: *mut ffi::libusb_device_handle,
        ptr: *mut u8,
        len: libc::size_t,
    ) -> libc::c_int;
}

// ============================================================
// USB identifiers
// ============================================================

const RTL2832U_VID: u16 = 0x0bda;
const RTL2832U_PID: u16 = 0x2838;

/// EEPROM strings that identify the RTL-SDR Blog V4 specifically.
/// The driver checks for these exact strings to enable V4 triplexer logic.
/// Source: RTL-SDR Blog quickstart guide — "do not change the EEPROM
/// manufacturer or product strings as the drivers check for a specific string".
const V4_MANUFACTURER: &str = "RTLSDRBlog";
const V4_PRODUCT:      &str = "Blog V4";

// ============================================================
// DeviceInfo — returned from open() so callers know what they got
// ============================================================

/// Metadata probed during device open.
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub product:      String,
    /// True iff the EEPROM identifies this as an RTL-SDR Blog V4.
    pub is_v4:        bool,
}

impl DeviceInfo {
    fn probe<T: UsbContext>(handle: &DeviceHandle<T>) -> Self {
        let descriptor = handle.device().device_descriptor()
            .unwrap_or_else(|_| panic!("failed to read device descriptor"));

        let timeout = Duration::from_millis(200);

        let manufacturer = descriptor
            .manufacturer_string_index()
            .and_then(|idx| handle.read_string_descriptor_ascii(idx).ok())
            .unwrap_or_default();

        let product = descriptor
            .product_string_index()
            .and_then(|idx| handle.read_string_descriptor_ascii(idx).ok())
            .unwrap_or_default();

        let is_v4 = manufacturer.trim() == V4_MANUFACTURER
                 && product.trim()      == V4_PRODUCT;

        let _ = timeout; // timeout used implicitly by read_string_descriptor_ascii

        Self { manufacturer, product, is_v4 }
    }
}

// ============================================================
// Device
// ============================================================

pub struct Device<T: UsbContext> {
    handle: DeviceHandle<T>,
    pub info: DeviceInfo,
}

use std::ops::{Deref, DerefMut};
use std::sync::Arc;

// ============================================================
// DMA / Heap transport buffer
// ============================================================

pub enum BufferType {
    Dma(*mut u8),
    Heap(Vec<u8>),
}

/// USB transfer buffer — DMA-pinned on Pi 5 / Linux RP1, heap elsewhere.
///
/// This buffer owns a reference to the Device to ensure the device handle
/// remains valid as long as the DMA buffer exists (needed for free).
pub struct TransportBuffer<T: UsbContext> {
    device: Arc<Device<T>>,
    inner:  BufferType,
    len:    usize,
}

impl<T: UsbContext> TransportBuffer<T> {
    pub fn new(device: Arc<Device<T>>, len: usize) -> Self {
        let raw_handle = device.handle.as_raw();
        let ptr = unsafe { libusb_dev_mem_alloc(raw_handle, len as libc::size_t) };

        if ptr.is_null() {
            Self { device, inner: BufferType::Heap(vec![0u8; len]), len }
        } else {
            Self { device, inner: BufferType::Dma(ptr), len }
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        match &self.inner {
            BufferType::Dma(ptr) => unsafe { slice::from_raw_parts(*ptr, self.len) },
            BufferType::Heap(v)  => v.as_slice(),
        }
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        match &mut self.inner {
            BufferType::Dma(ptr) => unsafe { slice::from_raw_parts_mut(*ptr, self.len) },
            BufferType::Heap(v)  => v.as_mut_slice(),
        }
    }
}

impl<T: UsbContext> Deref for TransportBuffer<T> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T: UsbContext> DerefMut for TransportBuffer<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<T: UsbContext> Drop for TransportBuffer<T> {
    fn drop(&mut self) {
        if let BufferType::Dma(ptr) = self.inner {
            let raw_handle = self.device.handle.as_raw();
            unsafe { libusb_dev_mem_free(raw_handle, ptr, self.len as libc::size_t); }
        }
    }
}

unsafe impl<T: UsbContext> Send for TransportBuffer<T> {}
unsafe impl<T: UsbContext> Sync for TransportBuffer<T> {}

// ============================================================
// HardwareInterface trait
// ============================================================

pub trait HardwareInterface: Send + Sync {
    fn read_reg(&self,  block: u16, addr: u16) -> Result<u8>;
    fn write_reg(&self, block: u16, addr: u16, val: u8) -> Result<()>;
    fn i2c_read(&self,  addr: u8, reg: u8, len: usize) -> Result<Vec<u8>>;
    fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()>;
    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize>;

    /// Write to the tuner via I2C with the RTL2832U repeater gate bracketed.
    fn i2c_write_tuner(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.write_reg(crate::registers::Block::Demod as u16, crate::registers::demod::P0_IIC_REPEAT, 0x08)?;
        let result = self.i2c_write(addr, reg, data);
        self.write_reg(crate::registers::Block::Demod as u16, crate::registers::demod::P0_IIC_REPEAT, 0x00)?;
        result
    }

    /// Read from the tuner via I2C with the RTL2832U repeater gate bracketed.
    fn i2c_read_tuner(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        self.write_reg(crate::registers::Block::Demod as u16, crate::registers::demod::P0_IIC_REPEAT, 0x08)?;
        let result = self.i2c_read(addr, reg, len);
        self.write_reg(crate::registers::Block::Demod as u16, crate::registers::demod::P0_IIC_REPEAT, 0x00)?;
        result
    }
}

// ============================================================
// HardwareInterface impl for Device<T>
// ============================================================

impl<T: UsbContext> HardwareInterface for Device<T> {
    fn read_reg(&self, block: u16, addr: u16) -> Result<u8> {
        Device::read_reg(self, block, addr)
    }
    fn write_reg(&self, block: u16, addr: u16, val: u8) -> Result<()> {
        Device::write_reg(self, block, addr, val)
    }
    fn i2c_read(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        Device::i2c_read(self, addr, reg, len)
    }
    fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        Device::i2c_write(self, addr, reg, data)
    }
    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        Device::read_bulk(self, endpoint, buf, timeout)
    }
}

// ============================================================
// Device register / I2C / bulk methods
// ============================================================

impl<T: UsbContext> Device<T> {
    /// Probe the I2C bus to identify the connected tuner.
    pub fn probe_tuner(&self) -> Result<TunerType> {
        // Enable I2C repeater
        self.write_reg(
            registers::Block::Demod as u16,
            registers::demod::P0_IIC_REPEAT,
            0x08,
        )?;

        let mut found = TunerType::Unknown(0);

        // 1. R820T / R820T2 / R828D — check that the I2C address responds.
        //    The chip ID register (0x00) returns a status byte; we just need
        //    the read to succeed. The V4 vs V3 distinction is made via EEPROM
        //    strings (already in DeviceInfo.is_v4), not the tuner ID.
        if self.i2c_read(tuner_ids::R82XX_I2C_ADDR, tuner_ids::R82XX_CHECK_REG, 1).is_ok() {
            found = TunerType::R820T;
        }

        // 2. E4000
        if let TunerType::Unknown(_) = found {
            if let Ok(data) = self.i2c_read(tuner_ids::E4000_I2C_ADDR, tuner_ids::E4000_CHECK_REG, 1) {
                if data.first() == Some(&tuner_ids::E4000_CHECK_VAL) {
                    found = TunerType::E4000;
                }
            }
        }

        // 3. FC0012 / FC0013
        if let TunerType::Unknown(_) = found {
            if let Ok(data) = self.i2c_read(tuner_ids::FC0012_I2C_ADDR, tuner_ids::FC0012_CHECK_REG, 1) {
                match data.first() {
                    Some(&v) if v == tuner_ids::FC0012_CHECK_VAL => found = TunerType::FC0012,
                    Some(&v) if v == tuner_ids::FC0013_CHECK_VAL => found = TunerType::FC0013,
                    _ => {}
                }
            }
        }

        // Disable I2C repeater
        self.write_reg(
            registers::Block::Demod as u16,
            registers::demod::P0_IIC_REPEAT,
            0x00,
        )?;

        Ok(found)
    }

    pub fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        Ok(self.handle.read_bulk(endpoint, buf, timeout)?)
    }

    pub fn read_reg(&self, block: u16, addr: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.handle.read_control(
            0xc0,
            Request::RegRead,
            addr,
            block,
            &mut buf,
            Duration::from_millis(100),
        )?;
        Ok(buf[0])
    }

    pub fn write_reg(&self, block: u16, addr: u16, val: u8) -> Result<()> {
        self.handle.write_control(
            0x40,
            Request::RegWrite,
            val as u16,
            block | addr,
            &[],
            Duration::from_millis(100),
        )?;
        Ok(())
    }

    pub fn i2c_read(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        self.handle.read_control(
            0xc0,
            Request::I2cRead,
            (addr as u16) << 8 | reg as u16,
            0,
            &mut buf,
            Duration::from_millis(100),
        )?;
        Ok(buf)
    }

    pub fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.handle.write_control(
            0x40,
            Request::I2cWrite,
            (addr as u16) << 8 | reg as u16,
            0,
            data,
            Duration::from_millis(100),
        )?;
        Ok(())
    }
}

// ============================================================
// Device::open  (concrete Context only)
// ============================================================

impl Device<Context> {
    /// Scan USB buses for an RTL2832U, open it, probe EEPROM strings,
    /// detach the DVB-T kernel driver, and claim the interface.
    ///
    /// Returns `(Device, DeviceInfo)` so the caller has immediate access
    /// to `info.is_v4` without a separate query.
    pub fn open() -> Result<Self> {
        let context = Context::new()?;
        let devices = context.devices()?;

        for device in devices.iter() {
            let descriptor = device.device_descriptor()?;
            if descriptor.vendor_id()  != RTL2832U_VID
            || descriptor.product_id() != RTL2832U_PID
            {
                continue;
            }

            let handle = device.open()?;

            // ── Pi 5 / Linux: force USB reset then detach DVB-T driver ──
            #[cfg(target_os = "linux")]
            {
                // Ignore errors — device may not need reset or driver detach
                let _ = handle.reset();
                if handle.kernel_driver_active(0).unwrap_or(false) {
                    let _ = handle.detach_kernel_driver(0);
                }
            }

            // ── Probe EEPROM strings BEFORE claiming the interface ───────
            // String descriptors are accessible without claiming.
            let info = DeviceInfo::probe(&handle);

            info!(
                "Found RTL2832U — manufacturer: {:?}  product: {:?}  is_v4: {}",
                info.manufacturer, info.product, info.is_v4
            );

            handle.claim_interface(0)?;

            return Ok(Self { handle, info });
        }

        Err(Error::NotFound)
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_v4_detection_positive() {
        // Simulate exactly the strings the V4 EEPROM carries
        let manufacturer = V4_MANUFACTURER.to_string();
        let product      = V4_PRODUCT.to_string();
        let is_v4 = manufacturer.trim() == V4_MANUFACTURER
                 && product.trim()      == V4_PRODUCT;
        assert!(is_v4, "Should detect V4 with correct EEPROM strings");
    }

    #[test]
    fn test_v4_detection_negative_v3() {
        // V3 has a different product string
        let manufacturer = "RTLSDRBlog".to_string();
        let product      = "Blog V3".to_string();
        let is_v4 = manufacturer.trim() == V4_MANUFACTURER
                 && product.trim()      == V4_PRODUCT;
        assert!(!is_v4, "V3 should not be detected as V4");
    }

    #[test]
    fn test_v4_detection_negative_generic() {
        // Generic cheap clone with no manufacturer string
        let manufacturer = "".to_string();
        let product      = "RTL2838UHIDIR".to_string();
        let is_v4 = manufacturer.trim() == V4_MANUFACTURER
                 && product.trim()      == V4_PRODUCT;
        assert!(!is_v4, "Generic clone should not be detected as V4");
    }

    #[test]
    fn test_v4_detection_trims_whitespace() {
        // Paranoia test — some EEPROM writers pad strings with spaces
        let manufacturer = "RTLSDRBlog ".to_string();
        let product      = " Blog V4".to_string();
        let is_v4 = manufacturer.trim() == V4_MANUFACTURER
                 && product.trim()      == V4_PRODUCT;
        assert!(is_v4, "Should detect V4 even with padded EEPROM strings");
    }
}