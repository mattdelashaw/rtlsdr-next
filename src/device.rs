use rusb::{Context, DeviceHandle, UsbContext};
use crate::error::{Error, Result};
use crate::registers::{self, tuner_ids, BREQUEST, block};
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

const V4_MANUFACTURER: &str = "RTLSDRBlog";
const V4_PRODUCT:      &str = "Blog V4";

// ============================================================
// DeviceInfo
// ============================================================

#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub product:      String,
    pub is_v4:        bool,
}

impl DeviceInfo {
    fn probe<T: UsbContext>(handle: &DeviceHandle<T>) -> Self {
        let descriptor = handle.device().device_descriptor()
            .unwrap_or_else(|_| panic!("failed to read device descriptor"));

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

        Self { manufacturer, product, is_v4 }
    }
}

// ============================================================
// DMA / Heap transport buffer
// ============================================================

pub enum BufferType {
    Dma(*mut u8),
    Heap(Vec<u8>),
}

pub struct TransportBuffer<T: UsbContext> {
    device: std::sync::Arc<Device<T>>,
    inner:  BufferType,
    len:    usize,
}

impl<T: UsbContext> TransportBuffer<T> {
    pub fn new(device: std::sync::Arc<Device<T>>, len: usize) -> Self {
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

impl<T: UsbContext> std::ops::Deref for TransportBuffer<T> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target { self.as_slice() }
}

impl<T: UsbContext> std::ops::DerefMut for TransportBuffer<T> {
    fn deref_mut(&mut self) -> &mut Self::Target { self.as_mut_slice() }
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
    // ── Regular register access (USB / SYS blocks) ──────────────────────
    fn read_reg(&self,   block: u8, addr: u16) -> Result<u8>;
    fn write_reg(&self,  block: u8, addr: u16, val: u8)  -> Result<()>;
    fn write_reg16(&self, block: u8, addr: u16, val: u16) -> Result<()>;

    // ── Demodulator register access (paged) ─────────────────────────────
    fn demod_read_reg(&self,  page: u8, addr: u16) -> Result<u8>;
    fn demod_write_reg(&self, page: u8, addr: u16, val: u8)  -> Result<()>;
    fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()>;

    // ── I2C (tuner) ──────────────────────────────────────────────────────
    fn i2c_read(&self,  addr: u8, reg: u8, len: usize) -> Result<Vec<u8>>;
    fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()>;
    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize>;

    // ── I2C repeater helpers ─────────────────────────────────────────────

    fn i2c_write_tuner(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.demod_write_reg(
            registers::demod::P0_PAGE,
            registers::demod::P0_IIC_REPEAT,
            0x08,
        )?;
        let result = self.i2c_write(addr, reg, data);
        self.demod_write_reg(
            registers::demod::P0_PAGE,
            registers::demod::P0_IIC_REPEAT,
            0x00,
        )?;
        result
    }

    fn i2c_read_tuner(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        self.demod_write_reg(
            registers::demod::P0_PAGE,
            registers::demod::P0_IIC_REPEAT,
            0x08,
        )?;
        let result = self.i2c_read(addr, reg, len);
        self.demod_write_reg(
            registers::demod::P0_PAGE,
            registers::demod::P0_IIC_REPEAT,
            0x00,
        )?;
        result
    }
}

// ============================================================
// HardwareInterface impl for Device<T>
// ============================================================

impl<T: UsbContext> HardwareInterface for Device<T> {
    fn read_reg(&self, blk: u8, addr: u16) -> Result<u8> {
        Device::read_reg(self, blk, addr)
    }
    fn write_reg(&self, blk: u8, addr: u16, val: u8) -> Result<()> {
        Device::write_reg(self, blk, addr, val)
    }
    fn write_reg16(&self, blk: u8, addr: u16, val: u16) -> Result<()> {
        Device::write_reg16(self, blk, addr, val)
    }
    fn demod_read_reg(&self, page: u8, addr: u16) -> Result<u8> {
        Device::demod_read_reg(self, page, addr)
    }
    fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()> {
        Device::demod_write_reg(self, page, addr, val)
    }
    fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()> {
        Device::demod_write_reg16(self, page, addr, val)
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
// Device struct
// ============================================================

pub struct Device<T: UsbContext> {
    handle: DeviceHandle<T>,
    pub info: DeviceInfo,
}

// ============================================================
// Device register / I2C / bulk methods
// ============================================================

const TIMEOUT: Duration = Duration::from_millis(200);

impl<T: UsbContext> Device<T> {
    // ── Regular register read/write ──────────────────────────────────────
    //
    // Encoding (matches librtlsdr rtlsdr_write_reg / rtlsdr_read_reg):
    //   write: bmRequestType=0x40, bRequest=0, wValue=addr, wIndex=(block<<8)|0x10, data=[val]
    //   read:  bmRequestType=0xC0, bRequest=0, wValue=addr, wIndex=(block<<8),      data=buf

    pub fn read_reg(&self, blk: u8, addr: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        let index = (blk as u16) << 8;
        self.handle.read_control(
            0xc0, BREQUEST,
            addr,   // wValue = register address
            index,  // wIndex = block<<8
            &mut buf,
            TIMEOUT,
        )?;
        Ok(buf[0])
    }

    pub fn write_reg(&self, blk: u8, addr: u16, val: u8) -> Result<()> {
        let index = ((blk as u16) << 8) | 0x10;
        let data = [val];
        self.handle.write_control(
            0x40, BREQUEST,
            addr,   // wValue = register address
            index,  // wIndex = (block<<8)|0x10
            &data,
            TIMEOUT,
        )?;
        Ok(())
    }

    pub fn write_reg16(&self, blk: u8, addr: u16, val: u16) -> Result<()> {
        let index = ((blk as u16) << 8) | 0x10;
        let data = [((val >> 8) & 0xff) as u8, (val & 0xff) as u8];
        self.handle.write_control(
            0x40, BREQUEST,
            addr,
            index,
            &data,
            TIMEOUT,
        )?;
        Ok(())
    }

    // ── Demodulator register read/write ──────────────────────────────────
    //
    // Encoding (matches librtlsdr rtlsdr_demod_write_reg / rtlsdr_demod_read_reg):
    //   write: bmRequestType=0x40, bRequest=0, wValue=(addr<<8)|0x20, wIndex=0x10|page, data=[val]
    //   read:  bmRequestType=0xC0, bRequest=0, wValue=(addr<<8)|0x20, wIndex=page,      data=buf

    pub fn demod_read_reg(&self, page: u8, addr: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        let w_value = (addr << 8) | 0x20;
        let w_index = page as u16;
        self.handle.read_control(
            0xc0, BREQUEST,
            w_value,
            w_index,
            &mut buf,
            TIMEOUT,
        )?;
        Ok(buf[0])
    }

    pub fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()> {
        let w_value = (addr << 8) | 0x20;
        let w_index = 0x10 | (page as u16);
        let data = [val, 0x00]; // librtlsdr always sends 2 bytes; for len=1, data[0]=val
        self.handle.write_control(
            0x40, BREQUEST,
            w_value,
            w_index,
            &data[..1], // 1-byte write
            TIMEOUT,
        )?;
        Ok(())
    }

    pub fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()> {
        let w_value = (addr << 8) | 0x20;
        let w_index = 0x10 | (page as u16);
        let data = [((val >> 8) & 0xff) as u8, (val & 0xff) as u8];
        self.handle.write_control(
            0x40, BREQUEST,
            w_value,
            w_index,
            &data,
            TIMEOUT,
        )?;
        Ok(())
    }

    // ── I2C ──────────────────────────────────────────────────────────────
    //
    // I2C transfers go through the RTL2832U's I2C master using a distinct
    // bRequest. The I2C address and register are packed into wValue.

    pub fn i2c_read(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        let r = self.handle.read_control(
            0xc0,
            0x01,
            (reg as u16) | ((addr as u16) << 8),
            0,
            &mut buf,
            TIMEOUT,
        );
        r?;
        Ok(buf)
    }

    pub fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        let r = self.handle.write_control(
            0x40,
            0x01,
            (reg as u16) | ((addr as u16) << 8),
            0,
            data,
            TIMEOUT,
        );
        r?;
        Ok(())
    }

    // ── Bulk read ────────────────────────────────────────────────────────

    pub fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        Ok(self.handle.read_bulk(endpoint, buf, timeout)?)
    }

    // ── Tuner probe ──────────────────────────────────────────────────────

    pub fn probe_tuner(&self) -> Result<TunerType> {
        let mut found = TunerType::Unknown(0);

        // R820T / R820T2 / R828D — check address responds
        if self.i2c_read(tuner_ids::R82XX_I2C_ADDR, tuner_ids::R82XX_CHECK_REG, 1).is_ok() {
            found = TunerType::R820T;
        }

        // E4000
        if let TunerType::Unknown(_) = found {
            if let Ok(data) = self.i2c_read(tuner_ids::E4000_I2C_ADDR, tuner_ids::E4000_CHECK_REG, 1) {
                if data.first() == Some(&tuner_ids::E4000_CHECK_VAL) {
                    found = TunerType::E4000;
                }
            }
        }

        // FC0012 / FC0013
        if let TunerType::Unknown(_) = found {
            if let Ok(data) = self.i2c_read(tuner_ids::FC0012_I2C_ADDR, tuner_ids::FC0012_CHECK_REG, 1) {
                match data.first() {
                    Some(&v) if v == tuner_ids::FC0012_CHECK_VAL => found = TunerType::FC0012,
                    Some(&v) if v == tuner_ids::FC0013_CHECK_VAL => found = TunerType::FC0013,
                    _ => {}
                }
            }
        }

        Ok(found)
    }
}

// ============================================================
// Device::open  (concrete Context only)
// ============================================================

impl Device<Context> {
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

            #[cfg(target_os = "linux")]
            {
                let _ = handle.reset();
                if handle.kernel_driver_active(0).unwrap_or(false) {
                    let _ = handle.detach_kernel_driver(0);
                }
            }

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
        let manufacturer = V4_MANUFACTURER.to_string();
        let product      = V4_PRODUCT.to_string();
        let is_v4 = manufacturer.trim() == V4_MANUFACTURER
                 && product.trim()      == V4_PRODUCT;
        assert!(is_v4);
    }

    #[test]
    fn test_v4_detection_negative() {
        let is_v4 = "RTLSDRBlog".trim() == V4_MANUFACTURER
                 && "Blog V3".trim()    == V4_PRODUCT;
        assert!(!is_v4);
    }

    #[test]
    fn test_write_reg16_encoding() {
        // Verify 2-byte value splits correctly: high byte first
        let val: u16 = 0x1002;
        let data = [((val >> 8) & 0xff) as u8, (val & 0xff) as u8];
        assert_eq!(data, [0x10, 0x02]);
    }

    #[test]
    fn test_demod_wvalue_encoding() {
        // demod_write_reg wValue = (addr<<8)|0x20
        let addr: u16 = 0x0001;
        let w_value = (addr << 8) | 0x20;
        assert_eq!(w_value, 0x0120);
    }

    #[test]
    fn test_regular_windex_encoding() {
        // write_reg wIndex = (block<<8)|0x10
        let blk = block::SYS; // = 2
        let index = ((blk as u16) << 8) | 0x10;
        assert_eq!(index, 0x0210);
    }
}
