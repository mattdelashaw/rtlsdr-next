//! RTL2832U USB device driver.

use log::{debug, error};
use rusb::{Context, DeviceHandle, UsbContext};
use std::sync::Arc;
use std::time::Duration;

use crate::error::{Error, Result};
use crate::registers::{self, BREQUEST, block};
use crate::tuner::TunerType;

const TIMEOUT: Duration = Duration::from_millis(1000);

pub trait HardwareInterface: Send + Sync {
    fn write_reg(&self, blk: u16, addr: u16, val: u8) -> Result<()>;
    fn write_reg16(&self, blk: u16, addr: u16, val: u16) -> Result<()>;
    fn read_reg(&self, blk: u16, addr: u16) -> Result<u8>;
    fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()>;
    fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()>;
    fn demod_read_reg(&self, page: u8, addr: u16) -> Result<u8>;
    fn set_i2c_repeater(&self, on: bool) -> Result<()>;
    fn i2c_write_raw(&self, addr: u8, data: &[u8]) -> Result<()>;
    fn i2c_read_raw(&self, addr: u8, len: usize) -> Result<Vec<u8>>;
    fn i2c_write_tuner(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()>;
    fn i2c_read_tuner(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>>;
    fn i2c_read_direct(&self, addr: u8, len: usize) -> Result<Vec<u8>>;
    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    fn set_gpio_output(&self, gpio: u8) -> Result<()>;
    fn set_gpio_bit(&self, gpio: u8, value: bool) -> Result<()>;
    fn probe_tuner(&self) -> Result<TunerType>;
}

pub struct Device<T: UsbContext> {
    handle: DeviceHandle<T>,
    _context: T,
}

impl<T: UsbContext> Device<T> {
    pub fn as_hw(&self) -> &dyn HardwareInterface {
        self
    }
    pub fn read_info(&self) -> DeviceInfo {
        DeviceInfo::probe(&self.handle)
    }
}

impl<T: UsbContext> HardwareInterface for Device<T> {
    fn write_reg(&self, blk: u16, addr: u16, val: u8) -> Result<()> {
        let index = (blk << 8) | 0x10;
        self.handle
            .write_control(0x40, BREQUEST, addr, index, &[val], TIMEOUT)
            .map_err(|e| {
                error!("write_reg failed: {:?}", e);
                Error::Usb(e)
            })?;
        Ok(())
    }

    fn write_reg16(&self, blk: u16, addr: u16, val: u16) -> Result<()> {
        let index = (blk << 8) | 0x10;
        let data = [(val >> 8) as u8, (val & 0xff) as u8];
        self.handle
            .write_control(0x40, BREQUEST, addr, index, &data, TIMEOUT)
            .map_err(|e| {
                error!("write_reg16 failed: {:?}", e);
                Error::Usb(e)
            })?;
        Ok(())
    }

    fn read_reg(&self, blk: u16, addr: u16) -> Result<u8> {
        let index = blk << 8;
        let mut buf = [0u8; 1];
        self.handle
            .read_control(0xc0, BREQUEST, addr, index, &mut buf, TIMEOUT)
            .map_err(|e| {
                error!("read_reg failed: {:?}", e);
                Error::Usb(e)
            })?;
        Ok(buf[0])
    }

    fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()> {
        let index = 0x10 | (page as u16);
        let addr_val = (addr << 8) | 0x20;
        self.handle
            .write_control(0x40, BREQUEST, addr_val, index, &[val], TIMEOUT)
            .map_err(|e| {
                error!(
                    "demod_write_reg(p={}, a=0x{:04x}) failed: {:?}",
                    page, addr, e
                );
                Error::Usb(e)
            })?;
        // Sync read — matches librtlsdr: rtlsdr_demod_read_reg(dev, 0x0a, 0x01, 1)
        let _ = self.demod_read_reg(0x0a, 0x01);
        Ok(())
    }

    fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()> {
        let index = 0x10 | (page as u16);
        let addr_val = (addr << 8) | 0x20;
        // DEMOD block is BIG ENDIAN for 16-bit writes
        let data = [(val >> 8) as u8, (val & 0xff) as u8];
        self.handle
            .write_control(0x40, BREQUEST, addr_val, index, &data, TIMEOUT)
            .map_err(|e| {
                error!(
                    "demod_write_reg16(p={}, a=0x{:04x}) failed: {:?}",
                    page, addr, e
                );
                Error::Usb(e)
            })?;
        let _ = self.demod_read_reg(0x0a, 0x01);
        Ok(())
    }

    fn demod_read_reg(&self, page: u8, addr: u16) -> Result<u8> {
        let index = page as u16;
        let addr_val = (addr << 8) | 0x20;
        let mut buf = [0u8; 1];
        self.handle
            .read_control(0xc0, BREQUEST, addr_val, index, &mut buf, TIMEOUT)
            .map_err(|e| {
                error!(
                    "demod_read_reg(p={}, a=0x{:04x}) failed: {:?}",
                    page, addr, e
                );
                Error::Usb(e)
            })?;
        Ok(buf[0])
    }

    fn set_i2c_repeater(&self, on: bool) -> Result<()> {
        let val = if on { 0x18 } else { 0x10 };
        self.demod_write_reg(1, 0x01, val)
    }

    fn i2c_write_raw(&self, addr: u8, data: &[u8]) -> Result<()> {
        let index = (block::I2C << 8) | 0x10;
        self.handle
            .write_control(0x40, BREQUEST, addr as u16, index, data, TIMEOUT)
            .map_err(|e| {
                debug!("i2c_write_raw(addr=0x{:02x}) failed: {:?}", addr, e);
                Error::Usb(e)
            })?;
        Ok(())
    }

    fn i2c_read_raw(&self, addr: u8, len: usize) -> Result<Vec<u8>> {
        let index = block::I2C << 8;
        let mut buf = vec![0u8; len];
        self.handle
            .read_control(0xc0, BREQUEST, addr as u16, index, &mut buf, TIMEOUT)
            .map_err(|e| {
                debug!("i2c_read_raw(addr=0x{:02x}) failed: {:?}", addr, e);
                Error::Usb(e)
            })?;
        Ok(buf)
    }

    fn i2c_write_tuner(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.set_i2c_repeater(true)?;
        let mut res = Ok(());
        if data.is_empty() {
            res = self.i2c_write_raw(addr, &[reg]);
        } else {
            let mut pos = 0;
            let mut current_reg = reg;
            while pos < data.len() {
                let chunk_len = (data.len() - pos).min(7);
                let mut buf = Vec::with_capacity(1 + chunk_len);
                buf.push(current_reg);
                buf.extend_from_slice(&data[pos..pos + chunk_len]);
                if let Err(e) = self.i2c_write_raw(addr, &buf) {
                    res = Err(e);
                    break;
                }
                pos += chunk_len;
                current_reg += chunk_len as u8;
            }
        }
        let _ = self.set_i2c_repeater(false);
        res
    }

    fn i2c_read_tuner(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        self.set_i2c_repeater(true)?;
        let data = match self.i2c_write_raw(addr, &[reg]) {
            Ok(_) => self.i2c_read_raw(addr, len),
            Err(e) => Err(e),
        };
        let _ = self.set_i2c_repeater(false);
        data
    }

    fn i2c_read_direct(&self, addr: u8, len: usize) -> Result<Vec<u8>> {
        self.set_i2c_repeater(true)?;
        let data = self.i2c_read_raw(addr, len);
        let _ = self.set_i2c_repeater(false);
        data
    }

    fn read_bulk(&self, ep: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.handle
            .read_bulk(ep, buf, timeout)
            .map_err(Error::Usb)
    }

    fn set_gpio_output(&self, gpio: u8) -> Result<()> {
        let bit = 1u8 << gpio;
        let val = self.read_reg(block::SYS, registers::sys::GPOE)?;
        self.write_reg(block::SYS, registers::sys::GPOE, val | bit)?;
        let val = self.read_reg(block::SYS, registers::sys::GPD)?;
        self.write_reg(block::SYS, registers::sys::GPD, val & !bit)?;
        Ok(())
    }

    fn set_gpio_bit(&self, gpio: u8, value: bool) -> Result<()> {
        let bit = 1u8 << gpio;
        let val = self.read_reg(block::SYS, registers::sys::GPO)?;
        let new = if value { val | bit } else { val & !bit };
        self.write_reg(block::SYS, registers::sys::GPO, new)?;
        Ok(())
    }

    fn probe_tuner(&self) -> Result<TunerType> {
        self.set_i2c_repeater(true)?;
        let mut found = TunerType::Unknown(0);
        // Probing 0x34
        if let Ok(res) = self.i2c_read_tuner(registers::tuner_ids::R82XX_I2C_ADDR, 0x00, 1)
            && res[0] == 0x69
        {
            found = TunerType::R820T;
        }
        // Probing 0x74
        if matches!(found, TunerType::Unknown(_))
            && let Ok(res) = self.i2c_read_tuner(registers::tuner_ids::R828D_I2C_ADDR, 0x00, 1)
            && res[0] == 0x69
        {
            found = TunerType::R828D;
        }
        if matches!(found, TunerType::Unknown(_)) {
            let _ = self.set_gpio_output(4);
            let _ = self.set_gpio_bit(4, true);
            let _ = self.set_gpio_bit(4, false);
        }
        let _ = self.set_i2c_repeater(false);
        Ok(found)
    }
}

impl Device<Context> {
    pub fn open() -> Result<Self> {
        let context = Context::new()?;
        let devices = context.devices()?;
        for dev in devices.iter() {
            let desc = dev.device_descriptor()?;
            if desc.vendor_id() != 0x0bda
                || (desc.product_id() != 0x2838 && desc.product_id() != 0x2832)
            {
                continue;
            }
            let handle = dev.open()?;
            #[cfg(target_os = "linux")]
            {
                let _ = handle.set_auto_detach_kernel_driver(true);
            }
            handle.claim_interface(0)?;
            return Ok(Self {
                handle,
                _context: context,
            });
        }
        Err(Error::NotFound)
    }
}

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub product: String,
    pub is_v4: bool,
}
impl DeviceInfo {
    pub fn probe<T: UsbContext>(handle: &DeviceHandle<T>) -> Self {
        let mut info = Self {
            manufacturer: String::new(),
            product: String::new(),
            is_v4: false,
        };
        if let Ok(desc) = handle.device().device_descriptor() {
            if let Ok(m) = handle.read_manufacturer_string_ascii(&desc) {
                info.manufacturer = m;
            }
            if let Ok(p) = handle.read_product_string_ascii(&desc) {
                info.product = p;
            }
        }
        info.is_v4 = info.manufacturer.contains("RTLSDRBlog") && info.product.contains("V4");
        info
    }
}

pub struct TransportBuffer<T: UsbContext> {
    data: Vec<u8>,
    _phantom: std::marker::PhantomData<T>,
}
impl<T: UsbContext> TransportBuffer<T> {
    pub fn new(_dev: Arc<Device<T>>, len: usize) -> Self {
        Self {
            data: vec![0u8; len],
            _phantom: std::marker::PhantomData,
        }
    }
}
impl<T: UsbContext> std::ops::Deref for TransportBuffer<T> {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        &self.data
    }
}
impl<T: UsbContext> std::ops::DerefMut for TransportBuffer<T> {
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
}

#[cfg(test)]
pub struct MockHardware;
#[cfg(test)]
impl HardwareInterface for MockHardware {
    fn write_reg(&self, _: u16, _: u16, _: u8) -> Result<()> {
        Ok(())
    }
    fn write_reg16(&self, _: u16, _: u16, _: u16) -> Result<()> {
        Ok(())
    }
    fn read_reg(&self, _: u16, _: u16) -> Result<u8> {
        Ok(0)
    }
    fn demod_write_reg(&self, _: u8, _: u16, _: u8) -> Result<()> {
        Ok(())
    }
    fn demod_write_reg16(&self, _: u8, _: u16, _: u16) -> Result<()> {
        Ok(())
    }
    fn demod_read_reg(&self, _: u8, _: u16) -> Result<u8> {
        Ok(0)
    }
    fn set_i2c_repeater(&self, _: bool) -> Result<()> {
        Ok(())
    }
    fn i2c_write_raw(&self, _: u8, _: &[u8]) -> Result<()> {
        Ok(())
    }
    fn i2c_read_raw(&self, _: u8, _: usize) -> Result<Vec<u8>> {
        Ok(vec![0])
    }
    fn i2c_write_tuner(&self, _: u8, _: u8, _: &[u8]) -> Result<()> {
        Ok(())
    }
    fn i2c_read_tuner(&self, _: u8, _: u8, _: usize) -> Result<Vec<u8>> {
        Ok(vec![0])
    }
    fn i2c_read_direct(&self, _: u8, _: usize) -> Result<Vec<u8>> {
        Ok(vec![0])
    }
    fn read_bulk(&self, _: u8, _: &mut [u8], _: Duration) -> Result<usize> {
        Ok(0)
    }
    fn set_gpio_output(&self, _: u8) -> Result<()> {
        Ok(())
    }
    fn set_gpio_bit(&self, _: u8, _: bool) -> Result<()> {
        Ok(())
    }
    fn probe_tuner(&self) -> Result<TunerType> {
        Ok(TunerType::R828D)
    }
}
