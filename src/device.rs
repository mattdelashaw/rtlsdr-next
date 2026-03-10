//! RTL2832U USB device driver.
//!
//! Control transfer encoding verified against rtl-sdr-blog librtlsdr.c:
//!
//!   Regular regs (USB/SYS blocks):
//!     write: bmRT=0x40, bReq=0, wValue=addr,           wIndex=(block<<8)|0x10, data=[val]
//!     read:  bmRT=0xc0, bReq=0, wValue=addr,           wIndex=(block<<8)
//!
//!   Demod regs (paged, DEMOD block=0):
//!     write: bmRT=0x40, bReq=0, wValue=(addr<<8)|0x20, wIndex=(0<<8)|0x10|page, data=[val]
//!     read:  bmRT=0xc0, bReq=0, wValue=(addr<<8)|0x20, wIndex=(0<<8)|page
//!
//!   I2C (IICB block=6):
//!     write: bmRT=0x40, bReq=0, wValue=i2c_addr,       wIndex=(6<<8)|0x10, data=[reg, bytes...]
//!     read:  write [reg] first, then read wValue=i2c_addr, wIndex=(6<<8)

use std::sync::Arc;
use std::time::Duration;
use rusb::{Context, DeviceHandle, UsbContext};
use log::debug;

use crate::error::{Error, Result};
use crate::registers::{self, BREQUEST, block};
use crate::tuner::TunerType;

const TIMEOUT: Duration = Duration::from_millis(200);

// ============================================================
// HardwareInterface trait
// ============================================================

pub trait HardwareInterface: Send + Sync {
    // Regular registers (USB / SYS blocks)
    fn write_reg(&self, blk: u16, addr: u16, val: u8) -> Result<()>;
    fn write_reg16(&self, blk: u16, addr: u16, val: u16) -> Result<()>;
    fn read_reg(&self, blk: u16, addr: u16) -> Result<u8>;

    // Demodulator registers (paged, DEMOD block = 0)
    fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()>;
    fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()>;
    fn demod_read_reg(&self, page: u8, addr: u16) -> Result<u8>;

    // I2C — IICB block (block=6)
    // i2c_write_tuner / i2c_read_tuner: bracket with I2C repeater ON/OFF
    // i2c_read_raw: presence probe with no reg write first
    fn i2c_write_tuner(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()>;
    fn i2c_read_tuner(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>>;
    fn i2c_read_raw(&self, addr: u8, len: usize) -> Result<Vec<u8>>;

    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    fn set_gpio_output(&self, gpio: u8) -> Result<()>;
    fn set_gpio_bit(&self, gpio: u8, value: bool) -> Result<()>;
    fn probe_tuner(&self) -> Result<TunerType>;
}

// ============================================================
// Device struct
// ============================================================

pub struct Device<T: UsbContext> {
    handle:   DeviceHandle<T>,
    pub info: DeviceInfo,
    _context: T,
}

impl<T: UsbContext> Device<T> {
    pub fn as_hw(&self) -> &dyn HardwareInterface { self }
}

// ============================================================
// HardwareInterface impl
// ============================================================

impl<T: UsbContext> HardwareInterface for Device<T> {

    // ── Regular register read / write ────────────────────────────────────
    // wValue = addr, wIndex = (block<<8) | 0x10, data = [val]

    fn write_reg(&self, blk: u16, addr: u16, val: u8) -> Result<()> {
        self.handle.write_control(
            0x40, BREQUEST,
            addr,
            (blk << 8) | 0x10,
            &[val],
            TIMEOUT,
        )?;
        Ok(())
    }

    fn write_reg16(&self, blk: u16, addr: u16, val: u16) -> Result<()> {
        // High byte first — matches librtlsdr write_reg(dev, block, addr, val, 2)
        let data = [(val >> 8) as u8, (val & 0xff) as u8];
        self.handle.write_control(
            0x40, BREQUEST,
            addr,
            (blk << 8) | 0x10,
            &data,
            TIMEOUT,
        )?;
        Ok(())
    }

    fn read_reg(&self, blk: u16, addr: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.handle.read_control(
            0xc0, BREQUEST,
            addr,
            blk << 8,
            &mut buf,
            TIMEOUT,
        )?;
        Ok(buf[0])
    }

    // ── Demod register read / write ──────────────────────────────────────
    // wValue = (addr<<8)|0x20, wIndex = (DEMOD<<8)|0x10|page
    // DEMOD block = 0, so wIndex = 0x10|page for writes, page for reads

    fn demod_write_reg(&self, page: u8, addr: u16, val: u8) -> Result<()> {
        self.handle.write_control(
            0x40, BREQUEST,
            (addr << 8) | 0x20,
            (block::DEMOD << 8) | 0x10 | (page as u16),
            &[val],
            TIMEOUT,
        )?;
        // Dummy read after every demod write — matches librtlsdr rtlsdr_demod_write_reg()
        // which always calls rtlsdr_demod_read_reg(dev, 0x0a, 0x01, 1) as a flush/sync.
        // Without this, subsequent control transfers can stall (Pipe error).
        let mut _buf = [0u8; 1];
        let _ = self.handle.read_control(
            0xc0, BREQUEST,
            (0x01u16 << 8) | 0x20,
            (block::DEMOD << 8) | 0x0a,
            &mut _buf,
            TIMEOUT,
        );
        Ok(())
    }

    fn demod_write_reg16(&self, page: u8, addr: u16, val: u16) -> Result<()> {
        let data = [(val >> 8) as u8, (val & 0xff) as u8];
        self.handle.write_control(
            0x40, BREQUEST,
            (addr << 8) | 0x20,
            (block::DEMOD << 8) | 0x10 | (page as u16),
            &data,
            TIMEOUT,
        )?;
        // Dummy read after every demod write — matches librtlsdr rtlsdr_demod_write_reg()
        let mut _buf = [0u8; 1];
        let _ = self.handle.read_control(
            0xc0, BREQUEST,
            (0x01u16 << 8) | 0x20,
            (block::DEMOD << 8) | 0x0a,
            &mut _buf,
            TIMEOUT,
        );
        Ok(())
    }

    fn demod_read_reg(&self, page: u8, addr: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.handle.read_control(
            0xc0, BREQUEST,
            (addr << 8) | 0x20,
            (block::DEMOD << 8) | (page as u16),
            &mut buf,
            TIMEOUT,
        )?;
        Ok(buf[0])
    }

    // ── I2C ─────────────────────────────────────────────────────────────
    // IICB = block 6
    // write: wValue=i2c_addr, wIndex=(6<<8)|0x10, data=[reg, bytes...]
    // read:  write [reg] first to wIndex=(6<<8)|0x10,
    //        then read from wValue=i2c_addr, wIndex=(6<<8)

    fn i2c_write_tuner(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        use registers::demod::{P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_ON, P1_IIC_REPEAT_OFF};

        // librtlsdr max_i2c_msg_len = 8, so max 7 bytes of data per transfer
        // (1 byte reg + 7 bytes data = 8 bytes total)
        const MAX_DATA: usize = 7;

        self.demod_write_reg(P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_ON)?;

        let mut pos = 0usize;
        let mut current_reg = reg;
        let mut last_err = Ok(());

        while pos < data.len() {
            let chunk_len = (data.len() - pos).min(MAX_DATA);
            let chunk = &data[pos..pos + chunk_len];

            let mut buf = Vec::with_capacity(1 + chunk_len);
            buf.push(current_reg);
            buf.extend_from_slice(chunk);

            let r = self.handle.write_control(
                0x40, BREQUEST,
                addr as u16,
                (block::I2C << 8) | 0x10,
                &buf,
                TIMEOUT,
            );

            if let Err(e) = r {
                last_err = Err(Error::Usb(e));
                break;
            }

            pos += chunk_len;
            current_reg += chunk_len as u8;
        }

        self.demod_write_reg(P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_OFF)?;
        last_err
    }

    fn i2c_read_tuner(&self, addr: u8, reg: u8, len: usize) -> Result<Vec<u8>> {
        use registers::demod::{P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_ON, P1_IIC_REPEAT_OFF};

        self.demod_write_reg(P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_ON)?;

        // Write the register address byte
        let r1 = self.handle.write_control(
            0x40, BREQUEST,
            addr as u16,
            (block::I2C << 8) | 0x10,
            &[reg],
            TIMEOUT,
        );

        // Read the response
        let mut buf = vec![0u8; len];
        let r2 = if r1.is_ok() {
            self.handle.read_control(
                0xc0, BREQUEST,
                addr as u16,
                block::I2C << 8,
                &mut buf,
                TIMEOUT,
            ).map(|_| ()).map_err(rusb::Error::from)
        } else {
            Err(r1.unwrap_err())
        };

        self.demod_write_reg(P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_OFF)?;
        r2.map_err(Error::Usb)?;
        Ok(buf)
    }

    /// Presence probe — just attempt a read with no reg write first.
    /// Used by probe_tuner to check if an I2C address responds at all.
    fn i2c_read_raw(&self, addr: u8, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        self.handle.read_control(
            0xc0, BREQUEST,
            addr as u16,
            block::I2C << 8,
            &mut buf,
            TIMEOUT,
        )?;
        Ok(buf)
    }

    // ── Bulk read ────────────────────────────────────────────────────────

    fn read_bulk(&self, ep: u8, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        Ok(self.handle.read_bulk(ep, buf, timeout)?)
    }

    // ── GPIO ─────────────────────────────────────────────────────────────

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

    // ── Tuner probe ──────────────────────────────────────────────────────
    // Matches librtlsdr open sequence:
    //   rtlsdr_set_i2c_repeater(dev, 1)
    //   ... probe all tuner addresses ...
    //   rtlsdr_set_i2c_repeater(dev, 0)

    fn probe_tuner(&self) -> Result<TunerType> {
        use registers::demod::{P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_ON, P1_IIC_REPEAT_OFF};

        self.demod_write_reg(P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_ON)?;

        let mut found = TunerType::Unknown(0);

        if self.i2c_read_raw(registers::tuner_ids::R82XX_I2C_ADDR, 1).is_ok() {
            found = TunerType::R820T;
        }

        if let TunerType::Unknown(_) = found {
            if self.i2c_read_raw(registers::tuner_ids::E4000_I2C_ADDR, 1).is_ok() {
                found = TunerType::E4000;
            }
        }

        if let TunerType::Unknown(_) = found {
            if self.i2c_read_raw(registers::tuner_ids::FC0012_I2C_ADDR, 1).is_ok() {
                found = TunerType::FC0012;
            }
        }

        let _ = self.demod_write_reg(P1_PAGE, P1_IIC_REPEAT, P1_IIC_REPEAT_OFF);

        debug!("probe_tuner: {:?}", found);
        Ok(found)
    }
}

// ============================================================
// Device::open
// ============================================================

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
                // auto_detach cleanly removes dvb_usb_rtl28xxu if loaded
                let _ = handle.set_auto_detach_kernel_driver(true);
            }

            handle.claim_interface(0)?;

            let info = DeviceInfo::probe(&handle);
            log::info!(
                "Found RTL2832U — manufacturer: {:?}  product: {:?}  is_v4: {}",
                info.manufacturer, info.product, info.is_v4
            );

            return Ok(Self { handle, info, _context: context });
        }

        Err(Error::NotFound)
    }
}

// ============================================================
// DeviceInfo
// ============================================================

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub product:      String,
    pub is_v4:        bool,
}

impl DeviceInfo {
    fn probe<T: UsbContext>(handle: &DeviceHandle<T>) -> Self {
        let mut info = Self {
            manufacturer: String::new(),
            product:      String::new(),
            is_v4:        false,
        };
        if let Ok(desc) = handle.device().device_descriptor() {
            if let Ok(m) = handle.read_manufacturer_string_ascii(&desc) { info.manufacturer = m; }
            if let Ok(p) = handle.read_product_string_ascii(&desc)      { info.product = p; }
        }
        info.is_v4 = info.manufacturer.contains("RTLSDRBlog")
                  && info.product.contains("V4");
        info
    }
}

// ============================================================
// TransportBuffer
// ============================================================

pub struct TransportBuffer<T: UsbContext> {
    data:     Vec<u8>,
    _phantom: std::marker::PhantomData<T>,
}

impl<T: UsbContext> TransportBuffer<T> {
    pub fn new(_dev: Arc<Device<T>>, len: usize) -> Self {
        Self { data: vec![0u8; len], _phantom: std::marker::PhantomData }
    }
    pub fn as_slice(&self)         -> &[u8]     { &self.data }
    pub fn as_mut_slice(&mut self) -> &mut [u8] { &mut self.data }
}

impl<T: UsbContext> std::ops::Deref for TransportBuffer<T> {
    type Target = [u8];
    fn deref(&self) -> &[u8] { &self.data }
}
impl<T: UsbContext> std::ops::DerefMut for TransportBuffer<T> {
    fn deref_mut(&mut self) -> &mut [u8] { &mut self.data }
}
