use crate::device::HardwareInterface;
use crate::tuner::{Tuner, FilterRange};
use crate::error::{Error, Result};
use std::sync::{Arc, Mutex};

const I2C_ADDR: u8 = 0x34;
const NUM_REGS: usize = 27;

static INIT_ARRAY: [u8; NUM_REGS] = [
    0x83, 0x30, 0x75, // 05 to 07
    0xc0, 0x40, 0xd6, 0x6c, // 08 to 0b
    0xf5, 0x63, 0x75, 0x68, // 0c to 0f
    0x6c, 0x83, 0x80, 0x00, // 10 to 13
    0x0f, 0x00, 0xc0, 0x30, // 14 to 17
    0x48, 0xcc, 0x60, 0x00, // 18 to 1b
    0x54, 0xae, 0x4a, 0xc0, // 1c to 1f
];

pub struct R828D {
    device: Arc<dyn HardwareInterface>,
    regs: Mutex<[u8; NUM_REGS]>,
    is_v4: bool,
}

impl R828D {
    pub fn new(device: Arc<dyn HardwareInterface>, is_v4: bool) -> Self {
        Self {
            device,
            regs: Mutex::new(INIT_ARRAY),
            is_v4,
        }
    }

    fn write_reg_mask(&self, reg: u8, val: u8, mask: u8) -> Result<()> {
        let mut regs = self.regs.lock().unwrap();
        let idx = (reg - 0x05) as usize;
        if idx >= NUM_REGS {
            return Err(Error::Tuner(format!("Invalid register 0x{:02x}", reg)));
        }

        let old = regs[idx];
        let new = (old & !mask) | (val & mask);
        regs[idx] = new;
        
        self.device.i2c_write(I2C_ADDR, reg, &[new])
    }
}

impl Tuner for R828D {
    fn initialize(&self) -> Result<()> {
        self.device.i2c_write(I2C_ADDR, 0x05, &INIT_ARRAY)?;
        self.write_reg_mask(0x0c, 0x00, 0x0f)?; // Init flag
        self.write_reg_mask(0x13, 0x01, 0x3f)?; // Version
        Ok(())
    }

    fn set_frequency(&self, hz: u64) -> Result<u64> {
        if self.is_v4 {
            if hz <= 28_800_000 {
                // HF Band
                self.write_reg_mask(0x06, 0x08, 0x08)?; // Cable 2
                self.write_reg_mask(0x05, 0x00, 0x40)?; // Cable 1 OFF
                self.write_reg_mask(0x05, 0x20, 0x20)?; // Air OFF
            } else if hz < 250_000_000 {
                // VHF Band
                self.write_reg_mask(0x06, 0x00, 0x08)?; // Cable 2 OFF
                self.write_reg_mask(0x05, 0x40, 0x40)?; // Cable 1 ON
                self.write_reg_mask(0x05, 0x20, 0x20)?; // Air OFF
            } else {
                // UHF Band
                self.write_reg_mask(0x06, 0x00, 0x08)?; // Cable 2 OFF
                self.write_reg_mask(0x05, 0x00, 0x40)?; // Cable 1 OFF
                self.write_reg_mask(0x05, 0x00, 0x20)?; // Air ON
            }
        }
        Ok(hz)
    }

    fn set_gain(&self, _db: f32) -> Result<f32> { Ok(0.0) }
    fn get_filters(&self) -> Vec<FilterRange> { vec![] }
    fn set_bias_t(&self, _on: bool) -> Result<()> { Ok(()) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::Block;
    use std::time::Duration;

    struct MockHardware {
        writes: Mutex<Vec<(u8, u8, Vec<u8>)>>,
    }

    impl HardwareInterface for MockHardware {
        fn read_reg(&self, _: Block, _: u16) -> Result<u8> { Ok(0) }
        fn write_reg(&self, _: Block, _: u16, _: u8) -> Result<()> { Ok(()) }
        fn i2c_read(&self, _: u8, _: u8, len: usize) -> Result<Vec<u8>> { Ok(vec![0; len]) }
        fn i2c_write(&self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
            self.writes.lock().unwrap().push((addr, reg, data.to_vec()));
            Ok(())
        }
        fn read_bulk(&self, _: u8, _: &mut [u8], _: Duration) -> Result<usize> { Ok(0) }
    }

    #[test]
    fn test_band_switching_v4() {
        let mock = Arc::new(MockHardware { writes: Mutex::new(vec![]) });
        let tuner = R828D::new(mock.clone(), true);

        // Test HF Switching (10 MHz)
        tuner.set_frequency(10_000_000).unwrap();
        
        let writes = mock.writes.lock().unwrap();
        // Check if register 0x06 had bit 0x08 set
        let reg_06_write = writes.iter().find(|(_, reg, _)| *reg == 0x06);
        assert!(reg_06_write.is_some());
        assert_eq!(reg_06_write.unwrap().2[0] & 0x08, 0x08);
    }
}
