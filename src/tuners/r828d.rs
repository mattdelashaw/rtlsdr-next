use crate::device::HardwareInterface;
use crate::tuner::{Tuner, FilterRange};
use crate::error::{Error, Result};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use log::warn;

const I2C_ADDR: u8 = 0x74;
const NUM_REGS: usize = 27;
const REG_SHADOW_START: u8 = 0x05;

const VCO_MIN: u64 = 1_770_000_000;
const VCO_MAX: u64 = 3_600_000_000;

const IF_FREQ_WIDE: u64 = 3_570_000;
const IF_FREQ_NARROW: u64 = 2_300_000;

const GAIN_STEPS: [i32; 29] = [
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
    166, 197, 207, 229, 254, 280, 297, 328,
    338, 364, 372, 386, 402, 421, 434, 439,
    445, 480, 496,
];

struct GainEntry { lna: u8, mix: u8, vga: u8 }

static GAIN_TABLE: [GainEntry; 29] = [
    GainEntry { lna: 0,  mix: 0,  vga: 0  }, GainEntry { lna: 1,  mix: 0,  vga: 2  },
    GainEntry { lna: 2,  mix: 0,  vga: 2  }, GainEntry { lna: 3,  mix: 0,  vga: 2  },
    GainEntry { lna: 4,  mix: 1,  vga: 3  }, GainEntry { lna: 5,  mix: 1,  vga: 3  },
    GainEntry { lna: 6,  mix: 2,  vga: 4  }, GainEntry { lna: 7,  mix: 2,  vga: 4  },
    GainEntry { lna: 8,  mix: 3,  vga: 5  }, GainEntry { lna: 9,  mix: 3,  vga: 5  },
    GainEntry { lna: 10, mix: 4,  vga: 6  }, GainEntry { lna: 11, mix: 4,  vga: 6  },
    GainEntry { lna: 12, mix: 5,  vga: 7  }, GainEntry { lna: 13, mix: 5,  vga: 7  },
    GainEntry { lna: 14, mix: 6,  vga: 8  }, GainEntry { lna: 15, mix: 6,  vga: 8  },
    GainEntry { lna: 15, mix: 7,  vga: 9  }, GainEntry { lna: 15, mix: 7,  vga: 9  },
    GainEntry { lna: 15, mix: 8,  vga: 10 }, GainEntry { lna: 15, mix: 8,  vga: 10 },
    GainEntry { lna: 15, mix: 9,  vga: 11 }, GainEntry { lna: 15, mix: 9,  vga: 11 },
    GainEntry { lna: 15, mix: 10, vga: 12 }, GainEntry { lna: 15, mix: 10, vga: 12 },
    GainEntry { lna: 15, mix: 11, vga: 13 }, GainEntry { lna: 15, mix: 11, vga: 13 },
    GainEntry { lna: 15, mix: 12, vga: 14 }, GainEntry { lna: 15, mix: 12, vga: 14 },
    GainEntry { lna: 15, mix: 13, vga: 15 },
];

static INIT_ARRAY: [u8; NUM_REGS] = [
    0x83, 0x30, 0x75,
    0xc0, 0x40, 0xd6, 0x6c,
    0xf5, 0x63, 0x75, 0x68,
    0x6c, 0x83, 0x80, 0x00,
    0x0f, 0x00, 0xc0, 0x30,
    0x48, 0xcc, 0x60, 0x00,
    0x54, 0xae, 0x4a, 0xc0,
];

struct FreqRange { freq_hz: u64, open_d: u8, rf_mux_ploy: u8, tf_c: u8, xtal_cap_sel: u8 }

static FREQ_RANGES: &[FreqRange] = &[
    FreqRange { freq_hz:          0, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xdf, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   50_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xbe, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   55_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x8b, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   60_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x7b, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   65_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x69, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   70_000_000, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x58, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   75_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   80_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, xtal_cap_sel: 0 },
    FreqRange { freq_hz:   90_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  100_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  110_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  120_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  140_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x14, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  180_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  220_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  250_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x11, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  280_000_000, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  310_000_000, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  450_000_000, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  588_000_000, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, xtal_cap_sel: 0 },
    FreqRange { freq_hz:  650_000_000, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, xtal_cap_sel: 0 },
];

static XTAL_CAP_SEL: [u8; 5] = [0x0b, 0x0b, 0x0b, 0x0b, 0x00];

pub struct R828D {
    device:     Arc<dyn HardwareInterface>,
    regs:       Mutex<[u8; NUM_REGS]>,
    xtal_freq:  Mutex<u64>,
    is_v4:      bool,
    has_lock:   Mutex<bool>,
    current_gain: Mutex<f32>,
    current_if:   Mutex<u64>,
}

impl R828D {
    pub fn new(device: Arc<dyn HardwareInterface>, is_v4: bool) -> Self {
        let xtal = if is_v4 { 28_800_000 } else { 16_000_000 };
        Self {
            device,
            regs:      Mutex::new(INIT_ARRAY),
            xtal_freq: Mutex::new(xtal),
            is_v4,
            has_lock:  Mutex::new(false),
            current_gain: Mutex::new(0.0),
            current_if:   Mutex::new(IF_FREQ_NARROW),
        }
    }

    fn write_reg_mask(&self, reg: u8, val: u8, mask: u8) -> Result<()> {
        let new = {
            let mut regs = self.regs.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))?;
            let idx = (reg - REG_SHADOW_START) as usize;
            if idx >= NUM_REGS { return Err(Error::Tuner(format!("Register 0x{:02x} out of range", reg))); }
            let old = regs[idx];
            let new = (old & !mask) | (val & mask);
            regs[idx] = new;
            new
        };
        self.device.i2c_write_tuner(I2C_ADDR, reg, &[new])
    }

    fn read_status(&self) -> Result<[u8; 5]> {
        self.device.i2c_write_tuner(I2C_ADDR, 0x00, &[])?;
        let mut data = self.device.i2c_read_direct(I2C_ADDR, 5)?;
        for byte in data.iter_mut() { *byte = bit_reverse(*byte); }
        Ok([data[0], data[1], data[2], data[3], data[4]])
    }

    fn wait_pll_lock(&self, retries: u32) -> Result<bool> {
        for i in 0..retries {
            self.device.i2c_write_tuner(I2C_ADDR, 0x00, &[])?;
            let mut status = self.device.i2c_read_direct(I2C_ADDR, 3)?;
            for byte in status.iter_mut() { *byte = bit_reverse(*byte); }

            if status[2] & 0x40 != 0 {
                *self.has_lock.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))? = true;
                return Ok(true);
            }
            if i == 0 {
                self.write_reg_mask(0x12, 0x06, 0xff)?;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        *self.has_lock.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))? = false;
        warn!("PLL not locked after {} retries", retries);
        Ok(false)
    }

    fn set_pll(&self, lo_freq_hz: u64) -> Result<u64> {
        let pll_ref = *self.xtal_freq.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))?;
        let pll_ref_khz = pll_ref / 1000;

        let mut mix_div: u64 = 2;
        let mut div_num: u8 = 0;
        while mix_div <= 64 {
            let vco = lo_freq_hz * mix_div;
            if vco >= VCO_MIN && vco < VCO_MAX {
                let mut div_buf = mix_div;
                while div_buf > 2 { div_buf >>= 1; div_num += 1; }
                break;
            }
            mix_div <<= 1;
        }
        if mix_div > 64 { return Err(Error::InvalidFrequency(lo_freq_hz)); }

        let vco_freq = lo_freq_hz * mix_div;
        let status = self.read_status()?;
        let vco_fine_tune = (status[4] & 0x30) >> 4;
        let vco_power_ref = if self.is_v4 { 1 } else { 2 };
        if vco_fine_tune > vco_power_ref { div_num = div_num.saturating_sub(1); }
        else if vco_fine_tune < vco_power_ref { div_num += 1; }

        self.write_reg_mask(0x10, div_num << 5, 0xe0)?;

        let nint: u64 = vco_freq / (2 * pll_ref);
        let mut vco_fra: u64 = (vco_freq - 2 * pll_ref * nint) / 1000;
        if nint > (128 / vco_power_ref as u64) - 1 { return Err(Error::InvalidFrequency(lo_freq_hz)); }

        let ni = ((nint - 13) / 4) as u8;
        let si = (nint as u8).wrapping_sub(4u8.wrapping_mul(ni).wrapping_add(13));
        self.device.i2c_write_tuner(I2C_ADDR, 0x14, &[ni | (si << 6)])?;

        let pw_sdm: u8 = if vco_fra == 0 { 0x08 } else { 0x00 };
        self.write_reg_mask(0x12, pw_sdm, 0x08)?;

        let mut sdm: u32 = 0;
        let mut n_sdm: u32 = 2;
        while vco_fra > 1 {
            if vco_fra > (2 * pll_ref_khz / n_sdm as u64) {
                sdm += 32768 / (n_sdm / 2);
                vco_fra -= 2 * pll_ref_khz / n_sdm as u64;
                if n_sdm >= 0x8000 { break; }
            }
            n_sdm <<= 1;
        }
        self.device.i2c_write_tuner(I2C_ADDR, 0x16, &[(sdm >> 8) as u8])?;
        self.device.i2c_write_tuner(I2C_ADDR, 0x15, &[(sdm & 0xff) as u8])?;

        if self.wait_pll_lock(10)? {
             self.write_reg_mask(0x1a, 0x08, 0x08)?;
        }
        Ok(lo_freq_hz)
    }

    fn set_bandwidth(&self, if_hz: u64) -> Result<()> {
        let (reg_0a, reg_0b) = if if_hz >= 3_500_000 { (0x10, 0x6b) } else { (0x00, 0x80) };
        self.write_reg_mask(0x0a, reg_0a, 0x70)?;
        self.write_reg_mask(0x0b, reg_0b, 0xef)?;
        Ok(())
    }

    fn set_mux(&self, freq_hz: u64) -> Result<()> {
        let range = FREQ_RANGES.iter().rev().find(|r| freq_hz >= r.freq_hz).unwrap_or(&FREQ_RANGES[0]);
        self.write_reg_mask(0x17, 0xa0, 0x30)?;
        
        if self.is_v4 {
            // V4 Dynamic Notch Filter Logic:
            // Turn OFF (0x00) within notch bands, ON (0x08) otherwise.
            let open_d = if freq_hz <= 2_200_000 
                || (freq_hz >= 85_000_000 && freq_hz <= 112_000_000) 
                || (freq_hz >= 172_000_000 && freq_hz <= 242_000_000) { 0x00 } else { 0x08 };
            self.write_reg_mask(0x17, open_d, 0x08)?;
        } else {
            self.write_reg_mask(0x17, range.open_d, 0x08)?;
        }

        self.write_reg_mask(0x1a, range.rf_mux_ploy, 0xc3)?;
        self.write_reg_mask(0x1b, range.tf_c, 0xff)?;
        let cap = XTAL_CAP_SEL[range.xtal_cap_sel as usize];
        self.write_reg_mask(0x10, cap, 0x0b)?;
        self.write_reg_mask(0x08, 0x00, 0x3f)?;
        self.write_reg_mask(0x09, 0x00, 0x3f)?;
        self.write_reg_mask(0x1d, 0x18, 0x38)?;
        self.write_reg_mask(0x1c, 0x24, 0x04)?;
        self.write_reg_mask(0x1e, 14, 0x1f)?;
        self.write_reg_mask(0x1a, 0x20, 0x30)?;
        Ok(())
    }

    pub(crate) fn set_v4_input(&self, freq_hz: u64) -> Result<()> {
        if freq_hz < 28_800_000 {
            // HF Input (Cable 2)
            self.write_reg_mask(0x06, 0x08, 0x08)?; // activate cable 2
            self.device.set_gpio_output(5)?;
            self.device.set_gpio_bit(5, false)?;    // control upconverter switch
            self.write_reg_mask(0x05, 0x00, 0x40)?; // deactivate cable 1 (VHF)
            self.write_reg_mask(0x05, 0x20, 0x20)?; // deactivate air_in (UHF)
        } else if freq_hz < 250_000_000 {
            // VHF Input (Cable 1)
            self.write_reg_mask(0x06, 0x00, 0x08)?; // deactivate cable 2
            self.device.set_gpio_output(5)?;
            self.device.set_gpio_bit(5, true)?;     // control upconverter switch
            self.write_reg_mask(0x05, 0x40, 0x40)?; // activate cable 1
            self.write_reg_mask(0x05, 0x20, 0x20)?; // deactivate air_in (UHF)
        } else {
            // UHF Input (Air In)
            self.write_reg_mask(0x06, 0x00, 0x08)?; // deactivate cable 2
            self.device.set_gpio_output(5)?;
            self.device.set_gpio_bit(5, true)?;     // control upconverter switch
            self.write_reg_mask(0x05, 0x00, 0x40)?; // deactivate cable 1
            self.write_reg_mask(0x05, 0x00, 0x20)?; // activate air_in
        }
        Ok(())
    }
}

impl Tuner for R828D {
    fn initialize(&self) -> Result<()> {
        let mid = 16;
        self.device.i2c_write_tuner(I2C_ADDR, REG_SHADOW_START, &INIT_ARRAY[..mid])?;
        self.device.i2c_write_tuner(I2C_ADDR, REG_SHADOW_START + mid as u8, &INIT_ARRAY[mid..])?;
        self.write_reg_mask(0x1a, 0x00, 0x0c)?;
        self.write_reg_mask(0x12, 0x06, 0xff)?;
        self.write_reg_mask(0x0c, 0x00, 0x0f)?;
        self.write_reg_mask(0x13, 0x01, 0x3f)?;
        self.set_bandwidth(IF_FREQ_NARROW)?;
        Ok(())
    }

    fn set_frequency(&self, hz: u64) -> Result<u64> {
        if hz == 0 { return Err(Error::InvalidFrequency(hz)); }
        let current_if = *self.current_if.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))?;
        let mut lo_freq = hz + current_if;
        if self.is_v4 && hz < 28_800_000 { lo_freq += 28_800_000; }
        self.set_mux(hz)?;
        if self.is_v4 { self.set_v4_input(hz)?; }
        self.set_pll(lo_freq)?;
        Ok(hz)
    }

    fn set_gain(&self, db: f32) -> Result<f32> {
        let target_tenths = (db * 10.0) as i32;
        let (idx, _) = GAIN_STEPS.iter().enumerate().min_by_key(|&(_, g)| (g - target_tenths).abs()).ok_or_else(|| Error::InvalidGain(target_tenths))?;
        let cfg = &GAIN_TABLE[idx];
        self.write_reg_mask(0x05, 0x10, 0x10)?; 
        self.write_reg_mask(0x07, 0x00, 0x10)?; 
        self.write_reg_mask(0x05, cfg.lna, 0x0f)?;
        self.write_reg_mask(0x07, cfg.mix, 0x0f)?;
        self.write_reg_mask(0x0c, cfg.vga, 0x0f)?;
        let actual = GAIN_STEPS[idx] as f32 / 10.0;
        *self.current_gain.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))? = actual;
        Ok(actual)
    }

    fn get_gain(&self) -> Result<f32> { Ok(*self.current_gain.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))?) }
    fn get_filters(&self) -> Vec<FilterRange> {
        if self.is_v4 {
            vec![
                FilterRange { start_hz: 0,           end_hz: 28_799_999   },
                FilterRange { start_hz: 28_800_000,  end_hz: 249_999_999  },
                FilterRange { start_hz: 250_000_000, end_hz: 1_766_000_000},
            ]
        } else {
            vec![FilterRange { start_hz: 0, end_hz: 1_766_000_000 }]
        }
    }
    fn set_if_freq(&self, hz: u64) -> Result<()> {
        *self.current_if.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))? = hz;
        self.set_bandwidth(hz)?;
        Ok(())
    }
    fn get_if_freq(&self) -> u64 { *self.current_if.lock().expect("Mutex poisoned") }
    fn set_ppm(&self, ppm: i32) -> Result<()> {
        let nominal = if self.is_v4 { 28_800_000u64 } else { 16_000_000u64 };
        let offset = (nominal as i64 * ppm as i64) / 1_000_000;
        let actual = (nominal as i64 + offset) as u64;
        *self.xtal_freq.lock().map_err(|e| Error::MutexPoisoned(e.to_string()))? = actual;
        Ok(())
    }
}

fn bit_reverse(mut b: u8) -> u8 {
    b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
    b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
    b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
    b
}
