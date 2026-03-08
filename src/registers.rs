//! RTL2832U register map and USB control transfer helpers.
//!
//! # Control transfer encoding
//!
//! The RTL2832U exposes two distinct register namespaces over USB vendor
//! control requests, with **different encodings**:
//!
//! ## Regular registers (USB / SYS blocks)
//! ```text
//! write: bmRequestType=0x40, bRequest=0, wValue=addr, wIndex=(block<<8)|0x10, data=[val]
//! read:  bmRequestType=0xC0, bRequest=0, wValue=addr, wIndex=(block<<8),      data=buf
//! ```
//! Block IDs: USB=1, SYS=2
//!
//! ## Demodulator registers (paged)
//! ```text
//! write: bmRequestType=0x40, bRequest=0, wValue=(addr<<8)|0x20, wIndex=0x10|page, data=[val]
//! read:  bmRequestType=0xC0, bRequest=0, wValue=(addr<<8)|0x20, wIndex=page,      data=buf
//! ```
//!
//! Source: rtl-sdr-blog librtlsdr.c — rtlsdr_write_reg / rtlsdr_demod_write_reg.

/// All RTL2832U vendor control transfers use bRequest=0.
pub const BREQUEST: u8 = 0;

// ============================================================
// Block IDs
// ============================================================

pub mod block {
    pub const DEMOD: u8 = 0;
    pub const USB:   u8 = 1;
    pub const SYS:   u8 = 2;
}

// ============================================================
// USB block registers
// ============================================================

pub mod usb {
    pub const SYSCTL:       u16 = 0x2000;
    pub const EPA_CFG:      u16 = 0x2144;
    pub const EPA_CTL:      u16 = 0x2148;
    pub const EPA_MAXPKT:   u16 = 0x2158;
    pub const EPA_MAXPKT2:  u16 = 0x215a;
    pub const EPA_FIFO_CFG: u16 = 0x2160;
}

// ============================================================
// SYS block registers
// ============================================================

pub mod sys {
    pub const DEMOD_CTL:  u16 = 0x3000;
    pub const GPO:        u16 = 0x3001;
    pub const GPI:        u16 = 0x3002;
    pub const GPOE:       u16 = 0x3003;
    pub const GPD:        u16 = 0x3004;
    pub const DEMOD_CTL1: u16 = 0x300b;

    pub const DEMOD_CTL_POWER_ON:  u8 = 0xe8;
    pub const DEMOD_CTL_POWER_OFF: u8 = 0x20;
    pub const GPIO_BIAS_T:         u8 = 0x01;
}

// ============================================================
// Demodulator registers (paged — use demod_write/read_reg)
// ============================================================

pub mod demod {
    // Page 0
    pub const P0_PAGE:       u8  = 0;
    pub const P0_IIC_REPEAT: u16 = 0x0008;
    pub const P0_ADC_CLK:    u16 = 0x000d;

    // Page 1
    pub const P1_PAGE:        u8  = 1;
    pub const P1_SOFT_RESET:  u16 = 0x0001;
    pub const P1_SOFT_RESET_ON:  u8 = 0x14;
    pub const P1_SOFT_RESET_OFF: u8 = 0x10;
    pub const P1_SPEC_INV:    u16 = 0x0015;
    pub const P1_ADJ_CHAN:    u16 = 0x0016;
    pub const P1_IF_FREQ_H:   u16 = 0x0019;
    pub const P1_IF_FREQ_M:   u16 = 0x001a;
    pub const P1_IF_FREQ_L:   u16 = 0x001b;

    // Page 2
    pub const P2_PAGE:         u8  = 2;
    pub const P2_RESAMPLE_H:   u16 = 0x0022;
    pub const P2_RESAMPLE_M:   u16 = 0x0023;
    pub const P2_RESAMPLE_L:   u16 = 0x0024;
    pub const P2_RESAMPLE_LSB: u16 = 0x0025;

    // Page 3
    pub const P3_PAGE:        u8  = 3;
    pub const P3_DAGC_TARGET: u16 = 0x001a;
}

// ============================================================
// Tuner Identification
// ============================================================

pub mod tuner_ids {
    pub const R82XX_I2C_ADDR:   u8 = 0x34;
    pub const R82XX_CHECK_REG:  u8 = 0x00;

    pub const E4000_I2C_ADDR:   u8 = 0xc8;
    pub const E4000_CHECK_REG:  u8 = 0x02;
    pub const E4000_CHECK_VAL:  u8 = 0x40;

    pub const FC0012_I2C_ADDR:  u8 = 0xc6;
    pub const FC0012_CHECK_REG: u8 = 0x00;
    pub const FC0012_CHECK_VAL: u8 = 0xa1;
    pub const FC0013_CHECK_VAL: u8 = 0xa3;
}

// ============================================================
// R828D tuner shadow register layout
// ============================================================

pub mod r828d {
    pub const I2C_ADDR:    u8    = 0x34;
    pub const SHADOW_START: u8   = 0x05;
    pub const SHADOW_LEN:  usize = 27;

    pub const REG_00_STATUS:   u8 = 0x00;
    pub const REG_05_LNA:      u8 = 0x05;
    pub const REG_06_CABLE:    u8 = 0x06;
    pub const REG_07_TF:       u8 = 0x07;
    pub const REG_08_MIXER:    u8 = 0x08;
    pub const REG_09_IF:       u8 = 0x09;
    pub const REG_0A_VGA:      u8 = 0x0a;
    pub const REG_0C_LNA_GAIN: u8 = 0x0c;
    pub const REG_0F_BIAS_T:   u8 = 0x0f;
    pub const REG_10_PLL_DIV:  u8 = 0x10;
    pub const REG_12_SDM_CTRL: u8 = 0x12;
    pub const REG_13_VERSION:  u8 = 0x13;
    pub const REG_14_NINT:     u8 = 0x14;
    pub const REG_15_SDM_L:    u8 = 0x15;
    pub const REG_16_SDM_H:    u8 = 0x16;
    pub const REG_17_RF_MUX:   u8 = 0x17;
    pub const REG_1A_RF_POLY:  u8 = 0x1a;
    pub const REG_1B_TF_C:     u8 = 0x1b;

    pub const STATUS_PLL_LOCK:    u8 = 0x40;
    pub const LNA_AIR_ON:         u8 = 0x00;
    pub const LNA_AIR_OFF:        u8 = 0x20;
    pub const LNA_CABLE1_ON:      u8 = 0x40;
    pub const LNA_CABLE1_OFF:     u8 = 0x00;
    pub const CABLE_CABLE2_ON:    u8 = 0x08;
    pub const CABLE_CABLE2_OFF:   u8 = 0x00;
    pub const SDM_CTRL_NO_DITHER: u8 = 0x08;
    pub const BIAS_T_ON:          u8 = 0x01;
    pub const BIAS_T_OFF:         u8 = 0x00;
}

// ============================================================
// Sample rate helpers
// ============================================================

pub const XTAL_FREQ_HZ: u32 = 28_800_000;
pub const IF_FREQ_HZ:   u64 = 3_570_000;

/// Compute the 3-byte IF frequency register value.
/// Returns `[high, mid, low]` for P1_IF_FREQ_H/M/L.
pub fn if_freq_regs(if_hz: u32, xtal_hz: u32) -> [u8; 3] {
    let ratio = (if_hz as u64) * (1u64 << 22) / xtal_hz as u64;
    [
        ((ratio >> 16) & 0x3f) as u8,
        ((ratio >>  8) & 0xff) as u8,
        ( ratio        & 0xff) as u8,
    ]
}

/// Compute the 4-byte resampler ratio.
/// Returns `[h, m, l, lsb]` for P2_RESAMPLE_H/M/L/LSB.
pub fn resample_regs(sample_rate_hz: u32, xtal_hz: u32) -> [u8; 4] {
    let ratio = (xtal_hz as u64) * (1u64 << 22) / sample_rate_hz as u64;
    [
        ((ratio >> 24) & 0x3f) as u8,
        ((ratio >> 16) & 0xff) as u8,
        ((ratio >>  8) & 0xff) as u8,
        ( ratio        & 0xff) as u8,
    ]
}

pub fn effective_sample_rate(regs: [u8; 4], xtal_hz: u32) -> u32 {
    let ratio = ((regs[0] as u64) << 24)
              | ((regs[1] as u64) << 16)
              | ((regs[2] as u64) <<  8)
              |  (regs[3] as u64);
    if ratio == 0 { return 0; }
    ((xtal_hz as u64 * (1u64 << 22)) / ratio) as u32
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_if_freq_3570khz() {
        let regs = if_freq_regs(3_570_000, XTAL_FREQ_HZ);
        let flat = ((regs[0] as u32) << 16)
                 | ((regs[1] as u32) <<  8)
                 |  (regs[2] as u32);
        let expected = (3_570_000u64 * (1u64 << 22) / 28_800_000) as u32;
        assert!(flat.abs_diff(expected) <= 1);
    }

    #[test]
    fn test_resample_roundtrip() {
        for &rate in &[250_000u32, 1_024_000, 2_048_000, 2_400_000, 3_200_000] {
            let regs      = resample_regs(rate, XTAL_FREQ_HZ);
            let effective = effective_sample_rate(regs, XTAL_FREQ_HZ);
            let err_ppm   = (effective as i64 - rate as i64).abs() * 1_000_000
                          / rate as i64;
            assert!(err_ppm < 10_000, "Rate {} Hz err={}ppm", rate, err_ppm);
        }
    }

    #[test]
    fn test_r828d_shadow_range() {
        let end: u8 = r828d::SHADOW_START + r828d::SHADOW_LEN as u8 - 1;
        assert_eq!(end, 0x1f);
    }
}
