//! RTL2832U register addresses and block IDs.
//! Verified against rtl-sdr-blog librtlsdr.c

pub const BREQUEST: u8 = 0;

// USB control transfer block IDs (wIndex upper byte)
// Source: librtlsdr.c enum rtlsdr_block
pub mod block {
    pub const DEMOD: u16 = 0; // DEMODB = 0
    pub const USB: u16 = 1; // USBB   = 1
    pub const SYS: u16 = 2; // SYSB   = 2
    pub const I2C: u16 = 6; // IICB   = 6
}

pub mod sys {
    pub const DEMOD_CTL: u16 = 0x3000;
    pub const DEMOD_CTL1: u16 = 0x300b;
    pub const GPO: u16 = 0x3001;
    pub const GPOE: u16 = 0x3003;
    pub const GPD: u16 = 0x3004;
    pub const DEMOD_CTL_POWER_ON: u8 = 0xe8;
    pub const DEMOD_CTL_POWER_OFF: u8 = 0x20;
}

pub mod usb {
    pub const SYSCTL: u16 = 0x2000;
    pub const EPA_MAXPKT: u16 = 0x2158;
    pub const EPA_CTL: u16 = 0x2148;
}

pub mod demod {
    pub const P0_PAGE: u8 = 0;
    pub const P1_PAGE: u8 = 1;
    pub const P2_PAGE: u8 = 2;
    pub const P0_AGC_CTL: u16 = 0x0019;
    pub const P1_IIC_REPEAT: u16 = 0x0001;
    pub const P1_IIC_REPEAT_ON: u8 = 0x18;
    pub const P1_IIC_REPEAT_OFF: u8 = 0x10;
    pub const P1_SOFT_RESET: u16 = 0x0001;
    pub const P1_SOFT_RESET_ON: u8 = 0x14;
    pub const P1_SOFT_RESET_OFF: u8 = 0x10;
    // IF frequency registers (page 1)
    pub const P1_IF_FREQ_H: u16 = 0x0019;
    pub const P1_IF_FREQ_M: u16 = 0x001a;
    pub const P1_IF_FREQ_L: u16 = 0x001b;
    // Resampler registers (page 2)
    pub const P2_RESAMPLE_H: u16 = 0x0022;
    pub const P2_RESAMPLE_M: u16 = 0x0023;
    pub const P2_RESAMPLE_L: u16 = 0x0024;
    pub const P2_RESAMPLE_LSB: u16 = 0x0025;
}

pub mod tuner_ids {
    pub const R82XX_I2C_ADDR: u8 = 0x34;
    pub const R828D_I2C_ADDR: u8 = 0x74;
    pub const E4000_I2C_ADDR: u8 = 0xc8;
    pub const FC0012_I2C_ADDR: u8 = 0xc2;
    pub const FC0013_I2C_ADDR: u8 = 0xc6;
}

pub const IF_FREQ_HZ: u64 = 3_570_000;
pub const XTAL_FREQ_HZ: u32 = 28_800_000;

pub fn if_freq_regs(if_hz: u32, xtal_hz: u32) -> [u8; 3] {
    let ratio = (if_hz as u64) * (1u64 << 22) / xtal_hz as u64;
    [
        ((ratio >> 16) & 0x3f) as u8,
        ((ratio >> 8) & 0xff) as u8,
        (ratio & 0xff) as u8,
    ]
}

pub fn resample_regs(sample_rate_hz: u32, xtal_hz: u32) -> [u8; 4] {
    let ratio = (xtal_hz as u64) * (1u64 << 22) / sample_rate_hz as u64;
    [
        ((ratio >> 24) & 0x3f) as u8,
        ((ratio >> 16) & 0xff) as u8,
        ((ratio >> 8) & 0xff) as u8,
        (ratio & 0xff) as u8,
    ]
}
