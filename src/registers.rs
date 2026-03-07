//! RTL2832U register map, I2C repeater control, and typed USB vendor request helpers.
//!
//! # Architecture
//!
//! The RTL2832U exposes its internal registers via USB vendor control requests.
//! Every register has a block address (the `wIndex` field) and an offset
//! (the `wValue` field).  The five blocks are:
//!
//! | Block  | Base    | Contents                              |
//! |--------|---------|---------------------------------------|
//! | USB    | 0x0000  | USB endpoint / FIFO control           |
//! | SYS    | 0x0100  | GPIO, clock, power                    |
//! | IR     | 0x0200  | Infrared receiver (unused for SDR)    |
//! | DEMOD  | 0x0300  | Demodulator pages 0-3                 |
//! | I2C    | 0x0600  | I2C master / repeater                 |
//!
//! # I2C repeater
//!
//! The RTL2832U contains an I2C master that talks to the tuner chip.
//! The repeater (IIC_repeat, demod page 1 reg 0x01 bit 3) **must be
//! enabled before every tuner I2C transaction and explicitly disabled
//! afterwards** — the hardware does NOT auto-clear it after STOP.
//!
//! Sequence:
//! ```text
//! write_demod(page=1, reg=0x01, val=set bit 3)   // repeater ON
//! i2c_write / i2c_read to tuner
//! write_demod(page=1, reg=0x01, val=clear bit 3) // repeater OFF
//! ```
//!
//! # Sample rate / IF registers
//!
//! After PLL lock the demodulator IF and resampler registers must be
//! programmed so the RTL2832U digitises at the requested sample rate.
//! The helpers `if_freq_regs` and `resample_regs` produce the correct
//! byte sequences from a desired sample rate in Hz.

// ============================================================
// USB vendor request codes
// ============================================================

/// USB vendor bRequest codes used in control transfers.
///
/// RegRead/I2cRead both use opcode 0x00, RegWrite/I2cWrite both use 0x01.
/// They are differentiated by the wIndex block field, not bRequest.
/// Using constants avoids the duplicate discriminant restriction.
pub mod request {
    /// Read a register or I2C byte  (bmRequestType = 0xC0)
    pub const READ:  u8 = 0x00;
    /// Write a register or I2C byte (bmRequestType = 0x40)
    pub const WRITE: u8 = 0x01;
}

/// Compatibility shim so existing call sites using Request::RegRead etc. still compile.
pub struct Request;
#[allow(non_upper_case_globals)]
impl Request {
    pub const RegRead:  u8 = request::READ;
    pub const RegWrite: u8 = request::WRITE;
    pub const I2cRead:  u8 = request::READ;
    pub const I2cWrite: u8 = request::WRITE;
}

// ============================================================
// Block addresses  (wIndex high byte)
// ============================================================

/// RTL2832U register block selector — used as the `wIndex` field in
/// USB vendor control transfers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Block {
    /// USB endpoint / FIFO / DMA control registers
    Usb   = 0x0000,
    /// System: GPIO, oscillator, power management
    Sys   = 0x0100,
    /// Infrared receiver (not used for SDR)
    Ir    = 0x0200,
    /// Demodulator — pages 0-3.  Use `Block::demod(page)` for convenience.
    Demod = 0x0300,
    /// I2C master registers
    I2c   = 0x0600,
}

impl Block {
    /// Return the block index for demodulator page `page` (0..=3).
    ///
    /// The RTL2832U demodulator has four register pages.  Each page is
    /// addressed as `Demod` + `page * 0x100`.
    #[inline]
    pub const fn demod(page: u8) -> u16 {
        0x0300u16 + (page as u16 * 0x0100)
    }
}

// ============================================================
// USB block registers  (Block::Usb, base 0x0000)
// ============================================================

pub mod usb {
    /// USB system control (EPA stall / flush)
    pub const SYSCTL:     u16 = 0x2000;
    /// Endpoint A control — stall / flush the bulk IN endpoint
    pub const EPA_CTL:    u16 = 0x2028;
    /// Endpoint A max packet size
    pub const EPA_MAX_PKT: u16 = 0x202a;
    /// Endpoint A FIFO config
    pub const EPA_FIFO_CONFIG: u16 = 0x2030;
}

// ============================================================
// SYS block registers  (Block::Sys, base 0x0100)
// ============================================================

pub mod sys {
    /// Demodulator enable / ADC power / I2C repeater master switch
    pub const DEMOD_CTL:   u16 = 0x3000;
    /// GPIO output register
    pub const GPO:         u16 = 0x3001;
    /// GPIO input register (read-only)
    pub const GPI:         u16 = 0x3002;
    /// GPIO direction register (1 = output)
    pub const GPD:         u16 = 0x3003;
    /// System configuration (osc sel, PLL)
    pub const SYS_CONFIG:  u16 = 0x3004;
    /// Clock out amplitude
    pub const CLK_OUT_ENB: u16 = 0x3005;
    /// Demodulator control 1 (reset)
    pub const DEMOD_CTL1:  u16 = 0x300b;

    // ── DEMOD_CTL bit masks ─────────────────────────────────────────────
    /// ADC_I enable (bit 0)
    pub const DEMOD_CTL_ADC_I_EN:  u8 = 0x01;
    /// ADC_Q enable (bit 1)
    pub const DEMOD_CTL_ADC_Q_EN:  u8 = 0x02;
    /// Demodulator PLL enable (bit 3)
    pub const DEMOD_CTL_DMOD_EN:   u8 = 0x08;
    /// I2C repeater enable — must be set before tuner I2C (bit 6)
    pub const DEMOD_CTL_I2C_GATE:  u8 = 0x40;
    /// DAGC (digital AGC) enable (bit 7)
    pub const DEMOD_CTL_DAGC_EN:   u8 = 0x80;

    // ── GPO / GPIO bit assignments ──────────────────────────────────────
    /// GPIO bit 0 — Bias-T control on RTL-SDR Blog hardware
    pub const GPIO_BIAS_T: u8 = 0x01;
}

// ============================================================
// Demodulator registers  (Block::demod(page), base 0x0300+)
//
// The RTL2832U demodulator has 4 pages.  Registers below are listed
// as (page, offset) pairs then mapped to the flat u16 namespace used
// by the read_reg/write_reg helpers via  Block::demod(page) + offset.
// ============================================================

pub mod demod {
    // ── Page 0 ──────────────────────────────────────────────────────────
    /// I2C repeater enable (page 0, reg 0x08)
    /// Bit 3: IIC_repeat — set 1 before any tuner I2C, clear after.
    pub const P0_IIC_REPEAT: u16 = 0x0008;

    /// ADC clock select (page 0, reg 0x0d)
    pub const P0_ADC_CLK: u16 = 0x000d;

    // ── Page 1 ──────────────────────────────────────────────────────────
    /// Soft reset demodulator (page 1, reg 0x01 bit 2)
    /// Note: librtlsdr comments say "bit 3" but the actual mask is 0x04.
    pub const P1_SOFT_RESET: u16 = 0x0101;
    /// Soft reset bit mask
    pub const P1_SOFT_RESET_BIT: u8 = 0x04;

    /// IF frequency register high byte (page 1, reg 0x19)
    pub const P1_IF_FREQ_H: u16 = 0x0119;
    /// IF frequency register mid byte  (page 1, reg 0x1a)
    pub const P1_IF_FREQ_M: u16 = 0x011a;
    /// IF frequency register low byte  (page 1, reg 0x1b)
    pub const P1_IF_FREQ_L: u16 = 0x011b;

    // ── Page 2 ──────────────────────────────────────────────────────────
    /// Resampler ratio high byte (page 2, reg 0x22)
    pub const P2_RESAMPLE_H: u16 = 0x0222;
    /// Resampler ratio mid byte  (page 2, reg 0x23)
    pub const P2_RESAMPLE_M: u16 = 0x0223;
    /// Resampler ratio low byte  (page 2, reg 0x24)
    pub const P2_RESAMPLE_L: u16 = 0x0224;
    /// Resampler ratio LSB byte  (page 2, reg 0x25)
    pub const P2_RESAMPLE_LSB: u16 = 0x0225;

    // ── Page 3 ──────────────────────────────────────────────────────────
    /// DAGC target level (page 3, reg 0x1a)
    pub const P3_DAGC_TARGET: u16 = 0x031a;
}

// ============================================================
// I2C master registers  (Block::I2c, base 0x0600)
// ============================================================

pub mod i2c {
    /// I2C clock divider register
    pub const I2CCR:  u16 = 0x6000;
    /// I2C master control register
    pub const I2CMCR: u16 = 0x6004;
    /// I2C master status register
    pub const I2CMSR: u16 = 0x600c;
    /// I2C master FIFO register
    pub const I2CMFR: u16 = 0x6010;
}

// ============================================================
// R828D tuner I2C address and shadow register layout
// ============================================================

pub mod r828d {
    /// 7-bit I2C address of the R828D tuner (write addr = 0x34, read = 0x35)
    pub const I2C_ADDR: u8 = 0x34;

    /// First writable register (shadow array starts here)
    pub const SHADOW_START: u8 = 0x05;

    /// Total number of shadow registers (0x05..=0x1f inclusive)
    pub const SHADOW_LEN: usize = 27;

    // ── Named register offsets (relative to I2C base, not shadow array) ─

    /// Status / PLL lock byte (read-only, not in shadow)
    pub const REG_00_STATUS: u8 = 0x00;

    /// LNA / input mux / power (V4 triplexer bits here)
    pub const REG_05_LNA: u8 = 0x05;

    /// Cable2 / open-drain control
    pub const REG_06_CABLE: u8 = 0x06;

    /// Tracking filter
    pub const REG_07_TF: u8 = 0x07;

    /// Mixer mode
    pub const REG_08_MIXER: u8 = 0x08;

    /// IF filter bandwidth
    pub const REG_09_IF: u8 = 0x09;

    /// VGA gain
    pub const REG_0A_VGA: u8 = 0x0a;

    /// LNA gain / init flag
    pub const REG_0C_LNA_GAIN: u8 = 0x0c;

    /// Version / chip ID register
    pub const REG_13_VERSION: u8 = 0x13;

    /// PLL divider / mix_div
    pub const REG_10_PLL_DIV: u8 = 0x10;

    /// PLL SDM dithering control
    pub const REG_12_SDM_CTRL: u8 = 0x12;

    /// PLL N-int low word (ni | si<<6)
    pub const REG_14_NINT: u8 = 0x14;

    /// PLL SDM fraction low byte
    pub const REG_15_SDM_L: u8 = 0x15;

    /// PLL SDM fraction high byte
    pub const REG_16_SDM_H: u8 = 0x16;

    /// RF mux open-drain
    pub const REG_17_RF_MUX: u8 = 0x17;

    /// RF mux + poly filter
    pub const REG_1A_RF_POLY: u8 = 0x1a;

    /// Tracking filter coefficient
    pub const REG_1B_TF_C: u8 = 0x1b;

    /// Bias-T control (bit 0)
    pub const REG_0F_BIAS_T: u8 = 0x0f;

    // ── Bit masks ────────────────────────────────────────────────────────

    /// REG_00: PLL lock indicator (bit 6, active high)
    pub const STATUS_PLL_LOCK: u8 = 0x40;

    /// REG_05: V4 Air input select (bit 5, 0 = ON)
    pub const LNA_AIR_ON: u8 = 0x00;
    pub const LNA_AIR_OFF: u8 = 0x20;

    /// REG_05: V4 Cable1 input select (bit 6)
    pub const LNA_CABLE1_ON:  u8 = 0x40;
    pub const LNA_CABLE1_OFF: u8 = 0x00;

    /// REG_06: V4 Cable2 input select (bit 3)
    pub const CABLE_CABLE2_ON:  u8 = 0x08;
    pub const CABLE_CABLE2_OFF: u8 = 0x00;

    /// REG_12: disable SDM dithering (integer-N mode)
    pub const SDM_CTRL_NO_DITHER: u8 = 0x08;

    /// REG_0F: Bias-T on
    pub const BIAS_T_ON:  u8 = 0x01;
    pub const BIAS_T_OFF: u8 = 0x00;
}

// ============================================================
// Tuner I2C probe identifiers
//
// Used by Device::probe_tuner() to auto-detect the attached tuner chip.
// Each entry is (i2c_addr, check_reg, expected_val).
// Sources: librtlsdr tuner detection table, osmocom/rtl-sdr.
// ============================================================

pub mod tuner_ids {
    // ── R820T / R820T2 / R828D ───────────────────────────────────────────
    /// I2C address shared by the entire R82XX family
    pub const R82XX_I2C_ADDR:  u8 = 0x34;
    /// Chip ID register
    pub const R82XX_CHECK_REG: u8 = 0x00;
    /// Expected chip ID value (bits [5:4] = 0b10 → masked to 0x60)
    pub const R82XX_CHECK_VAL: u8 = 0x69; // raw PLL status — any non-zero value is a hit

    // ── E4000 ─────────────────────────────────────────────────────────────
    pub const E4000_I2C_ADDR:  u8 = 0xc8;
    pub const E4000_CHECK_REG: u8 = 0x02;
    pub const E4000_CHECK_VAL: u8 = 0x40;

    // ── FC0012 / FC0013 ───────────────────────────────────────────────────
    pub const FC0012_I2C_ADDR:   u8 = 0xc6;
    pub const FC0012_CHECK_REG:  u8 = 0x00;
    pub const FC0012_CHECK_VAL:  u8 = 0xa1; // FC0012
    pub const FC0013_CHECK_VAL:  u8 = 0xa3; // FC0013
}

// ============================================================
// Sample rate helpers
// ============================================================

/// RTL2832U internal reference / XTAL frequency.
pub const XTAL_FREQ_HZ: u32 = 28_800_000;

/// Intermediate frequency the RTL2832U demodulator expects from the tuner.
pub const IF_FREQ_HZ: u64 = 3_570_000;

/// Compute the 3-byte IF frequency register value for a given IF and crystal frequency.
///
/// Formula (from reference driver):
/// ```text
/// if_freq = (IF_Hz * 2^22) / xtal_Hz
/// ```
/// Returns `[high, mid, low]` for registers `P1_IF_FREQ_H/M/L`.
pub fn if_freq_regs(if_hz: u32, xtal_hz: u32) -> [u8; 3] {
    let ratio = (if_hz as u64) * (1u64 << 22) / xtal_hz as u64;
    [
        ((ratio >> 16) & 0x3f) as u8, // 6 bits
        ((ratio >>  8) & 0xff) as u8,
        ( ratio        & 0xff) as u8,
    ]
}

/// Compute the 4-byte resampler ratio for a given sample rate and crystal frequency.
///
/// Formula (from reference driver):
/// ```text
/// resample = (xtal_Hz * 2^22) / sample_rate_Hz
/// ```
/// Returns `[h, m, l, lsb]` for registers `P2_RESAMPLE_H/M/L/LSB`.
pub fn resample_regs(sample_rate_hz: u32, xtal_hz: u32) -> [u8; 4] {
    let ratio = (xtal_hz as u64) * (1u64 << 22) / sample_rate_hz as u64;
    [
        ((ratio >> 24) & 0x3f) as u8,
        ((ratio >> 16) & 0xff) as u8,
        ((ratio >>  8) & 0xff) as u8,
        ( ratio        & 0xff) as u8,
    ]
}

/// Compute the effective sample rate from a resampler register value and crystal frequency.
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

    // ── Block helpers ────────────────────────────────────────────────────

    #[test]
    fn test_demod_page_addresses() {
        assert_eq!(Block::demod(0), 0x0300);
        assert_eq!(Block::demod(1), 0x0400);
        assert_eq!(Block::demod(2), 0x0500);
        assert_eq!(Block::demod(3), 0x0600);
    }

    // ── IF frequency registers ───────────────────────────────────────────

    #[test]
    fn test_if_freq_zero() {
        let regs = if_freq_regs(0, XTAL_FREQ_HZ);
        assert_eq!(regs, [0, 0, 0]);
    }

    #[test]
    fn test_if_freq_3570khz() {
        // Standard RTL-SDR IF: 3.57 MHz
        // Expected: (3_570_000 * 2^22) / 28_800_000 = 521_386 ≈ 0x07F5AA
        let regs = if_freq_regs(3_570_000, XTAL_FREQ_HZ);
        let flat = ((regs[0] as u32) << 16)
                 | ((regs[1] as u32) <<  8)
                 |  (regs[2] as u32);
        // Allow ±1 for integer rounding
        let expected: u32 = (3_570_000u64 * (1u64 << 22) / 28_800_000) as u32;
        assert!(
            flat.abs_diff(expected) <= 1,
            "IF freq reg mismatch: got 0x{:06x}, expected ~0x{:06x}",
            flat, expected
        );
    }

    // ── Resampler registers ──────────────────────────────────────────────

    #[test]
    fn test_resample_2048000() {
        // Standard 2.048 MSPS
        let regs = resample_regs(2_048_000, XTAL_FREQ_HZ);
        let effective = effective_sample_rate(regs, XTAL_FREQ_HZ);
        // Allow 1% tolerance for quantisation
        let err = (effective as i64 - 2_048_000i64).abs();
        assert!(
            err < 20_480,
            "Resampler 2.048M: effective={} error={}",
            effective, err
        );
    }

    #[test]
    fn test_resample_2400000() {
        let regs = resample_regs(2_400_000, XTAL_FREQ_HZ);
        let effective = effective_sample_rate(regs, XTAL_FREQ_HZ);
        let err = (effective as i64 - 2_400_000i64).abs();
        assert!(err < 24_000, "Resampler 2.4M error={}", err);
    }

    #[test]
    fn test_resample_roundtrip() {
        for &rate in &[250_000u32, 1_024_000, 2_048_000, 2_400_000, 3_200_000] {
            let regs      = resample_regs(rate, XTAL_FREQ_HZ);
            let effective = effective_sample_rate(regs, XTAL_FREQ_HZ);
            let err_ppm   = (effective as i64 - rate as i64).abs() * 1_000_000
                          / rate as i64;
            assert!(
                err_ppm < 10_000, // < 1%
                "Rate {} Hz: effective={} err={}ppm",
                rate, effective, err_ppm
            );
        }
    }

    // ── R828D constants sanity ───────────────────────────────────────────

    #[test]
    fn test_r828d_shadow_range() {
        // Shadow registers 0x05..=0x1f = 27 registers
        let end: u8 = r828d::SHADOW_START + r828d::SHADOW_LEN as u8 - 1;
        assert_eq!(end, 0x1f);
    }

    #[test]
    fn test_pll_lock_bit() {
        // Bit 6 of status byte
        assert_eq!(r828d::STATUS_PLL_LOCK, 0x40);
        let locked_byte: u8 = 0x40;
        assert_ne!(locked_byte & r828d::STATUS_PLL_LOCK, 0);
        let unlocked_byte: u8 = 0x00;
        assert_eq!(unlocked_byte & r828d::STATUS_PLL_LOCK, 0);
    }

    // ── I2C repeater bit ─────────────────────────────────────────────────

    #[test]
    fn test_i2c_repeater_register_address() {
        // Must be page 0, offset 0x08
        assert_eq!(demod::P0_IIC_REPEAT, 0x0008);
    }

    #[test]
    fn test_soft_reset_bit_mask() {
        // Reference driver uses 0x04 (bit 2), not 0x08 (bit 3) despite comment
        assert_eq!(demod::P1_SOFT_RESET_BIT, 0x04);
    }
}