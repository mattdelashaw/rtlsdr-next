//! RTL2832U demodulator (baseband) initialization and configuration.
//!
//! # Responsibility split
//!
//! | Module      | Responsibility                                      |
//! |-------------|-----------------------------------------------------|
//! | `tuners/`   | R828D PLL, RF mux, gain — tuner chip only           |
//! | `demod`     | RTL2832U ADC power, IF freq, sample rate, DAGC      |
//! | `device`    | Raw USB register read/write, I2C repeater gating    |
//!
//! # Initialization sequence
//!
//! On `Driver::new()` the full sequence is:
//! ```text
//! 1. demod::power_on()          — enable ADCs, demod PLL, DAGC
//! 2. demod::init_registers()    — write static init table
//! 3. tuner.initialize()         — R828D shadow register upload
//! 4. demod::set_if_freq()       — program IF frequency registers
//! 5. demod::set_sample_rate()   — program resampler ratio
//! 6. demod::reset_demod()       — soft-reset demodulator
//! 7. demod::start_streaming()   — flush EPA FIFO, enable bulk IN
//! ```
//!
//! Steps 4–7 are also called on every `set_frequency()` / `set_sample_rate()`
//! call to re-sync the demodulator after the PLL relock.

use crate::device::HardwareInterface;
use crate::error::Result;
use crate::registers::{self, Block, demod, sys, usb};
use log::info;

// ── Default sample rate ──────────────────────────────────────────────────────

/// Default output sample rate: 2.048 MSPS.
///
/// Chosen because it is exactly 2^11 * 1000 Hz — an integer multiple of common
/// audio rates and evenly divisible by most decimation factors used in practice.
pub const DEFAULT_SAMPLE_RATE: u32 = 2_048_000;

// ── Static init table ────────────────────────────────────────────────────────
//
// Sourced from librtlsdr rtlsdr_open() init_tab[] in librtlsdr.c.
// Format: (page, reg_offset, value)
// These registers configure the demodulator core for direct-sampling SDR use
// rather than the DVB-T application the chip was originally designed for.

struct InitEntry {
    page:  u8,
    reg:   u16,
    val:   u8,
}

/// RTL2832U demodulator static initialization table.
///
/// These writes configure the chip for direct-sampling SDR mode.
/// Sources: librtlsdr init_tab[], cross-referenced with rtl2832.c kernel driver.
static INIT_TABLE: &[InitEntry] = &[
    // ── Page 1: IQ path, spectrum inversion, ADC config ──────────────────
    InitEntry { page: 1, reg: 0x00, val: 0x00 }, // no spectrum inversion
    InitEntry { page: 1, reg: 0x01, val: 0x00 }, // IIC_repeat off (will be set per-transaction)
    InitEntry { page: 1, reg: 0x15, val: 0x00 }, // carrier recovery loop bandwidth = 0
    InitEntry { page: 1, reg: 0x16, val: 0x00 },
    InitEntry { page: 1, reg: 0x17, val: 0x00 },
    InitEntry { page: 1, reg: 0x1a, val: 0x00 }, // carrier freq offset = 0
    InitEntry { page: 1, reg: 0x1b, val: 0x00 },

    // IQ estimation / compensation
    InitEntry { page: 1, reg: 0xb1, val: 0x1b },

    // ── Page 0: ADC and DAGC setup ────────────────────────────────────────
    InitEntry { page: 0, reg: 0x19, val: 0x05 }, // magic: enable ADC I/Q
    InitEntry { page: 0, reg: 0x1f, val: 0x00 }, // disable test mode

    // ── Page 2: resampler defaults (overwritten by set_sample_rate) ───────
    InitEntry { page: 2, reg: 0x22, val: 0x00 },
    InitEntry { page: 2, reg: 0x23, val: 0x00 },
    InitEntry { page: 2, reg: 0x24, val: 0x00 },
    InitEntry { page: 2, reg: 0x25, val: 0x00 },

    // ── Page 3: DAGC target level ─────────────────────────────────────────
    // 0x05 is the default target used by librtlsdr (approx -25 dBFS)
    InitEntry { page: 3, reg: 0x1a, val: 0x05 },
];

// ── Demodulator operations ───────────────────────────────────────────────────

/// Power on the RTL2832U ADCs and demodulator PLL.
///
/// Writes to `DEMOD_CTL` (SYS block) to enable:
/// - ADC_I and ADC_Q (bits 0 and 1)
/// - Demodulator PLL (bit 3)
/// - DAGC (bit 7)
///
/// Also clears the power-down bit (bit 5) in `DEMOD_CTL1`.
pub fn power_on(hw: &dyn HardwareInterface) -> Result<()> {
    // Enable ADC_I, ADC_Q, demod PLL, DAGC; leave I2C gate (bit 6) alone
    let ctl = sys::DEMOD_CTL_ADC_I_EN
            | sys::DEMOD_CTL_ADC_Q_EN
            | sys::DEMOD_CTL_DMOD_EN
            | sys::DEMOD_CTL_DAGC_EN;
    hw.write_reg(Block::Sys as u16, sys::DEMOD_CTL, ctl)?;

    // Clear power-down bit in DEMOD_CTL1 (bit 5 = 0x20 means powered down)
    let ctl1 = hw.read_reg(Block::Sys as u16, sys::DEMOD_CTL1)?;
    hw.write_reg(Block::Sys as u16, sys::DEMOD_CTL1, ctl1 & !0x20)?;

    info!("ADCs and PLL powered on (DEMOD_CTL=0x{:02x})", ctl);
    Ok(())
}

/// Write the static demodulator initialization table.
///
/// Must be called after `power_on()` and before `set_if_freq` / `set_sample_rate`.
pub fn init_registers(hw: &dyn HardwareInterface) -> Result<()> {
    for entry in INIT_TABLE {
        let block = Block::demod(entry.page);
        hw.write_reg(block, entry.reg, entry.val)?;
    }
    info!("Static init table written ({} entries)", INIT_TABLE.len());
    Ok(())
}

/// Program the IF frequency registers (page 1, regs 0x19/0x1a/0x1b).
///
/// The RTL2832U expects the tuner to deliver the signal at a fixed IF.
/// For the R828D this is 3.57 MHz. The demodulator needs to know this
/// to correctly position its internal NCO.
///
/// The formula (from reference driver):
/// ```text
/// if_reg = -(if_hz * 2^22 / xtal_hz)   [two's complement, 22-bit]
/// ```
/// The negation accounts for the R828D's low-side injection — the tuner
/// places the signal at -IF relative to the LO.
pub fn set_if_freq(hw: &dyn HardwareInterface, if_hz: u32, xtal_hz: u32) -> Result<()> {
    // Two's complement negated IF ratio for low-side injection
    let ratio_pos = (if_hz as u64) * (1u64 << 22) / xtal_hz as u64;
    // Negate and mask to 22 bits (sign-extend in 22-bit two's complement)
    let ratio = ((!ratio_pos).wrapping_add(1)) & 0x3F_FFFF;

    let h = ((ratio >> 16) & 0x3f) as u8;
    let m = ((ratio >>  8) & 0xff) as u8;
    let l = ( ratio        & 0xff) as u8;

    hw.write_reg(Block::demod(1), demod::P1_IF_FREQ_H, h)?;
    hw.write_reg(Block::demod(1), demod::P1_IF_FREQ_M, m)?;
    hw.write_reg(Block::demod(1), demod::P1_IF_FREQ_L, l)?;

    info!(
        "IF freq {} Hz (xtal {} Hz) → ratio=0x{:06x} regs=[0x{:02x}, 0x{:02x}, 0x{:02x}]",
        if_hz, xtal_hz, ratio, h, m, l
    );
    Ok(())
}

/// Program the resampler ratio registers (page 2, regs 0x22–0x25).
///
/// Controls the RTL2832U output sample rate. The formula:
/// ```text
/// resample_ratio = xtal_hz * 2^22 / sample_rate_hz
/// ```
///
/// Valid range is approximately 225 kSPS to 3.2 MSPS. Rates above
/// 2.56 MSPS will lose samples on most USB 2.0 hosts under load.
pub fn set_sample_rate(hw: &dyn HardwareInterface, rate_hz: u32, xtal_hz: u32) -> Result<()> {
    if !(225_000..=3_200_000).contains(&rate_hz) {
        return Err(crate::Error::InvalidSampleRate(rate_hz));
    }
    let regs = registers::resample_regs(rate_hz, xtal_hz);

    hw.write_reg(Block::demod(2), demod::P2_RESAMPLE_H,   regs[0])?;
    hw.write_reg(Block::demod(2), demod::P2_RESAMPLE_M,   regs[1])?;
    hw.write_reg(Block::demod(2), demod::P2_RESAMPLE_L,   regs[2])?;
    hw.write_reg(Block::demod(2), demod::P2_RESAMPLE_LSB, regs[3])?;

    let effective = registers::effective_sample_rate(regs, xtal_hz);
    info!(
        "Sample rate {} Hz (xtal {} Hz) → effective {} Hz (regs={:02x?})",
        rate_hz, xtal_hz, effective, regs
    );
    Ok(())
}

/// Soft-reset the demodulator.
///
/// Sets then clears the soft-reset bit (page 1, reg 0x01 bit 2).
/// Required after reprogramming IF or sample rate to flush internal
/// state machines and prevent sample counter drift.
pub fn reset_demod(hw: &dyn HardwareInterface) -> Result<()> {
    // Set soft reset
    hw.write_reg(
        Block::demod(1),
        demod::P1_SOFT_RESET,
        demod::P1_SOFT_RESET_BIT,
    )?;
    // Clear soft reset
    hw.write_reg(Block::demod(1), demod::P1_SOFT_RESET, 0x00)?;
    info!("Soft reset complete");
    Ok(())
}

/// Flush the endpoint A FIFO and re-enable the bulk IN pipe.
///
/// Must be called before starting USB bulk reads. The FIFO may contain
/// stale data from a previous session or from the demodulator reset.
///
/// Sequence:
/// 1. Stall EPA (stops DMA, allows FIFO flush)
/// 2. Flush EPA FIFO
/// 3. Unstall EPA (re-enables bulk IN)
pub fn start_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    // Stall bulk IN endpoint (bit 1 of EPA_CTL)
    hw.write_reg(Block::Usb as u16, usb::EPA_CTL, 0x02)?;
    // Flush FIFO (bit 1 of SYSCTL)
    hw.write_reg(Block::Usb as u16, usb::SYSCTL, 0x09)?;
    hw.write_reg(Block::Usb as u16, usb::SYSCTL, 0x08)?;
    // Unstall — clear bit 1, keep bit 0 set
    hw.write_reg(Block::Usb as u16, usb::EPA_CTL, 0x00)?;

    info!("EPA FIFO flushed, bulk IN ready");
    Ok(())
}

/// Stop streaming: stall the bulk IN endpoint and power down ADCs.
pub fn stop_streaming(hw: &dyn HardwareInterface) -> Result<()> {
    // Stall EPA
    hw.write_reg(Block::Usb as u16, usb::EPA_CTL, 0x02)?;
    // Power down ADCs and demod PLL (leave DAGC off too)
    hw.write_reg(Block::Sys as u16, sys::DEMOD_CTL, 0x20)?;
    info!("Streaming stopped, ADCs powered down");
    Ok(())
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::{IF_FREQ_HZ, XTAL_FREQ_HZ};
    use std::sync::Mutex;
    use std::time::Duration;

    // ── Mock hardware ────────────────────────────────────────────────────

    struct MockHw {
        writes: Mutex<Vec<(u16, u16, u8)>>, // (block_as_u16, reg, val)
    }

    impl MockHw {
        fn new() -> Self {
            Self { writes: Mutex::new(vec![]) }
        }

        fn wrote(&self, block: u16, reg: u16, val: u8) -> bool {
            self.writes.lock().unwrap().iter()
                .any(|&(b, r, v)| b == block && r == reg && v == val)
        }
    }

    impl HardwareInterface for MockHw {
        fn read_reg(&self, _block: u16, _addr: u16) -> Result<u8> { Ok(0x00) }
        fn write_reg(&self, block: u16, addr: u16, val: u8) -> Result<()> {
            self.writes.lock().unwrap().push((block, addr, val));
            Ok(())
        }
        fn i2c_read(&self,  _: u8, _: u8, len: usize) -> Result<Vec<u8>> {
            Ok(vec![0u8; len])
        }
        fn i2c_write(&self, _: u8, _: u8, _: &[u8]) -> Result<()> { Ok(()) }
        fn read_bulk(&self, _: u8, _: &mut [u8], _: Duration) -> Result<usize> { Ok(0) }
    }

    // ── power_on ─────────────────────────────────────────────────────────

    #[test]
    fn test_power_on_sets_demod_ctl() {
        let hw = MockHw::new();
        power_on(&hw).unwrap();

        // Must write ADC_I | ADC_Q | DMOD_EN | DAGC_EN = 0x8b
        let expected = sys::DEMOD_CTL_ADC_I_EN
                     | sys::DEMOD_CTL_ADC_Q_EN
                     | sys::DEMOD_CTL_DMOD_EN
                     | sys::DEMOD_CTL_DAGC_EN;
        assert!(
            hw.wrote(Block::Sys as u16, sys::DEMOD_CTL, expected),
            "DEMOD_CTL not set correctly (expected 0x{:02x})", expected
        );
    }

    #[test]
    fn test_power_on_clears_power_down_bit() {
        let hw = MockHw::new();
        power_on(&hw).unwrap();

        // DEMOD_CTL1 write should NOT have bit 5 (0x20) set
        let writes = hw.writes.lock().unwrap();
        let ctl1_writes: Vec<_> = writes.iter()
            .filter(|&&(b, r, _)| b == Block::Sys as u16 && r == sys::DEMOD_CTL1)
            .collect();

        assert!(!ctl1_writes.is_empty(), "No DEMOD_CTL1 write");
        for &(_, _, v) in &ctl1_writes {
            assert_eq!(v & 0x20, 0, "Power-down bit still set in DEMOD_CTL1");
        }
    }

    // ── init_registers ───────────────────────────────────────────────────

    #[test]
    fn test_init_registers_writes_all_entries() {
        let hw = MockHw::new();
        init_registers(&hw).unwrap();

        let count = hw.writes.lock().unwrap().len();
        assert_eq!(count, INIT_TABLE.len(), "Wrong number of init writes");
    }

    #[test]
    fn test_init_registers_dagc_target() {
        let hw = MockHw::new();
        init_registers(&hw).unwrap();
        // Page 3, reg 0x1a should be written with 0x05
        assert!(
            hw.wrote(Block::demod(3), 0x1a, 0x05),
            "DAGC target register not written"
        );
    }

    // ── set_if_freq ──────────────────────────────────────────────────────

    #[test]
    fn test_set_if_freq_writes_three_regs() {
        let hw = MockHw::new();
        set_if_freq(&hw, IF_FREQ_HZ as u32, XTAL_FREQ_HZ).unwrap();

        let writes = hw.writes.lock().unwrap();
        let page1_writes: Vec<_> = writes.iter()
            .filter(|&(b, _, _)| *b == Block::demod(1))
            .collect();

        // Must have written H, M, L
        let has_h = page1_writes.iter().any(|&&(_, r, _)| r == demod::P1_IF_FREQ_H);
        let has_m = page1_writes.iter().any(|&&(_, r, _)| r == demod::P1_IF_FREQ_M);
        let has_l = page1_writes.iter().any(|&&(_, r, _)| r == demod::P1_IF_FREQ_L);
        assert!(has_h, "IF_FREQ_H not written");
        assert!(has_m, "IF_FREQ_M not written");
        assert!(has_l, "IF_FREQ_L not written");
    }

    #[test]
    fn test_set_if_freq_nonzero_for_standard_if() {
        let hw = MockHw::new();
        set_if_freq(&hw, 3_570_000, XTAL_FREQ_HZ).unwrap();

        let writes = hw.writes.lock().unwrap();
        // At least one of the three bytes must be non-zero for a real IF
        let any_nonzero = writes.iter()
            .filter(|&(b, r, _)| {
                *b == Block::demod(1) && (
                    *r == demod::P1_IF_FREQ_H ||
                    *r == demod::P1_IF_FREQ_M ||
                    *r == demod::P1_IF_FREQ_L
                )
            })
            .any(|&(_, _, v)| v != 0);

        assert!(any_nonzero, "All IF freq registers are zero for 3.57 MHz IF");
    }

    #[test]
    fn test_set_if_freq_zero_gives_zero_regs() {
        // IF=0 should write all zeros (DC — rarely used but shouldn't panic)
        let hw = MockHw::new();
        set_if_freq(&hw, 0, XTAL_FREQ_HZ).unwrap();

        let writes = hw.writes.lock().unwrap();
        for &(b, r, v) in writes.iter() {
            if b == Block::demod(1) && (
                r == demod::P1_IF_FREQ_H ||
                r == demod::P1_IF_FREQ_M ||
                r == demod::P1_IF_FREQ_L
            ) {
                assert_eq!(v, 0, "IF=0 should produce zero register values");
            }
        }
    }

    // ── set_sample_rate ──────────────────────────────────────────────────

    #[test]
    fn test_set_sample_rate_writes_four_regs() {
        let hw = MockHw::new();
        set_sample_rate(&hw, DEFAULT_SAMPLE_RATE, XTAL_FREQ_HZ).unwrap();

        let writes = hw.writes.lock().unwrap();
        let page2_writes: Vec<_> = writes.iter()
            .filter(|&(b, _, _)| *b == Block::demod(2))
            .collect();

        assert_eq!(page2_writes.len(), 4, "Expected 4 resampler register writes");
    }

    #[test]
    fn test_set_sample_rate_nonzero_for_standard_rate() {
        let hw = MockHw::new();
        set_sample_rate(&hw, 2_048_000, XTAL_FREQ_HZ).unwrap();

        let writes = hw.writes.lock().unwrap();
        let any_nonzero = writes.iter()
            .filter(|&(b, _, _)| *b == Block::demod(2))
            .any(|&(_, _, v)| v != 0);
        assert!(any_nonzero, "All resampler registers zero for 2.048 MSPS");
    }

    // ── reset_demod ──────────────────────────────────────────────────────

    #[test]
    fn test_reset_demod_set_then_clear() {
        let hw = MockHw::new();
        reset_demod(&hw).unwrap();

        let writes = hw.writes.lock().unwrap();
        let reset_writes: Vec<_> = writes.iter()
            .filter(|&(b, r, _)| *b == Block::demod(1) && *r == demod::P1_SOFT_RESET)
            .map(|&(_, _, v)| v)
            .collect();

        assert_eq!(reset_writes.len(), 2, "Expected set+clear of reset bit");
        assert_eq!(reset_writes[0], demod::P1_SOFT_RESET_BIT, "First write should set reset bit");
        assert_eq!(reset_writes[1], 0x00, "Second write should clear reset bit");
    }

    // ── start_streaming ──────────────────────────────────────────────────

    #[test]
    fn test_start_streaming_unstalls_epa() {
        let hw = MockHw::new();
        start_streaming(&hw).unwrap();

        // Final EPA_CTL write must be 0x00 (unstalled)
        let writes = hw.writes.lock().unwrap();
        let epa_writes: Vec<_> = writes.iter()
            .filter(|&(b, r, _)| *b == Block::Usb as u16 && *r == usb::EPA_CTL)
            .map(|&(_, _, v)| v)
            .collect();

        assert!(!epa_writes.is_empty(), "No EPA_CTL writes");
        assert_eq!(*epa_writes.last().unwrap(), 0x00, "EPA not unstalled at end");
    }

    // ── stop_streaming ───────────────────────────────────────────────────

    #[test]
    fn test_stop_streaming_powers_down_adcs() {
        let hw = MockHw::new();
        stop_streaming(&hw).unwrap();

        assert!(
            hw.wrote(Block::Sys as u16, sys::DEMOD_CTL, 0x20),
            "ADC power-down write missing"
        );
    }

    // ── IF freq math cross-check ─────────────────────────────────────────

    #[test]
    fn test_if_freq_negation_roundtrip() {
        // Verify that the negated ratio we write can be decoded back to
        // approximately the original IF frequency.
        let if_hz = IF_FREQ_HZ as u32;
        let xtal  = XTAL_FREQ_HZ as u64;

        let ratio_pos = (if_hz as u64) * (1u64 << 22) / xtal;
        let ratio_neg = ((!ratio_pos).wrapping_add(1)) & 0x3F_FFFF;

        // Decode: negate the 22-bit two's complement back
        let decoded_pos = ((!ratio_neg).wrapping_add(1)) & 0x3F_FFFF;
        let decoded_if  = decoded_pos * xtal / (1u64 << 22);

        let err_hz = (decoded_if as i64 - if_hz as i64).unsigned_abs();
        assert!(
            err_hz < 1000,
            "IF roundtrip error too large: {} Hz (decoded {} Hz, expected {} Hz)",
            err_hz, decoded_if, if_hz
        );
    }
}
