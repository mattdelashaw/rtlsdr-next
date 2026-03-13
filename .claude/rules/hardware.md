# Hardware Rules

## Control Transfer Encoding
- Regular reg:  `wValue=addr, wIndex=(block<<8)|0x10, data=[val]`
- Demod reg:    `wValue=(addr<<8)|0x20, wIndex=0x10|page, data=[val]` + dummy read after
- I2C:          `wValue=i2c_addr, wIndex=(6<<8)|0x10, data=[reg, bytes...]`
- Block IDs:    DEMOD=0, USB=1, SYS=2, I2C=6

## USB Behavior
- Every `demod_write_reg` and `demod_write_reg16` must be followed by `demod_read_reg(0x0a, 0x01)` — hardware sync requirement, not optional
- I2C writes chunk at 7 bytes max; increment register address per chunk
- EPA endpoint unstall sequence: write `EPA_CTL=0x1002` then `EPA_CTL=0x0000`

## V4 Board Init Sequence
1. `demod::power_on()` → 100ms sleep
2. Read EEPROM strings → determine `BoardConfig`
3. If V4: GPIO 4+5 output high → 250ms sleep  ← board level, NOT in tuner
4. `probe_tuner()` → I2C presence check at 0x34 then 0x74
5. `tuner.initialize()`
6. `demod::set_tuner_low_if()` + page1 reg 0x15 = 0x01
7. Set IF freq, sample rate, reset demod, start streaming

## Adding a New Tuner
1. Add I2C address to `registers.rs` tuner_ids
2. Add probe in `device.rs` `probe_tuner()` returning new `TunerType` variant
3. Add variant to `TunerType` enum in `tuner.rs`
4. Create `src/tuners/your_tuner.rs` implementing `Tuner` trait
5. Add match arm in `lib.rs` Driver::new()
- Tuner chip driver must never contain GPIO, triplexer, or board-specific logic
- All board-specific orchestration belongs in `Driver::set_frequency()` via `BoardConfig`

## EEPROM Recovery (V4)
- `~/rtl-sdr-blog/build/src/rtl_eeprom -m "RTLSDRBlog" -p "Blog V4" -s "00000001"`
