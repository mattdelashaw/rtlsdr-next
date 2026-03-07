//! RTL2832U and Tuner Register Definitions

/// RTL2832U Block Addresses
pub enum Block {
    Usb = 0x000,
    Sys = 0x100,
    Reg1 = 0x200,
    Reg2 = 0x300,
    I2c = 0x600,
}

/// RTL2832U USB Registers
pub mod usb {
    pub const SYSCTL: u16 = 0x2000;
    pub const EPA_CTL: u16 = 0x2028;
    pub const EPA_MAX_PKT: u16 = 0x202a;
}

/// RTL2832U System Registers
pub mod sys {
    pub const DEMOD_CTL: u16 = 0x3000;
    pub const GPO: u16 = 0x3001;
    pub const GPI: u16 = 0x3002;
    pub const GPD: u16 = 0x3003;
    pub const SYS_CONFIG: u16 = 0x3004;
}

/// R828D Tuner Registers (Offsets from I2C base)
pub mod r828d {
    pub const I2C_ADDR: u8 = 0x34; // Default I2C address for R828D
    
    // Register indices for R828D
    pub const REG_00: u8 = 0x00;
    pub const REG_05: u8 = 0x05; // Power/Mode
    pub const REG_06: u8 = 0x06; // Filter
    pub const REG_07: u8 = 0x07; // LNA
    pub const REG_08: u8 = 0x08; // Mixer
    pub const REG_09: u8 = 0x09; // IF
    pub const REG_0A: u8 = 0x0a; // VGA
    pub const REG_10: u8 = 0x10; // PLL
    pub const REG_17: u8 = 0x17; // Frequency (LSB)
}

/// USB Vendor Requests
pub enum Request {
    RegRead = 0x00,
    RegWrite = 0x01,
    I2cRead = 0x02,
    I2cWrite = 0x03,
}
