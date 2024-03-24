//!
//! # Rust Implementation of the w25q128 Status Register Statuses
//! 
//! ## First 8 Status Register of the w25q128 module
//! 
//!                                S7    S6   S5    S4    S3    S2    S1    S0
//!                             -------------------------------------------------
//!                             | SRP | SEC | TB | BP2 | BP1 | BP0 | WEL | BUSY |
//!                             -------------------------------------------------
//!                                ^     ^     ^    ^     ^     ^     ^      ^ 
//!                                |     |     |    |     |     |     |      |
//!     Status Register Protect ----     |     |    |     |     |     |      |
//!     (non volatile)                   |     |    |     |     |     |      |
//!                                      |     |    |     |     |     |      |
//!     Sector Protect -------------------     |    |     |     |     |      |
//!     (non-volatile)                         |    |     |     |     |      |
//!                                            |    |     |     |     |      |
//!     Top/Bottom Protect ---------------------    |     |     |     |      |
//!     (non-volatile)                              |     |     |     |      |
//!                                                 |     |     |     |      |
//!     Block Protect Bits --------------------------------------     |      | 
//!     (non-volatile)                                                |      |
//!                                                                   |      |
//!     Write Enable Latch --------------------------------------------      |
//!                                                                          |
//!     Erase/Write In Progress ----------------------------------------------
//!
//! ## Second 8 Status Registers of the  w25q128 module
//! 
//!                                       S15   S14   S13   S12   S11   S10   S9    S8
//!                                     ------------------------------------------------
//!                                     | SUS | CMP | LB3 | LB2 | LB1 | (R) | QE | SRL |
//!                                     ------------------------------------------------
//!                                        ^     ^     ^     ^     ^     ^    ^     ^
//!                                        |     |     |     |     |     |    |     |
//!     Suspend Status ---------------------     |     |     |     |     |    |     |
//!     (Status-Only)                            |     |     |     |     |    |     |
//!                                              |     |     |     |     |    |     |
//!     Complement Protect -----------------------     |     |     |     |    |     |
//!     (Volatile/Non-Volatile Writable)               |     |     |     |    |     |
//!                                                    |     |     |     |    |     |
//!     Security Register Lock Bits --------------------------------     |    |     |
//!     (Volatile/Non-Volatile Writable)                                 |    |     |
//!                                                                      |    |     |
//!     Reserved ---------------------------------------------------------    |     |
//!                                                                           |     |
//!     Quad Enable -----------------------------------------------------------     |
//!     (Volatile/Non-Volatile Writable)                                            |
//!                                                                                 |
//!     Status Register Lock --------------------------------------------------------
//! 
//! ## Third (and final) 8 Status Registers of the w25q128 module
//! 
//!                                   S23    S22    S21   S20   S19   S18   S17   S16
//!                                 ---------------------------------------------------
//!                                 | (R) | DRV1 | DRV2 | (R) | (R) | WPS | (R) | (R) |
//!                                 ---------------------------------------------------
//!                                    ^      ^      ^     ^     ^     ^     ^     ^
//!                                    |      |      |     |     |     |     |     |
//!     Reserved -----------------------      |      |     |     |     |     |     |
//!                                           |      |     |     |     |     |     |
//!     Output Driver Strength -----------------------     |     |     |     |     |
//!     (Volatile/Non-Volatile Writable)                   |     |     |     |     |
//!                                                        |     |     |     |     |
//!     Reserved -------------------------------------------------     |     |     |
//!                                                                    |     |     |
//!     Write Protect Selection ----------------------------------------     |     |
//!     (Volatile/Non-Volatile Writable)                                     |     |
//!                                                                          |     |
//!     Reserved -------------------------------------------------------------------
//! 

use core::fmt::Debug;
use core::ops::BitOr;

pub trait ContainsStatus<Status: Debug + Copy + Clone> {
    /// Return true if the u8 status obtained from reading a status register contains the given status.
    fn contains_status(&self, status: Status) -> bool;
}

pub trait CheckForStatus<E: Debug, Status: Debug + Copy + Clone> {
    /// Check that the w25q128 module contains the given status.
    fn check_for_status(&mut self, status: Status) -> Result<bool, E>;
}

/// Enum to allow access to read the various status registers.
pub enum StatusRegister {
    StatusRegister1,
    StatusRegister2,
    StatusRegister3,
}

/// Enum to allow access to read and erase the various security registers.
pub enum SecurityRegister {
    SecurityRegister1,
    SecurityRegister2,
    SecurityRegister3,
}

#[derive(Debug, Copy, Clone)]
pub enum StatusReg1 {
    /// Read-only bit that is set to 1 when the device is busy erasing
    /// or reading from a provided instruction.  When busy, the device can
    /// still execute the ReadStatusRegister and Erase/Program Suspend Instructions.
    /// 
    /// Busy will be cleard to a 0 when the device has completed the current erase
    /// or write instruction
    Busy = 0b00000001,
    /// Read-only bit that is set to 1 after a Write Enable Instruction.  The device is
    /// write disabled when powered up or when executing any of the following instructions
    /// WriteDisable, PageProgram, QuadPageProgram, SecturErase, BlockErase, ChipErase,
    /// WriteStatusRegister, EraseSecurityRegister, and ProgramSecurityRegister (see instruction.rs)
    WriteEnableLatch = 0b00000010,
    /// Read/Write bit in S2 that protects a specific section of memory from being writable to.
    /// 
    /// The default setting is BP0 = 0
    /// 
    /// See Section 7.1.8 W25Q128JV Status Register Memory Protection at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    BlockProtectBits0 = 0b00000100,
    /// Read/Write bit in S3 that protects a specific section of memory from being writable to.
    /// 
    /// The default setting is BP1 = 0
    /// 
    /// See Section 7.1.8 W25Q128JV Status Register Memory Protection at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    BlockProtectBits1 = 0b00001000,
    /// Read/Write bit in S4 that protects a specific section of memory from being writable to.
    /// 
    /// The default setting is BP2 = 0
    /// 
    /// See Section 7.1.8 W25Q128JV Status Register Memory Protection at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    BlockProtectBits2 = 0b0010000,
    /// Determines whether the BlockProtectBits protect either the Top (TB=0) or
    /// Bottom (TB=1) of the array shown in section 7.1.8 of the Datasheet.  The
    /// Default setting is TB=0.
    /// 
    /// See Section 7.1.8 W25Q128JV Status Register Memory Protection at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    TopBottomBlockProtect = 0b00100000,
    /// Determines whether the BlockProtectBits protect either 4KB Sectors (SEC=1) or
    /// 64KB Blocks (SEC=0) in the Top or Bottom of the array shown in section 7.1.8 of
    /// the datasheet.  THe Default settings is SEC=0
    /// 
    /// See Section 7.1.8 W25Q128JV Status Register Memory Protection at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    SectorBlockProtectBits = 0b01000000,
    /// Together with StatusRegisterLock controls the ability to write to the status registers.
    /// 
    /// For more information see section 7.1.1 Status Register Protect at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    StatusRegisterProtect = 0b10000000,
}

impl StatusReg1 {
    /// Return all (writable) bits of the first status register set to true
    pub fn all() -> u8 {
        0xFE
    }
}

impl ContainsStatus<StatusReg1> for u8 {
    fn contains_status(&self, status: StatusReg1) -> bool {
        *self & (status as u8) > 0
    }
}

impl BitOr for StatusReg1 {
    type Output = u8;

    fn bitor(self, rhs: Self) -> Self::Output {
        (self as u8) | (rhs as u8)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum StatusReg2 {
    /// Together with StatusRegisterProtect controls the ability to write to the status registers.
    /// 
    /// For more information see section 7.1.1 Status Register Protect at <https://datasheet.lcsc.com/lcsc/1912111437_Winbond-Elec-W25Q128JVSIQ_C113767.pdf>
    StatusRegisterLock = 0b00000001,
    /// Enables QuadSPI operations when QE = 1.  Defaults as QE = 0, which uses the standard/dual spi mode.
    QuadEnable = 0b00000010,
    /// Non Functional Bit to be Ignored
    Reserved1 = 0b00000100,
    /// When set to 1 (One-Time Programmable), the 1st security register is locked (and cannot be modified)
    SecurityRegisterLockBit1 = 0b00001000,
    /// When set to 1 (One-Time Programmable), the 2nd security register is locked (and cannot be modified)
    SecurityRegisterLockBit2 = 0b00010000,
    /// When set to 1 (One-Time Programmable), the 3rd security register is locked (and cannot be modified)
    SecurityRegisterLockBit3 = 0b00100000,
    /// Complements (reverses) the protected bits protected by the BlockProtectBitsX Status.
    ComplementProtect = 0b01000000,
    /// Read Only bit set to 1 after executing EraseProgramSuspend, and will be cleared when executing the EraseProgramResume
    /// instruction.
    SuspendStatus = 0b10000000,
}

impl StatusReg2 {
    /// Return all (writable) bits of the second status register set to true
    pub fn all() -> u8 {
        0b01111011
    }
}

impl ContainsStatus<StatusReg2> for u8 {
    fn contains_status(&self, status: StatusReg2) -> bool {
        *self & (status as u8) > 0
    }
}

impl BitOr for StatusReg2 {
    type Output = u8;

    fn bitor(self, rhs: Self) -> Self::Output {
        (self as u8) | (rhs as u8)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum StatusReg3 {
    /// Non Functional Bit to be Ignored
    Reserved2 = 0b00000001,
    /// Non Functional Bit to be Ignored
    Reserved3 = 0b00000010,
    /// Identifies which write protection scheme to use.  When WP=0 will use CMP, SEC, TB, BP\[2:0\] bits to determine
    /// which areas of memory are protected.  When WP=1the individual block locks will be used to determine memory
    /// protection
    WriteProtectSelection = 0b00000100,
    /// Non Functional Bit to be Ignored
    Reserved4 = 0b00001000,
    /// Non Functional Bit to be Ignored
    Reserved5 = 0b00010000,
    /// Determines the output strength of read operations (in combination with OutputDriverStrength2)
    /// 
    /// i.e.    | DRV1 | DRV0 | Driver Strength    |
    ///         |  0   |  0   |      100%          |
    ///         |  0   |  1   |       75%          |
    ///         |  1   |  0   |       50%          |
    ///         |  1   |  1   |       25% (default)|
    OutputDriverStrength1 = 0b00100000,
    /// Determines the output strength of read operations (in combination with OutputDriverStrength2)
    /// 
    /// i.e.    | DRV1 | DRV0 | Driver Strength    |
    ///         |  0   |  0   |      100%          |
    ///         |  0   |  1   |       75%          |
    ///         |  1   |  0   |       50%          |
    ///         |  1   |  1   |       25% (default)|
    OutputDriverStrength2 = 0b01000000,
    /// Non Functional Bit to be Ignored
    Reserved6 = 0b10000000,
}

impl StatusReg3 {
    /// Return all (writable) bits of the third status register set to true
    pub fn all() -> u8 {
        0b01100100
    }
}

impl ContainsStatus<StatusReg3> for u8 {
    fn contains_status(&self, status: StatusReg3) -> bool {
        *self & (status as u8) > 0
    }
}

impl BitOr for StatusReg3 {
    type Output = u8;

    fn bitor(self, rhs: Self) -> Self::Output {
        (self as u8) | (rhs as u8)
    }
}