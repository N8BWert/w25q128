//! 
//! List of Instructions / Op Codes for the w25q128
//! 
#![allow(dead_code)]

pub(crate) enum Instructions {
    WriteEnable = 0x06,
    VolatileSrWriteEnable = 0x50,
    WriteDisable = 0x04,
    ReleasePowerDown = 0xAB,
    DeviceID = 0x90,
    JedecID = 0x9F,
    ReadUniqueId = 0x4B,
    ReadData = 0x03,
    FastRead = 0x0B,
    PageProgram = 0x02,
    SectorErase = 0x20,
    BlockErase32 = 0x52,
    BlockErase64 = 0xD8,
    ChipErase = 0xC7,
    ReadStatusReg1 = 0x05,
    WriteStatusReg1 = 0x01,
    ReadStatusReg2 = 0x35,
    WriteStatusReg2 = 0x31,
    ReadStatusReg3 = 0x15,
    WriteStatusReg3 = 0x11,
    ReadSFDPRegister = 0x5A,
    EraseSecurityRegister = 0x44,
    ProgramSecurityRegister = 0x42,
    ReadSecurityRegister = 0x48,
    GlobalBlockLock = 0x7E,
    GlobalBlockUnlock = 0x98,
    ReadBlockLock = 0x3D,
    IndividualBlockLock = 0x36,
    IndividualBlockUnlock = 0x39,
    EraseProgramSuspend = 0x75,
    EraseProgramResume = 0x7A,
    PowerDown = 0xB9,
    EnableReset = 0x66,
    ResetDevice = 0x99,
}

impl Instructions {
    pub fn opcode(self) -> u8 {
        self as u8
    }
}