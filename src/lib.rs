//!
//! Device Implementation for the W25Q128 Module
//! 
#![no_std]

extern crate alloc;

use alloc::vec::Vec;

use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::DelayUs;

pub mod instruction;
use crate::instruction::Instructions;

pub mod status;
use crate::status::{StatusRegister, ContainsStatus, StatusReg1, StatusReg2, StatusReg3, SecurityRegister};

pub mod error;
use crate::error::W25Q128Error;

/// W25Q128 Driver.
/// 
/// This driver takes control over a chip select pin, an spi interface, and has the ability to
/// give up access to a delay for work with multiple spi devices.  The driver also takes advantage
/// of the different W25Q128 clock frequencies possible for the device so that commands can be
/// issued at maximum possible frequencies and speeds.
/// 
/// TODO: Reconfigure SPI rate on read to maximize usage
/// 
/// max read frequency = 550 MHz
/// 
/// max nominal frequency = 133 MHz
/// 
pub struct StorageModule<CSN, SPI, SPIE, GPIOE> where
    CSN: OutputPin<Error=GPIOE>,
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE> {
    csn: CSN,
    volatile: bool,
    wait_delay: u32,
    _phantom: PhantomData<SPI>,
}

impl<CSN, SPI, SPIE, GPIOE> StorageModule<CSN, SPI, SPIE, GPIOE> where
    CSN: OutputPin<Error=GPIOE>,
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE> {
    pub fn new(
        csn: CSN,
    ) -> Self {
        Self {
            csn,
            volatile: false,
            wait_delay: 10,
            _phantom: PhantomData
        }
    }

    pub fn new_with_wait_delay(
        csn: CSN,
        wait_delay: u32,
    ) -> Self {
        Self {
            csn,
            volatile: false,
            wait_delay,
            _phantom: PhantomData
        }
    }

    /// Sets the WriteEnable status bit for the w25q128 allowing the memory to be written to.
    /// 
    /// It is very unlikely you should ever run this function on its own as I'm making concerted
    /// effort to run this in the function that actually writes to memory.  However, I may miss
    /// something so I'm allowing this to be called (but please don't).
    pub fn write_enable(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        self.write_spi(&[Instructions::WriteEnable.opcode()], spi, delay)?;

        let register_1 = self.read_from_status_register(StatusRegister::StatusRegister1, spi, delay)?;

        if !register_1.contains_status(StatusReg1::WriteEnableLatch) {
            return Err(W25Q128Error::UnableToSetWriteEnable);
        }

        Ok(())
    }

    /// Used to turn the non-volatile status registers into volatile bits so they can be changed quickly
    /// without waiting for the typical non-volatile bit write cycles.
    pub fn volatile_sr_write_enable(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;
        self.write_spi(&[Instructions::VolatileSrWriteEnable.opcode()], spi, delay)?;
        self.volatile = true;

        Ok(())
    }

    /// Resets the Write Enable Latch (WEL) bit in the Status Register to 0.  The WEL bit is also
    /// automatically reset after writing to the status registers, modifying the security registers,
    /// writing to the w25q128, erasing from the w25q128, and resetting the w25q128.
    /// 
    /// It is very unlikely you should ever run this function on its own as write access will automatically
    /// toggle write disable.  However, there may be some edge case where this is necessary so I'm leaving
    /// it public.
    pub fn write_disable(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;
        
        self.write_spi(&[Instructions::WriteDisable.opcode()], spi, delay)?;

        let register_1 = self.read_from_status_register(StatusRegister::StatusRegister1, spi, delay)?;
        if register_1.contains_status(StatusReg1::WriteEnableLatch) {
            return Err(W25Q128Error::UnableToResetWriteEnable);
        }

        Ok(())
    }

    /// Read the current value from a given status register.
    pub fn read_from_status_register(
        &mut self, status_register: StatusRegister, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<u8, W25Q128Error<SPIE, GPIOE>> {
        let instruction = match status_register {
            StatusRegister::StatusRegister1 => Instructions::ReadStatusReg1.opcode(),
            StatusRegister::StatusRegister2 => Instructions::ReadStatusReg2.opcode(),
            StatusRegister::StatusRegister3 => Instructions::ReadStatusReg3.opcode(),
        };
        let mut register_buffer = [instruction, 0x00];

        self.transfer_spi(&mut register_buffer, spi, delay)?;

        Ok(register_buffer[1])
    }

    pub fn write_to_status_register(
        &mut self, status_register: StatusRegister, status: u8, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        let base_instruction = match status_register {
            StatusRegister::StatusRegister1 => Instructions::WriteStatusReg1.opcode(),
            StatusRegister::StatusRegister2 => Instructions::WriteStatusReg2.opcode(),
            StatusRegister::StatusRegister3 => Instructions::WriteStatusReg3.opcode(),
        };

        self.write_spi(&[base_instruction, status], spi, delay)?;

        Ok(())
    }

    // Read a given number of bytes starting at an address
    pub fn read(
        &mut self, start_address: [u8; 3], buffer: &mut [u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        let mut instruction = Vec::with_capacity(4 + buffer.len());
        instruction[0] = Instructions::ReadData.opcode();
        instruction[1] = start_address[0];
        instruction[2] = start_address[1];
        instruction[3] = start_address[2];

        self.transfer_spi(instruction.as_mut_slice(), spi, delay)?;

        buffer[..].copy_from_slice(&instruction[4..]);
        Ok(())
    }

    pub fn fast_read(
        &mut self, start_address: [u8; 3], buffer: &mut [u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        let mut instruction = Vec::with_capacity(12 + buffer.len());
        instruction[0] = Instructions::FastRead.opcode();
        instruction[1] = start_address[0];
        instruction[2] = start_address[1];
        instruction[3] = start_address[2];

        self.transfer_spi(instruction.as_mut_slice(), spi, delay)?;
        buffer[..].copy_from_slice(&instruction[12..]);
        Ok(())
    }

    // Writes a maximum of 256 bytes to the w25q128
    pub fn write(
        &mut self, start_address: [u8; 3], data: &[u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        if !self.check_status_one(StatusReg1::WriteEnableLatch, spi, delay)? {
            self.write_enable(spi, delay)?;
        }

        let mut instruction = Vec::with_capacity(4 + data.len());
        instruction[0] = Instructions::PageProgram.opcode();
        instruction[1] = start_address[0];
        instruction[2] = start_address[1];
        instruction[3] = start_address[2];

        self.write_spi(instruction.as_slice(), spi, delay)?;

        Ok(())
    }

    /// Erase 4K Bytes (setting them to 1s) starting from the address given
    pub fn erase_4k_bytes(
        &mut self, address: [u8; 3], spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        // Set to write enable
        self.write_enable(spi, delay)?;

        self.blocking_wait_for_spi(spi, delay)?;

        let instruction = [
            Instructions::SectorErase.opcode(),
            address[0],
            address[1],
            address[2],
        ];

        self.write_spi(&instruction, spi, delay)?;

        Ok(())
    }

    /// Erase 32 KBytes of data (by setting them to 1s) starting from the address provided.
    pub fn erase_32k_bytes(
        &mut self, address: [u8; 3], spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        // set to write enable
        self.write_enable(spi, delay)?;

        self.blocking_wait_for_spi(spi, delay)?;

        let instruction = [
            Instructions::BlockErase32 as u8,
            address[0],
            address[1],
            address[2],
        ];

        self.write_spi(&instruction, spi, delay)?;

        Ok(())
    }

    /// Erase (by setting to 1s) 64 bytes of data starting from the address given.
    pub fn erase_64k_bytes(
        &mut self, address: [u8; 3], spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        // set to write enable
        self.write_enable(spi, delay)?;

        self.blocking_wait_for_spi(spi, delay)?;

        let instruction = [
            Instructions::BlockErase64 as u8,
            address[0],
            address[1],
            address[2],
        ];

        self.write_spi(&instruction, spi, delay)?;

        Ok(())
    }

    /// Erase (by setting to 1s) all data on the w25q128 chip
    pub fn erase_all_bytes(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        // set to write enable
        self.write_enable(spi, delay)?;

        self.blocking_wait_for_spi(spi, delay)?;

        let instruction = Instructions::ChipErase.opcode();

        self.write_spi(&[instruction], spi, delay)
    }

    /// Suspend the current erase or program in progress.
    pub fn interrupt_erase_or_program(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.write_spi(&[Instructions::EraseProgramSuspend.opcode()], spi, delay)
    }

    /// Resume a previously suspended resume or erase operation.
    pub fn resume_erase_or_program(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.write_spi(&[Instructions::EraseProgramResume.opcode()], spi, delay)
    }

    /// Put the w25q128 into power down mode (only to be interrupted by power_up)
    pub fn power_down(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.write_spi(&[Instructions::PowerDown.opcode()], spi, delay)
    }

    /// Put the w25q128 into power up mode (interrupts the power_down function)
    pub fn power_up(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.write_spi(&[Instructions::ReleasePowerDown.opcode()], spi, delay)
    }

    /// Reads the device ID from the release power down instruction
    pub fn read_device_id(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<u8, W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        let mut buffer = [0u8; 4];
        buffer[0] = Instructions::DeviceID.opcode();

        self.transfer_spi(&mut buffer, spi, delay)?;
        Ok(buffer[1])
    }

    pub fn read_manufacturer_device_id(&mut self, _address: [u8; 3]) -> Result<[u8; 2], W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    /// Read the unique id of the w25q128 device
    /// TODO: Debug this
    pub fn read_unique_id(&mut self) -> Result<u8, W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    pub fn read_jdec_id(&mut self) -> Result<[u8; 3], W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    pub fn read_sfdp_register(&mut self) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    /// Erase the data from the security register given.
    pub fn erase_security_register(
        &mut self, register: SecurityRegister, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        let instructions = match register {
            SecurityRegister::SecurityRegister1 => [Instructions::EraseSecurityRegister as u8, 0x00, 0x10, 0x00],
            SecurityRegister::SecurityRegister2 => [Instructions::EraseSecurityRegister as u8, 0x00, 0x20, 0x00],
            SecurityRegister::SecurityRegister3 => [Instructions::EraseSecurityRegister as u8, 0x00, 0x30, 0x00],
        };

        self.write_spi(&instructions, spi, delay)
    }

    /// Write up to 256 bytes to the given security register
    pub fn write_to_security_register(
        &mut self, register: SecurityRegister, data: &[u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        if data.len() > 256 {
            return Err(W25Q128Error::DataTooLarge);
        }

        self.blocking_wait_for_spi(spi, delay)?;

        let mut instructions = Vec::with_capacity(4 + data.len());
        let start = match register {
            SecurityRegister::SecurityRegister1 => [Instructions::ProgramSecurityRegister as u8, 0x00, 0x10, 0x00],
            SecurityRegister::SecurityRegister2 => [Instructions::ProgramSecurityRegister as u8, 0x00, 0x20, 0x00],
            SecurityRegister::SecurityRegister3 => [Instructions::ProgramSecurityRegister as u8, 0x00, 0x30, 0x00],
        };
        instructions[0..4].copy_from_slice(&start);

        self.transfer_spi(instructions.as_mut_slice(), spi, delay)
    }

    pub fn read_from_security_register(
        &mut self, register: SecurityRegister, buffer: &mut [u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        let mut instructions = Vec::with_capacity(4 + buffer.len());
        let start = match register {
            SecurityRegister::SecurityRegister1 => [Instructions::ReadSecurityRegister as u8, 0x00, 0x10, 0x00],
            SecurityRegister::SecurityRegister2 => [Instructions::ReadSecurityRegister as u8, 0x00, 0x20, 0x00],
            SecurityRegister::SecurityRegister3 => [Instructions::ReadSecurityRegister as u8, 0x00, 0x30, 0x00],
        };
        instructions[..4].copy_from_slice(&start);

        self.transfer_spi(instructions.as_mut_slice(), spi, delay)?;
        buffer[..].copy_from_slice(&instructions[4..]);
        Ok(())
    }

    pub fn lock_individual_block(&mut self, _address: [u8; 3]) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    pub fn unlock_individual_block(&mut self, _address: [u8; 3]) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    pub fn read_block_lock(&mut self, _address: [u8; 3]) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        todo!()
    }

    /// Write lock all of the data on the device
    pub fn lock_device_data(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        self.write_spi(&[Instructions::GlobalBlockLock.opcode()], spi, delay)
    }

    pub fn unlock_device_data(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.blocking_wait_for_spi(spi, delay)?;

        self.write_spi(&[Instructions::GlobalBlockUnlock.opcode()], spi, delay)
    }

    pub fn reset_device(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.write_spi(&[Instructions::EnableReset.opcode()], spi, delay)?;
        self.blocking_wait_for_ms(30, delay);
        self.write_spi(&[Instructions::ResetDevice as u8], spi, delay)?;
        self.blocking_wait_for_ms(30, delay);
        Ok(())
    }

    pub fn leave_qpi(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.write_spi(&[0xFF], spi, delay)
    }

    fn blocking_wait_for_spi(
        &mut self, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        loop {
            if !self.check_status_one(StatusReg1::Busy, spi, delay)? {
                break;
            }

            delay.delay_us(self.wait_delay);
        }

        Ok(())
    }

    fn blocking_wait_for_ms(&mut self, ms: u32, delay: &mut dyn DelayUs<u32>) {
        delay.delay_us(ms * 1_000);
    }

    fn write_spi(&mut self, data: &[u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.csn.set_low().map_err(W25Q128Error::GpioError)?;
        let spi_err = spi.write(data);
        let gpio_err = self.csn.set_high();

        delay.delay_us(1);

        match (spi_err, gpio_err) {
            (Err(spi), Err(gpio)) => Err(W25Q128Error::SpiGpioError((spi, gpio))),
            (Err(spi), _) => Err(W25Q128Error::SpiError(spi)),
            (_, Err(gpio)) => Err(W25Q128Error::GpioError(gpio)),
            _ => Ok(())
        }
    }

    fn transfer_spi(&mut self, buffer: &mut [u8], spi: &mut SPI, delay: &mut dyn DelayUs<u32>) -> Result<(), W25Q128Error<SPIE, GPIOE>> {
        self.csn.set_low().map_err(W25Q128Error::GpioError)?;
        let spi_err = spi.transfer(buffer);
        let gpio_err = self.csn.set_high();

        delay.delay_us(1);

        match (spi_err, gpio_err) {
            (Err(spi), Err(gpio)) => Err(W25Q128Error::SpiGpioError((spi, gpio))),
            (Err(spi), _) => Err(W25Q128Error::SpiError(spi)),
            (_, Err(gpio)) => Err(W25Q128Error::GpioError(gpio)),
            _ => Ok(())
        }
    }

    pub fn check_status_one(
        &mut self, status: StatusReg1, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<bool, W25Q128Error<SPIE, GPIOE>> {
        let register_status = self.read_from_status_register(StatusRegister::StatusRegister1, spi, delay)?;
        Ok(register_status.contains_status(status))
    }

    pub fn check_status_two(
        &mut self, status: StatusReg2, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<bool, W25Q128Error<SPIE, GPIOE>> {
        let register_status = self.read_from_status_register(StatusRegister::StatusRegister2, spi, delay)?;
        Ok(register_status.contains_status(status))
    }

    pub fn check_status_three(
        &mut self, status: StatusReg3, spi: &mut SPI, delay: &mut dyn DelayUs<u32>
    ) -> Result<bool, W25Q128Error<SPIE, GPIOE>> {
        let register_status = self.read_from_status_register(StatusRegister::StatusRegister3, spi, delay)?;
        Ok(register_status.contains_status(status))
    }
}