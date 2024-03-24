//!
//! Device Implementation for the W25Q128 Module
//! 
#![no_std]

use core::fmt::Debug;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::DelayUs;

use error::convert_error;

pub mod instruction;
use crate::instruction::Instructions;

pub mod status;
use crate::status::{StatusRegister, ContainsStatus, CheckForStatus, StatusReg1, StatusReg2, StatusReg3, SecurityRegister};

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
pub struct W25Q128<E: Debug, CSN: OutputPin<Error = E>, SPI: Transfer<u8, Error = E> + Write<u8, Error = E>, DELAY: DelayUs<u32>> {
    csn: CSN,
    spi: SPI,
    delay: Option<DELAY>,
    volatile: bool,
    wait_delay: u32,
}

impl<E: Debug, CSN: OutputPin<Error = E>, SPI: Transfer<u8, Error = E> + Write<u8, Error = E>, DELAY: DelayUs<u32>> W25Q128<E, CSN, SPI, DELAY> {
    pub fn new(
        csn: CSN,
        spi: SPI,
        delay: DELAY,
    ) -> Self {
        Self { csn, spi, delay: Some(delay), volatile: false, wait_delay: 10 }
    }

    pub fn new_with_wait_delay(
        csn: CSN,
        spi: SPI,
        delay: DELAY,
        wait_delay: u32,
    ) -> Self {
        Self { csn, spi, delay: Some(delay), volatile: false, wait_delay }
    }

    /// If the delay is able to be removed from the driver, then it must be able to be added to the
    /// driver.
    pub fn add_delay(&mut self, delay: DELAY) {
        self.delay = Some(delay);
    }

    /// When using RTIC it is often a lot easier to just have drivers take control over the specific peripheral,
    /// timer, or what-have-you so this alows the delay to be removed from the W25Q128 driver.
    pub fn take_delay(&mut self) -> Option<DELAY> {
        self.delay.take()
    }

    /// Sets the WriteEnable status bit for the w25q128 allowing the memory to be written to.
    /// 
    /// It is very unlikely you should ever run this function on its own as I'm making concerted
    /// effort to run this in the function that actually writes to memory.  However, I may miss
    /// something so I'm allowing this to be called (but please don't).
    pub fn write_enable(&mut self) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instruction = Instructions::WriteEnable as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        let register_1 = self.read_from_status_register(StatusRegister::StatusRegister1)?;

        if !register_1.contains_status(StatusReg1::WriteEnableLatch) {
            return Err(W25Q128Error::UnableToSetWriteEnable);
        }

        Ok(())
    }

    /// Used to turn the non-volatile status registers into volatile bits so they can be changed quickly
    /// without waitin for the typical non-volatile bit write cycles.
    pub fn volatile_sr_write_enable(&mut self) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instruction = Instructions::VolatileSrWriteEnable as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

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
    pub fn write_disable(&mut self) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instruction = Instructions::WriteDisable as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        let register_1 = self.read_from_status_register(StatusRegister::StatusRegister1)?;
        if register_1.contains_status(StatusReg1::WriteEnableLatch) {
            return Err(W25Q128Error::UnableToResetWriteEnable);
        }

        Ok(())
    }

    /// Read the current value from a given status register.
    pub fn read_from_status_register(&mut self, status_register: StatusRegister) -> Result<u8, W25Q128Error<E>> {        
        if self.delay.is_none() {
            return Err(W25Q128Error::NoDelay);
        }

        let instruction = match status_register {
            StatusRegister::StatusRegister1 => Instructions::ReadStatusReg1 as u8,
            StatusRegister::StatusRegister2 => Instructions::ReadStatusReg2 as u8,
            StatusRegister::StatusRegister3 => Instructions::ReadStatusReg3 as u8,
        };
        let mut register_buffer = [0x00];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.spi.transfer(&mut register_buffer).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(register_buffer[0])
    }

    pub fn write_to_status_register(&mut self, status_register: StatusRegister, status: u8) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let base_instruction = match status_register {
            StatusRegister::StatusRegister1 => Instructions::WriteStatusReg1 as u8,
            StatusRegister::StatusRegister2 => Instructions::WriteStatusReg2 as u8,
            StatusRegister::StatusRegister3 => Instructions::WriteStatusReg3 as u8,
        };

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[base_instruction, status]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    // Below function would need malloc, which I'd like to stay away from, but I'll keep the method signature so that
    // if it is needed in the future it can be used.
    // pub fn read_data(&mut self, bytes: u32) -> Result<Vec<u8>, E>

    /// Read data from the w25q128 into the buffer provided.
    pub fn read_to_buffer(&mut self, start_address: [u8; 3], buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let read_instruction = [
            Instructions::ReadData as u8,
            start_address[0],
            start_address[1],
            start_address[2]
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&read_instruction).map_err(convert_error)?;
        self.spi.transfer(buffer).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    pub fn fast_read_to_buffer(&mut self, start_address: [u8; 3], buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        // TODO: Make this better
        let read_instruction = [
            Instructions::FastRead as u8,
            start_address[0],
            start_address[1],
            start_address[2],
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&read_instruction).map_err(convert_error)?;
        self.spi.transfer(buffer).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    // TODO: I'm not 100% certain how to implement this in rust.  It will require merging what the spi
    // interface thinks is an input line with the output line, which I'm not 100% sure how to do.
    #[cfg(feature = "dual-spi")]
    pub fn fast_read_dual_output_to_buffer(&mut self, start_address: [u8; 3], buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    // TODO: My module doesn't have quad spi capabilities so I can't exactly test this
    #[cfg(feature = "quad-spi")]
    pub fn fast_read_quad_output_to_buffer(&mut self, start_address: [u8; 3], buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    // TODO: I'm not 100% certain how to implement this in rust.  It will require both merging as in
    // fast_read_dual_output_to_buffer, but also splitting the start_address between the two lines
    #[cfg(feature = "dual-spi")]
    pub fn fast_read_dual_io_to_buffer(&mut self, start_address: [u8; 3], buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    // TODO: My module doesn't have quad spi capabilities so I can't test this
    #[cfg(feature = "quad-spi")]
    pub fn fast_read_quad_io_to_buffer(&mut self, start_address: [u8; 3], buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    // TODO: My module doesn't have quad spi capabilities so I can't test this
    #[cfg(feature = "quad-spi")]
    pub fn set_burst_with_wrap(&mut self, w4: bool, w5: bool, w6: bool) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    /// Writes a given amount of data to the w25q128 with wrapping bytes.  This function is
    /// more of a convenience and may block if the data will overflow a single page.
    pub fn write_bytes(&mut self, start_address: [u8; 3], data: &[u8]) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        if !self.check_for_status(StatusReg1::WriteEnableLatch)? {
            self.write_enable()?
        }

        // TODO: Check that the right length of data was provided

        let instruction = [
            Instructions::PageProgram as u8,
            start_address[0],
            start_address[1],
            start_address[2],
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instruction).map_err(convert_error)?;
        self.spi.write(data).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        // TODO: Handle case where data is more than 256 bytes

        // According to the data sheet it takes a maximum of 3ms for a page program to take
        self.blocking_wait_for_ms(3)?;

        Ok(())
    }

    pub fn quick_write_bytes(&mut self, start_address: [u8; 3], data: &[u8]) -> Result<(), W25Q128Error<E>> {
        self.write_enable()?;

        self.blocking_wait_for_ms(5)?;

        let instruction = [
            Instructions::PageProgram as u8,
            start_address[0],
            start_address[1],
            start_address[2],
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instruction).map_err(convert_error)?;
        self.spi.write(data).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        self.blocking_wait_for_ms(3)?;
        
        Ok(())
    }

    /// Writes exactly a page (or 256 bytes) to a memory location identified by the first 16 significant
    /// bytes.
    pub fn write_page(&mut self, first_bytes: [u8; 2], page: &[u8; 256]) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        self.write_enable()?;

        let instruction = [
            Instructions::PageProgram as u8,
            first_bytes[0],
            first_bytes[1],
            0x00,
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instruction).map_err(convert_error)?;
        self.spi.write(page).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        // According to the data sheet it takes a maximum of 3ms for a page program to take
        self.blocking_wait_for_ms(3)?;

        Ok(())
    }

    // TODO: My module doesn't have quad spi capabilities so I can't test this
    #[cfg(feature = "quad-spi")]
    pub fn write_bytes_quad_input(&mut self, start_address: [u8; 3], data: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    // TODO: My module doesn't have quad spi capabilities so I can't test this
    #[cfg(feature = "quad-spi")]
    pub fn write_page_quad_input(&mut self, start_address: [u8; 3], data: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    /// Erase 4K Bytes (setting them to 1s) starting from the address given
    pub fn erase_4k_bytes(&mut self, address: [u8; 3]) -> Result<(), W25Q128Error<E>> {
        // Set to write enable
        self.write_enable()?;

        self.blocking_wait_for_spi()?;

        let instruction = [
            Instructions::SectorErase as u8,
            address[0],
            address[1],
            address[2],
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instruction).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Erase 32 KBytes of data (by setting them to 1s) starting from the address provided.
    pub fn erase_32k_bytes(&mut self, address: [u8; 3]) -> Result<(), W25Q128Error<E>> {
        // set to write enable
        self.write_enable()?;

        self.blocking_wait_for_spi()?;

        let instruction = [
            Instructions::BlockErase32 as u8,
            address[0],
            address[1],
            address[2],
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instruction).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Erase (by setting to 1s) 64 bytes of data starting from the address given.
    pub fn erase_64k_bytes(&mut self, address: [u8; 3]) -> Result<(), W25Q128Error<E>> {
        // set to write enable
        self.write_enable()?;

        self.blocking_wait_for_spi()?;

        let instruction = [
            Instructions::BlockErase64 as u8,
            address[0],
            address[1],
            address[2],
        ];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instruction).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Erase (by setting to 1s) all data on the w25q128 chip
    pub fn erase_all_bytes(&mut self) -> Result<(), W25Q128Error<E>> {
        // set to write enable
        self.write_enable()?;

        self.blocking_wait_for_spi()?;

        let instruction = Instructions::ChipErase as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Suspend the current erase or program in progress.
    pub fn interrupt_erase_or_program(&mut self) -> Result<(), W25Q128Error<E>> {
        if self.delay.is_none() {
            return Err(W25Q128Error::NoDelay);
        }

        let instruction = Instructions::EraseProgramSuspend as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Resume a previously suspended resume or erase operation.
    pub fn resume_erase_or_program(&mut self) -> Result<(), W25Q128Error<E>> {
        if self.delay.is_none() {
            return Err(W25Q128Error::NoDelay);
        }

        let instruction = Instructions::EraseProgramResume as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_low().map_err(convert_error)?;

        Ok(())
    }

    /// Put the w25q128 into power down mode (only to be interrupted by power_up)
    pub fn power_down(&mut self) -> Result<(), W25Q128Error<E>> {
        if self.delay.is_none() {
            return Err(W25Q128Error::NoDelay);
        }

        let instruction = Instructions::PowerDown as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Put the w25q128 into power up mode (interrupts the power_down function)
    pub fn power_up(&mut self) -> Result<(), W25Q128Error<E>> {
        if self.delay.is_none() {
            return Err(W25Q128Error::NoDelay);
        }

        let instruction = Instructions::ReleasePowerDown as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Reads the device ID from the release power down instruction
    pub fn read_device_id(&mut self) -> Result<u8, W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let mut buffer = [0u8; 3];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[Instructions::ReleasePowerDown as u8]).map_err(convert_error)?;
        self.spi.transfer(&mut buffer).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(buffer[0])
    }

    // TODO: Not sure why this is useful so it will implemented later
    pub fn read_manufacturer_device_id(&mut self, _address: [u8; 3]) -> Result<[u8; 2], W25Q128Error<E>> {
        todo!()
    }

    // TODO: Still not sure why this is necessary
    #[cfg(feature = "dual-spi")]
    pub fn read_manufacturer_device_id_dual_io(&mut self, address: [u8; 3]) -> Result<[u8; 2], W25Q128Error<E>> {
        todo!()
    }

    // TODO: Still not sure why this is necessary
    #[cfg(feature = "quad-spi")]
    pub fn read_manufacturer_device_id_quad_spi(&mut self, address: [u8; 3]) -> Result<[u8; 2], W25Q128Error<E>> {
        todo!()
    }

    /// Read the unique id of the w25q128 device
    /// TODO: Debug this
    pub fn read_unique_id(&mut self) -> Result<u8, W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let mut buffer = [0u8; 4];

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[Instructions::ReadUniqueId as u8]).map_err(convert_error)?;
        self.spi.transfer(&mut buffer).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(buffer[0])
    }

    pub fn read_jdec_id(&mut self) -> Result<[u8; 3], W25Q128Error<E>> {
        todo!()
    }

    pub fn read_sfdp_register(&mut self) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    /// Erase the data from the security register given.
    pub fn erase_security_register(&mut self, register: SecurityRegister) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instructions = match register {
            SecurityRegister::SecurityRegister1 => [Instructions::EraseSecurityRegister as u8, 0x00, 0x10, 0x00],
            SecurityRegister::SecurityRegister2 => [Instructions::EraseSecurityRegister as u8, 0x00, 0x20, 0x00],
            SecurityRegister::SecurityRegister3 => [Instructions::EraseSecurityRegister as u8, 0x00, 0x30, 0x00],
        };

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instructions).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Write up to 256 bytes to the given security register
    pub fn write_to_security_register(&mut self, register: SecurityRegister, data: &[u8]) -> Result<(), W25Q128Error<E>> {
        if data.len() > 256 {
            return Err(W25Q128Error::DataTooLarge);
        }

        self.blocking_wait_for_spi()?;

        let instructions = match register {
            SecurityRegister::SecurityRegister1 => [Instructions::ProgramSecurityRegister as u8, 0x00, 0x10, 0x00],
            SecurityRegister::SecurityRegister2 => [Instructions::ProgramSecurityRegister as u8, 0x00, 0x20, 0x00],
            SecurityRegister::SecurityRegister3 => [Instructions::ProgramSecurityRegister as u8, 0x00, 0x30, 0x00],
        };

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instructions).map_err(convert_error)?;
        self.spi.write(data).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    /// Read the data from the specified security register into the provided buffer.
    pub fn read_from_security_register_to_buffer(&mut self, register: SecurityRegister, buffer: &mut [u8]) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instructions = match register {
            SecurityRegister::SecurityRegister1 => [Instructions::ReadSecurityRegister as u8, 0x00, 0x10, 0x00],
            SecurityRegister::SecurityRegister2 => [Instructions::ReadSecurityRegister as u8, 0x00, 0x20, 0x00],
            SecurityRegister::SecurityRegister3 => [Instructions::ReadSecurityRegister as u8, 0x00, 0x30, 0x00],
        };

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&instructions).map_err(convert_error)?;
        self.spi.transfer(buffer).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    pub fn lock_individual_block(&mut self, _address: [u8; 3]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    pub fn unlock_individual_block(&mut self, _address: [u8; 3]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    pub fn read_block_lock(&mut self, _address: [u8; 3]) -> Result<(), W25Q128Error<E>> {
        todo!()
    }

    /// Write lock all of the data on the device
    pub fn lock_device_data(&mut self) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instruction = Instructions::GlobalBlockLock as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    pub fn unlock_device_data(&mut self) -> Result<(), W25Q128Error<E>> {
        self.blocking_wait_for_spi()?;

        let instruction = Instructions::GlobalBlockUnlock as u8;

        self.csn.set_low().map_err(convert_error)?;
        self.spi.write(&[instruction]).map_err(convert_error)?;
        self.csn.set_high().map_err(convert_error)?;

        Ok(())
    }

    pub fn reset_device(&mut self) -> Result<(), W25Q128Error<E>> {
        if let Some(delay) = self.delay.as_mut() {
            self.csn.set_low().map_err(convert_error)?;
            self.spi.write(&[Instructions::EnableReset as u8]).map_err(convert_error)?;
            self.csn.set_high().map_err(convert_error)?;

            delay.delay_us(30_000);
            
            self.csn.set_low().map_err(convert_error)?;
            self.spi.write(&[Instructions::ResetDevice as u8]).map_err(convert_error)?;
            self.csn.set_high().map_err(convert_error)?;

            delay.delay_us(30_000);

            return Ok(());
        }
        Err(W25Q128Error::NoDelay)
    }

    fn blocking_wait_for_spi(&mut self) -> Result<(), W25Q128Error<E>> {
        if self.delay.is_none() {
            return Err(W25Q128Error::NoDelay);
        }

        loop {
            if !self.check_for_status(StatusReg1::Busy)? {
                break;
            }

            self.delay.as_mut().unwrap().delay_us(self.wait_delay);
        }

        Ok(())
    }

    fn blocking_wait_for_ms(&mut self, ms: u32) -> Result<(), W25Q128Error<E>> {
        if let Some(delay) = self.delay.as_mut() {
            delay.delay_us(ms * 1_000);
        } else {
            return Err(W25Q128Error::NoDelay);
        }

        Ok(())
    }
}

impl<E: Debug, CSN: OutputPin<Error = E>, SPI: Transfer<u8, Error = E> + Write<u8, Error = E>, DELAY: DelayUs<u32>> CheckForStatus<W25Q128Error<E>, StatusReg1> for W25Q128<E, CSN, SPI, DELAY> {
    /// Check that the w25q128 contains any of the statuses given in the first status register.
    fn check_for_status(&mut self, status: StatusReg1) -> Result<bool, W25Q128Error<E>> {
        let register_status = self.read_from_status_register(StatusRegister::StatusRegister1)?;
        Ok(register_status.contains_status(status))
    }
}

impl<E: Debug, CSN: OutputPin<Error = E>, SPI: Transfer<u8, Error = E> + Write<u8, Error = E>, DELAY: DelayUs<u32>> CheckForStatus<W25Q128Error<E>, StatusReg2> for W25Q128<E, CSN, SPI, DELAY> {
    /// Check that the w25q128 contains any of the statuses given in the second status register.
    fn check_for_status(&mut self, status: StatusReg2) -> Result<bool, W25Q128Error<E>> {
        let register_status = self.read_from_status_register(StatusRegister::StatusRegister2)?;
        Ok(register_status.contains_status(status))
    }
}

impl<E: Debug, CSN: OutputPin<Error = E>, SPI: Transfer<u8, Error = E> + Write<u8, Error = E>, DELAY: DelayUs<u32>> CheckForStatus<W25Q128Error<E>, StatusReg3> for W25Q128<E, CSN, SPI, DELAY> {
    /// Check that the w25q128 contains any of the statuses given in the third status register.
    fn check_for_status(&mut self, status: StatusReg3) -> Result<bool, W25Q128Error<E>> {
        let register_status = self.read_from_status_register(StatusRegister::StatusRegister3)?;
        Ok(register_status.contains_status(status))
    }
}