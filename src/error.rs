//!
//! Errors for Operating the W25Q128 Module
//! 

use core::fmt::Debug;
use defmt::Format;

pub fn convert_error<E: Debug>(err: E) -> W25Q128Error<E> { W25Q128Error::Other(err) }

#[derive(Debug, Clone, Copy, Format)]
pub enum W25Q128Error<E> {
    NoDelay,
    UnableToSetWriteEnable,
    UnableToResetWriteEnable,
    DataTooLarge,
    Other(E),
}