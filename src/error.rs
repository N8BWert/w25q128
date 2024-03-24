//!
//! Errors for Operating the W25Q128 Module
//! 

#[derive(Clone, Copy)]
pub enum W25Q128Error<SPIE, GPIOE> {
    NoDelay,
    UnableToSetWriteEnable,
    UnableToResetWriteEnable,
    DataTooLarge,
    SpiError(SPIE),
    GpioError(GPIOE),
    SpiGpioError((SPIE, GPIOE)),
}