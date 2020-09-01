use embedded_hal::digital::v2::OutputPin;

pub type OutputPinError<TPIN> = <TPIN as OutputPin>::Error;

pub enum SpiError<TWERR, TTERR> {
    Write(TWERR),
    Transfer(TTERR),
}
