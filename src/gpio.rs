pub struct DisconnectedPin;

impl embedded_hal::digital::v2::OutputPin for DisconnectedPin {
    type Error = core::convert::Infallible;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal::digital::v2::InputPin for DisconnectedPin {
    type Error = core::convert::Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(false)
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}
