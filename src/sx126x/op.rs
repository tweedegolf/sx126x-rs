#[repr(u8)]
pub enum PacketType {
    TODO
}
#[repr(u8)]
pub enum StandbyConfig {
    TODO
}

pub struct Status {}

impl From<u8> for Status {
    fn from(_: u8) -> Self {
        todo!()
    }
}

pub struct CalibParam {}

impl Into<u8> for CalibParam {
    fn into(self) -> u8 {
        todo!()
    }
}

#[repr(u8)]
pub enum TcxoVoltage {
    TODO
}

pub struct TcxoDelay {}

impl Into<[u8; 3]> for TcxoDelay {
    fn into(self) -> [u8; 3] {
        todo!()
    }
}

pub struct DeviceErrors{}

impl From<u16> for DeviceErrors{
    fn from(_: u16) -> Self {
        todo!()
    }
}

