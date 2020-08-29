#[repr(u8)]
#[derive(Copy, Clone)]
pub enum StandbyConfig {
    StbyRc = 0x00,
    StbyXOSC = 0x01,
}
