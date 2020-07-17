// Divide a value in a quick fashion using multiplication and shifting
macro_rules! divide {
    ($v: ident, 15.625) => {
        // 15.625 == 125/8
        // -> 8/125 ~ 131/2048
        // -> 2048 = 2^11
        // -> v / 15.625 ~ (v * 131) >> 11
        ($v * 131) >> 11
    };
}

pub mod calib;
pub mod err;
pub mod init;
pub mod irq;
pub mod modulation;
pub mod packet;
pub mod status;
pub mod tcxo;
pub mod tx;

pub use calib::*;
pub use err::*;
pub use init::*;
pub use irq::*;
pub use modulation::*;
pub use packet::*;
pub use status::*;
pub use tcxo::*;
pub use tx::*;
