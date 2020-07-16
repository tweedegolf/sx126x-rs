use core::marker::PhantomData;

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

pub mod init;
pub mod status;
pub mod calib;
pub mod tcxo;
pub mod err;
pub mod irq;
pub mod tx;
pub mod packet;
pub mod modulation;

pub use init::*;
pub use status::*;
pub use calib::*;
pub use tcxo::*;
pub use err::*;
pub use irq::*;
pub use tx::*;
pub use packet::*;
pub use modulation::*;