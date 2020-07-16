use super::op::*;

pub struct Config<TF1, TF2> {
    pub freq_1: TF1,
    pub freq_2: TF2,
    pub packet_type: PacketType,
    pub standby_config: StandbyConfig,
    pub sync_word: u16,
    #[cfg(feature = "tcxo")]
    pub tcxo_delay: TcxoDelay,
    #[cfg(feature = "tcxo")]
    pub tcxo_voltage: TcxoVoltage,
    pub packet_params: PacketParams,
    pub mod_params: ModParams,
    pub tx_params: TxParams,
}