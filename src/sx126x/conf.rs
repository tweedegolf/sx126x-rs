use super::op::*;

pub struct Config {
    pub packet_type: PacketType,
    pub standby_config: StandbyConfig,
    pub sync_word: u16,
    #[cfg(feature = "tcxo")]
    pub tcxo_delay: TcxoDelay,
    #[cfg(feature = "tcxo")]
    pub tcxo_voltage: TcxoVoltage,
    pub calib_param: CalibParam,
    pub packet_params: PacketParams,
    pub mod_params: ModParams,
    pub pa_config: PaConfig,
    pub tx_params: TxParams,
    pub dio1_irq_mask: IrqMask,
    pub rf_freq: u32,
}
