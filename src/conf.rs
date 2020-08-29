use super::op::*;

/// Configuration parameters
pub struct Config {
    pub packet_type: PacketType,
    pub standby_config: StandbyConfig,
    pub sync_word: u16,
    pub calib_param: CalibParam,
    pub mod_params: ModParams,
    pub pa_config: PaConfig,
    pub tx_params: TxParams,
    pub dio1_irq_mask: IrqMask,
    pub dio2_irq_mask: IrqMask,
    pub dio3_irq_mask: IrqMask,
    pub rf_freq: u32,
    pub rf_frequency: u32,
}
