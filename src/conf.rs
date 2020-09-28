use super::op::*;

/// Configuration parameters.
/// Used to initialize the SX126x modem
pub struct Config {
    /// Packet type
    pub packet_type: PacketType,
    /// LoRa sync word
    pub sync_word: u16,
    /// Calibration parameters
    pub calib_param: CalibParam,
    /// Modulation parameters
    pub mod_params: ModParams,
    /// Power-amplifier configuration
    pub pa_config: PaConfig,
    /// Packet parameters. Set tot none if you want to configure
    /// these later
    pub packet_params: Option<PacketParams>,
    /// TX parameters
    pub tx_params: TxParams,
    /// DIO1 IRQ mask
    pub dio1_irq_mask: IrqMask,
    /// DIO2 IRQ mask
    pub dio2_irq_mask: IrqMask,
    /// DIO3 IRW mask
    pub dio3_irq_mask: IrqMask,
    /// RF freq, calculated using crate::calc_rf_freq
    pub rf_freq: u32,
    /// RF frequency in MHz
    pub rf_frequency: u32,
}
