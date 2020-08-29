#[allow(dead_code)]
#[repr(u16)]
/// Every register defined in the SX126X datasheet
pub enum Register {
    /// Non-standard DIOx control
    DioxOutputEnable = 0x0580,
    /// Non-standard DIOx control
    DioxInputEnable = 0x0583,
    /// Non-standard DIOx control
    DioxPullUpControl = 0x0584,
    /// Non-standard DIOx control
    DioxPullDownControl = 0x0585,
    /// Initial value used for the whitening LFSR in
    /// FSK mode; MSB. The user should not change the value of the 7
    /// MSB of this register
    WhiteningInitialValueMsb = 0x06B8,
    /// Initial value used for the whitening LFSR in
    /// FSK mode; LSB
    WhiteningInitialValueLsb = 0x06B9,
    /// Initial value used for the polynomial used to
    /// compute the CRC in FSK mode; LSB
    CrcMsbInitialValue = 0x06BC,
    /// Initial value used for the polynomial used to
    /// compute the CRC in FSK mode; LSB
    CrcLsbInitialValue = 0x006BD,
    /// Polynomial used to compute the CRC in FSK mode; MSB
    CrcMsbPolynomialValue = 0x06BE,
    /// Polynomial used to compute the CRC in FSK mode; LSB
    CrcLsbPolynomialValue = 0x06BF,
    /// 1st byte of the Sync Word in FSK mode
    SyncWord0 = 0x06C0,
    /// 2nd byte of the Sync Word in FSK mode
    SyncWord1 = 0x06C1,
    /// 3rd byte of the Sync Word in FSK mode
    SyncWord2 = 0x06C2,
    /// 4th byte of the Sync Word in FSK mode
    SyncWord3 = 0x06C3,
    /// 5th byte of the Sync Word in FSK mode
    SyncWord4 = 0x06C4,
    /// 6th byte of the Sync Word in FSK mode
    SyncWord5 = 0x06C5,
    /// 7th byte of the Sync Word in FSK mode
    SyncWord6 = 0x06C6,
    /// 8th byte of the Sync Word in FSK mode
    SyncWord7 = 0x06C7,
    /// Node Address used in FSK mode
    NodeAddress = 0x06CD,
    /// Broadcast Address used in FSK mode
    BroadcastAddress = 0x06CE,
    /// Optimize the inverted IQ operation
    IqPolaritySetup = 0x0736,
    /// Differantiate the LoRa signal for Public or Private Network; MSB
    /// Set to 0x3444 for Public Netwok
    /// Set to 0x1424 for Private Network
    LoRaSyncWordMsb = 0x0740,
    /// Differantiate the LoRa signal for Public or Private Network; LSB
    /// Set to 0x3444 for Public Netwok
    /// Set to 0x1424 for Private Network
    LoRaSyncWordLsb = 0x0741,
    /// Can be used to get a 32-bit random numer; 1st byte
    RandomNumberGen0 = 0x0819,
    /// Can be used to get a 32-bit random numer; 2nd byte
    RandomNumberGen1 = 0x081A,
    /// Can be used to get a 32-bit random numer; 3rd byte
    RandomNumberGen2 = 0x081B,
    /// Can be used to get a 32-bit random numer; 4th byte
    RandomNumberGen3 = 0x081C,
    /// Refer to Section 15 of the Data Sheet
    TxModulaton = 0x0889,
    /// Set the gain used in Rx mode:
    /// Rx Power Saving gain: 0x94,
    /// Rx Boosted gain: 0x96
    RxGain = 0x08AC,
    /// Refer to Section 15 of the Data Sheet
    TxClampConfig = 0x08D8,
    /// Set the Over Current Protection level.
    /// The value is changed internally depending
    /// on the device selected. Default values are:
    /// SX1262: 0x38 (140mA)
    /// SX1261; 0x18: (60mA)
    OcpConfiguration = 0x08E7,
    /// Enable or disable RTC Timer
    RtcControl = 0x0902,
    /// Value of the trimming cap on XTA pin.
    /// This register should only be changed while
    /// the radio is in STDBY_XOSC mode
    XtaTrim = 0x0911,
    /// Value of the trimming cap on XTB pin.
    /// This register should only be changed while
    /// the radio is in STDBY_XOSC mode
    XtbTrim = 0x0912,
    /// Non-standard DIO3 control
    Dio3OutputVoltageControl = 0x0920,
    /// Used to clear events
    EventMask = 0x0944,
}
