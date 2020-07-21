#[repr(u8)]
#[derive(Copy, Clone)]
pub enum PacketType {
    GFSK = 0x00,
    LoRa = 0x01,
}

pub struct PacketParams {
    inner: [u8; 9],
}

impl Into<[u8; 9]> for PacketParams {
    fn into(self) -> [u8; 9] {
        self.inner
    }
}

pub mod lora {
    use super::PacketParams;

    #[repr(u8)]
    #[derive(Copy, Clone)]
    pub enum LoRaHeaderType {
        /// Variable length packet (explicit header)
        VarLen = 0x00,
        /// Fixed length packet (implicit header)
        FixedLen = 0x01,
    }

    #[repr(u8)]
    #[derive(Copy, Clone)]
    pub enum LoRaCrcType {
        /// CRC off
        CrcOff = 0x00,
        /// CRC on
        CrcOn = 0x01,
    }

    #[repr(u8)]
    #[derive(Copy, Clone)]
    pub enum LoRaInvertIq {
        /// Standard IQ setup
        Standard = 0x00,
        /// Inverted IQ setup
        Inverted = 0x01,
    }

    pub struct LoRaPacketParams {
        /// preamble length: number of symbols sent as preamble
        /// The preamble length is a 16-bit value which represents
        /// the number of LoRa® symbols which will be sent by the radio.
        preamble_len: u16, // 1, 2
        /// Header type. When the byte headerType is at 0x00,
        /// the payload length, coding rate and the header
        /// CRC will be added to the LoRa® header and transported
        /// to the receiver.
        header_type: LoRaHeaderType, // 3
        /// Size of the payload (in bytes) to transmit or maximum size of the
        /// payload that the receiver can accept.
        payload_len: u8, // 4
        /// CRC type
        crc_type: LoRaCrcType, // 5
        /// Invert IW
        invert_iq: LoRaInvertIq,
    }

    impl Into<PacketParams> for LoRaPacketParams {
        fn into(self) -> PacketParams {
            let preamble_len = self.preamble_len.to_be_bytes();

            PacketParams {
                inner: [
                    preamble_len[0],
                    preamble_len[1],
                    self.header_type as u8,
                    self.payload_len,
                    self.crc_type as u8,
                    self.invert_iq as u8,
                    0x00,
                    0x00,
                    0x00,
                ],
            }
        }
    }

    impl Default for LoRaPacketParams {
        fn default() -> Self {
            Self {
                preamble_len: 0x0000,
                header_type: LoRaHeaderType::VarLen,
                payload_len: 0x00,
                crc_type: LoRaCrcType::CrcOff,
                invert_iq: LoRaInvertIq::Standard,
            }
        }
    }

    impl LoRaPacketParams {
        pub fn set_preamble_len(mut self, preamble_len: u16) -> Self {
            self.preamble_len = preamble_len;
            self
        }

        pub fn set_header_type(mut self, header_type: LoRaHeaderType) -> Self {
            self.header_type = header_type;
            self
        }

        pub fn set_payload_len(mut self, payload_len: u8) -> Self {
            self.payload_len = payload_len;
            self
        }

        pub fn set_crc_type(mut self, crc_type: LoRaCrcType) -> Self {
            self.crc_type = crc_type;
            self
        }

        pub fn set_invert_iq(mut self, invert_iq: LoRaInvertIq) -> Self {
            self.invert_iq = invert_iq;
            self
        }
    }
}
