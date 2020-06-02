# LoRaWAN Demo implementation in Rust
Uses Semtech's SX1261 LoRa modem to send messages to a LoRaWAN Gateway and receive messages from it. Runs on the STM32F303VC CPU, but aims to be generic enough be easily ported to use other CPUs.

*This project is for experimenting and demontration purposes, not for production use*

## MVP
This project aims to implement the following functionalities:

 - Initialize the SX1261 using SPI
 - Send a LoRaWAN message
 - Receive a LoRaWAN message
 - Minimize power usage 


[SX1261/62 Datasheet](https://drive.google.com/file/d/1_Fcxab7j0AaavaMOK3st-larq5S1oWow/view?usp=sharing)