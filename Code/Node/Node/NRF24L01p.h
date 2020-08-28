/*
 * NRF24L01p.h
 *
 * Created: 26.03.2020 18:22:03
 *  Author: Jochen
 */ 


#ifndef NRF24L01P_H_
#define NRF24L01P_H_

#include <stdint.h>


void nrf_init(uint8_t channel, uint8_t dataRate, uint8_t addressWidth, uint8_t CRCLength);

void nrf_powerUp();
void nrf_powerDown();
uint8_t nrf_isPoweredUp();

void nrf_setModeRX(void);
void nrf_setModeTX(void);

// Ensure that dynamic Ack is enabled if you set getAck=false.
// If auto ack is disabled, do net set getAck=0.
void nrf_writeTXData(uint8_t* data, uint8_t len);
void nrf_writeTXDataNoAck(uint8_t* data, uint8_t len);

// TODO: maybe implement fast write method (just writing to the TX FIFO without sending directly)
// uint8_t writeTXDataFast(uint8_t* data, uint8_t len, uint8_t getAck = true);

void nrf_writeAckData(uint8_t pipe, uint8_t* data, uint8_t len);

void nrf_startSending(void);

void nrf_readRXData(uint8_t* data, uint8_t* len, uint8_t* pipe);

uint8_t nrf_getStatus(void);

void nrf_maskIRQ(uint8_t rx_ready, uint8_t tx_done, uint8_t tx_maxRetry);
uint8_t nrf_getIRQStatus(uint8_t* rx_ready, uint8_t* tx_done, uint8_t* tx_maxRetry);
void nrf_resetIRQFlags(void);

void nrf_flushRX(void);
void nrf_flushTX(void);

void nrf_reuseTX(void);

// TODO: isSending may not be required, check for TX_DS and MAX_RT pins (checking for TXFifoEmpty may be forbidden during sending...)
uint8_t nrf_isSending(void);

void nrf_startListening(void);
void nrf_stopListening(void); // needs todo

uint8_t nrf_RXFifoFull(void);
uint8_t nrf_RXFifoEmpty(void);
uint8_t nrf_TXFifoFull(void);
uint8_t nrf_TXFifoEmpty(void);
uint8_t nrf_dataAvailable(void);

uint8_t nrf_getRXPipeNumber(void);

// nrf setup methods
void nrf_setChannel(uint8_t channel);
uint8_t nrf_getChannel(void);

void nrf_setAddressWidth(uint8_t width);
uint8_t nrf_getAddressWidth(void);

void nrf_setCRCLength(uint8_t length);
uint8_t nrf_getCRCLength(void);
void nrf_disableCRC(void);

// open the TX pipe, pipe 0 RX address is set to TX address if enAutoAck is set to true, Ack payload and dynamic length are deactivated
void nrf_openTXPipe(const uint8_t* address, uint8_t numBytes, uint8_t enAutoAck, uint8_t enDynAck);
void nrf_openDynamicTXPipe(const uint8_t* address, uint8_t enAckPayload, uint8_t enDynAck);

void nrf_openRXPipe(uint8_t pipe, const uint8_t* address, uint8_t numBytes, uint8_t enAutoAck, uint8_t enDynAck);
void nrf_openDynamicRXPipe(uint8_t pipe, const uint8_t* address, uint8_t enAckPayload, uint8_t enDynAck);
void nrf_closeRXPipe(uint8_t pipe);

void nrf_setTXAddress(const uint8_t* address, uint8_t addrLen);
// for pipes 2 - 5, only LSB is used, set base address in pipe 1
void nrf_setRXAddress(uint8_t pipe, const uint8_t* address, uint8_t addrLen);

void nrf_setRetries(uint16_t delay_us, uint8_t count);
uint8_t nRF_getRetryCount(void);

void nrf_setPayloadLength(uint8_t pipe, uint8_t size);
uint8_t nrf_getPayloadLength(uint8_t pipe);
uint8_t nrf_getDynamicPayloadLength();

void nrf_setDataRate(uint8_t rate);
uint8_t nrf_getDataRate(void);

void nrf_setRFOutPower(uint8_t power);
uint8_t nrf_getRFOutPower(void);

void nrf_enableAutoAck(uint8_t pipe);
void nrf_disableAutoAck(uint8_t pipe);

void nrf_enableRXAddress(uint8_t pipe);
void nrf_disableRXAddress(uint8_t pipe);

// automatically enable dynamic payload lengths (in general) and auto ack for this pipe
void nrf_enableDynamicPayloadLength(uint8_t pipe);
void nrf_disableDynamicPayloadLength(uint8_t pipe);
uint8_t nrf_hasDynamicPayloadLength(uint8_t pipe);

void nrf_enableDynamicPayloadLengths();
void nrf_disableDynamicPayloadLengths();
uint8_t nrf_hasDynamicPayloadLengths(void);

void nrf_enableAckPayload();
void nrf_disableAckPayload();

// enables the W_TX_PAYLOAD_NOACK command
void nrf_enableDynamicAck();
void nrf_disableDynamicAck();
uint8_t nrf_hasDynamicAck();


uint8_t SPI_Read_Byte(uint8_t reg);
void SPI_Write_Byte(uint8_t reg, uint8_t data);
void SPI_Write_Bytes(uint8_t reg, uint8_t* data, uint8_t len);
void writePayload(uint8_t* data, uint8_t len);


#endif /* NRF24L01P_H_ */