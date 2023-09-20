#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__


#include "mbed.h"
#include "nrf.h"

class Nrf24l01p  {
public:
    Nrf24l01p(PinName mosi, PinName miso, PinName sck, PinName ncsn, PinName nce);
    ~Nrf24l01p();
    
    void init(uint8_t channel, uint8_t dataRate, uint8_t addressWidth, uint8_t CRCLength);
    
    void powerUp();
    void powerDown();
    bool isPoweredUp();
    
    void setModeRX(void);
    void setModeTX(void);
    
    // Ensure that dynamic Ack is enabled if you set getAck=false.
    // If auto ack is disabled, do net set getAck=false.
    // Writes to the txBuffer and starts sending in one call (TX buffer gets flushed beforehand as well).
    // Waits until transmission done or max retries exceeded.
    void writeTXData(uint8_t* data, uint8_t len, bool getAck = true);
    
    // TODO: maybe implement fast write method (just writing to the TX FIFO without sending directly)
    // bool writeTXDataFast(uint8_t* data, uint8_t len, bool getAck = true);
    
    void writeAckData(uint8_t pipe, uint8_t* data, uint8_t len);
    
    void startSending(void);
    
    void readRXData(uint8_t* data, uint8_t &len, uint8_t &pipe);
    
    uint8_t getStatus(void);
    
    void maskIRQ(bool rx_ready, bool tx_done, bool tx_maxRetry);
    // returns true if any IRQ bit is set
    bool getIRQStatus(bool &rx_ready, bool &tx_done, bool &tx_maxRetry);
    void resetIRQFlags(void);
    
    void flushRX(void);
    void flushTX(void);
    
    void reuseTX(void);
    
    // TODO: isSending may not be required, check for TX_DS and MAX_RT pins (checking for TXFifoEmpty may be forbidden during sending...)
    uint8_t isSending(void);
    
    void startListening(void);
    void stopListening(void); // needs todo
    
    bool RXFifoFull(void);
    bool RXFifoEmpty(void);
    bool TXFifoFull(void);
    bool TXFifoEmpty(void);
    bool dataAvailable(void);
    
    uint8_t getRXPipeNumber(void);
    
    // nrf setup methods
    void setChannel(uint8_t channel);
    uint8_t getChannel(void);
    
    void setAddressWidth(uint8_t width);
    uint8_t getAddressWidth(void);
    
    void setCRCLength(uint8_t length);
    uint8_t getCRCLength(void);
    void disableCRC(void);
    
    // open the TX pipe, pipe 0 RX address is set to TX address if enAutoAck is set to true, Ack payload and dynamic length are deactivated
    void openTXPipe(const uint8_t* address, uint8_t numBytes, bool enAutoAck, bool enDynAck);
    void openDynamicTXPipe(const uint8_t* address, bool enAckPayload, bool enDynAck);
    
    void openRXPipe(uint8_t pipe, const uint8_t* address, uint8_t numBytes, bool enAutoAck, bool enDynAck);
    void openDynamicRXPipe(uint8_t pipe, const uint8_t* address, bool enAckPayload, bool enDynAck);
    void closeRXPipe(uint8_t pipe);
    
    void setTXAddress(const uint8_t* address, uint8_t addrLen);
    // for pipes 2 - 5, only LSB is used, set base address in pipe 1
    void setRXAddress(uint8_t pipe, const uint8_t* address, uint8_t addrLen);

    void setRetries(uint16_t delay_us, uint8_t count);
    uint8_t getRetryCount(void);
    
    void setPayloadLength(uint8_t pipe, uint8_t size);
    uint8_t getPayloadLength(uint8_t pipe);
    uint8_t getDynamicPayloadLength();
    
    void setDataRate(uint8_t rate);
    uint8_t getDataRate(void);
    
    void setRFOutPower(uint8_t power);
    uint8_t getRFOutPower(void);
    
    void enableAutoAck(uint8_t pipe);
    void disableAutoAck(uint8_t pipe);
    
    void enableRXAddress(uint8_t pipe);
    void disableRXAddress(uint8_t pipe);
    
    // automatically enable dynamic payload lengths (in general) and auto ack for this pipe
    void enableDynamicPayloadLength(uint8_t pipe);
    void disableDynamicPayloadLength(uint8_t pipe);
    bool hasDynamicPayloadLength(uint8_t pipe);
    
    void enableDynamicPayloadLengths();
    void disableDynamicPayloadLengths();
    bool hasDynamicPayloadLengths(void);
    
    void enableAckPayload();
    void disableAckPayload();
    
    // enables the W_TX_PAYLOAD_NOACK command
    void enableDynamicAck();
    void disableDynamicAck();
    bool hasDynamicAck();

private:
    SPI         spi;
    DigitalOut  csn;
    DigitalOut  ce;
    
    void SPI_Init(void);
    
    void SPI_Write_Byte(uint8_t reg, uint8_t data);
    void SPI_Write_Bytes(uint8_t reg, uint8_t* data, uint8_t len);
    uint8_t SPI_Read_Byte(uint8_t reg);

    void writePayload(uint8_t* data, uint8_t len);
};

#endif /* __NRF24L01P_H__ */
