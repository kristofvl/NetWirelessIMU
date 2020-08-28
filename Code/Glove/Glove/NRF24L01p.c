/*
 * NRF24L01p.c
 *
 * Created: 26.03.2020 18:21:14
 *  Author: Jochen
 */ 

#include <avr/io.h>

#include "config.h"
#include <util/delay.h>
#include "nrf.h"
#include "NRF24L01p.h"
#include "SPI.h"


	

void nrf_init(uint8_t channel, uint8_t dataRate, uint8_t addressWidth, uint8_t CRCLength)
{
	nrf_powerUp();
	nrf_setChannel(channel);
	nrf_setDataRate(dataRate);
	nrf_setRFOutPower(3);
	nrf_setAddressWidth(addressWidth);
	nrf_setCRCLength(CRCLength);
	nrf_setRetries(500, 15);
}

void nrf_powerUp()
{
	if (!nrf_isPoweredUp())
	{
		// CE low - Standby-I
		PORTB &= ~_BV(CE);
		SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PWR_UP));
		// 1.5 ms power up delay w/o external clock, 150 us else.
		// Just to be sure: wait 2 ms (will only be executed at the beginning or very sparsely anyway)
		_delay_ms(2);
	}
}

void nrf_powerDown()
{
	// CE low - Standby-I
	PORTB &= ~_BV(CE);
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PWR_UP));
}

uint8_t nrf_isPoweredUp()
{
	return SPI_Read_Byte(CONFIG) & (1 << PWR_UP);
}


void nrf_setModeRX(void)
{
	// CE low - Standby-I
	PORTB &= ~_BV(CE);
	
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PRIM_RX));
	
	// nrf_flushTX();
	// nrf_flushRX();
	
	// Reset IRQ status
	nrf_resetIRQFlags();
	
	// Mask TX_DR and MAX_RT interrupts
	// SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	
	// PORTB |= _BV(CE);                             //CE high
	// _delay_us(150);
}

void nrf_setModeTX(void)
{
	// CE low - Standby-I
	PORTB &= ~_BV(CE);
	
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PRIM_RX));
	
	// nrf_flushTX();                     //Flush TX FIFO
	// nrf_flushRX();
	
	nrf_resetIRQFlags();
	
	// Mask TX_DR and MAX_RT interrupts
	// SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	
	// _delay_us(150);
}

void nrf_writeTXData(uint8_t* data, uint8_t len)
{
	nrf_flushTX();
	nrf_resetIRQFlags();
	
	//Transmit payload with ACK enabled
	SPI_Write_Bytes(W_TX_PAYLOAD, data, len);
	
	// TODO: do not send data directly, call this outside the writeData method
	nrf_startSending();
	
	while (nrf_isSending());
}

void nrf_writeTXDataNoAck(uint8_t* data, uint8_t len)
{
	nrf_flushTX();
	nrf_resetIRQFlags();
	
	//Transmit payload with ACK disabled
	SPI_Write_Bytes(W_TX_PAYLOAD_NOACK, data, len);
	
	// TODO: do not send data directly, call this outside the writeData method
	nrf_startSending();
	
	while (nrf_isSending());
}

void nrf_writeAckData(uint8_t pipe, uint8_t* data, uint8_t len)
{
	// flush needed? (I guess not, should be done outside...)
	// nrf_flushTX();
	
	if (pipe > 5)
	{
		return;
	}
	
	SPI_Write_Bytes(W_ACK_PAYLOAD + pipe, data, len);
}

void nrf_startSending(void)
{
	_delay_us(10);        //Need at least 10us before sending
	PORTB |= _BV(CE);             //CE high
	_delay_us(11);        //Hold CE high for at least 10us and not longer than 4ms
	PORTB &= ~_BV(CE);             //CE low
}

void nrf_readRXData(uint8_t* data, uint8_t* len, uint8_t* pipe)
{
	if (nrf_RXFifoEmpty())
	{
		*len = 0;
		*pipe = 0;
		return;
	}
	
	*pipe = nrf_getRXPipeNumber();
	
	if (nrf_hasDynamicPayloadLength(*pipe))
	{
		*len = nrf_getDynamicPayloadLength();
	}
	else
	{
		*len = nrf_getPayloadLength(*pipe);
	}
	
	//Pull down chip select
	PORTB &= ~_BV(CSN);
	
	uint8_t length = *len;
	//Send command to read RX payload
	SPI_Write(R_RX_PAYLOAD);
	
	for (uint8_t i = 0; i < length; ++i)
	{
		data[i] = SPI_Write(0x00);
	}
	
	PORTB |= _BV(CSN);
	_delay_us(1);
}

uint8_t nrf_getStatus(void)
{
	uint8_t status;
	PORTB &= ~_BV(CSN);            //CSN low
	status = SPI_Write(NOP);
	PORTB |= _BV(CSN);            //CSN high
	_delay_us(1);
	return status;
}

void nrf_maskIRQ(uint8_t rx_ready, uint8_t tx_done, uint8_t tx_maxRetry)
{
	uint8_t config = SPI_Read_Byte(CONFIG);
	
	// clear the interrupt flags
	config &= ~(1 << MASK_RX_DR | 1 << MASK_TX_DS | 1 << MASK_MAX_RT);
	// set the specified interrupt flags
	config |= ((rx_ready ? 1:0) << MASK_RX_DR) | ((tx_done ? 1:0) << MASK_TX_DS) | ((tx_maxRetry ? 1:0) << MASK_MAX_RT);
	SPI_Write_Byte(CONFIG, config);
}

uint8_t nrf_getIRQStatus(uint8_t* rx_ready, uint8_t* tx_done, uint8_t* tx_maxRetry)
{
	uint8_t status = nrf_getStatus();

	// Report to the user what happened
	*rx_ready = status & (1 << RX_DR);
	*tx_done = status & (1 << TX_DS);
	*tx_maxRetry = status & (1 << MAX_RT);
	return status & 0x70;
}

void nrf_resetIRQFlags(void)
{
	//Reset IRQ-flags in status register
	SPI_Write_Byte(STATUS, 0x70);
}

void nrf_flushRX(void)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);            //CSN low
	//_delay_us(10);
	SPI_Write(FLUSH_RX);
	//_delay_us(10);
	PORTB |= _BV(CSN);            //CSN high
	_delay_us(1);
}

void nrf_flushTX(void)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);            //CSN low
	//_delay_us(10);
	SPI_Write(FLUSH_TX);
	//_delay_us(10);
	PORTB |= _BV(CSN);            //CSN high
	_delay_us(1);
}

void nrf_reuseTX(void)
{
	PORTB &= ~_BV(CSN);
	SPI_Write(REUSE_TX_PL);
	PORTB |= _BV(CSN);
	_delay_us(1);
}

uint8_t nrf_isSending(void)
{
	//Read the current status
	uint8_t status = nrf_getStatus();

	// If sending successful (TX_DS) or max retries exceeded (MAX_RT)
	if (status & ((1 << TX_DS) | (1 << MAX_RT)))// || TXFifoEmpty())
	{
		return 0;       // False
	}
	return 1;           // True
}

void nrf_startListening(void)
{
	PORTB |= _BV(CE);
	// ce high to csn low requires minimum 4 us
	_delay_us(5);
}

void nrf_stopListening(void)
{
	PORTB &= ~_BV(CE);
}

uint8_t nrf_RXFifoFull()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << RX_FULL);
}

uint8_t nrf_RXFifoEmpty()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << RX_EMPTY);
}

uint8_t nrf_TXFifoFull()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << FIFO_FULL);
}

uint8_t nrf_TXFifoEmpty()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << TX_EMPTY);
}

uint8_t nrf_dataAvailable(void)
{
	return !(SPI_Read_Byte(FIFO_STATUS) & (1 << RX_EMPTY));
}

uint8_t nrf_getRXPipeNumber(void)
{
	return (nrf_getStatus() >> RX_P_NO) & 0x07;
}


// nrf setup methods

void nrf_setChannel(uint8_t channel)
{
	// max channel is 125
	if (channel > 125)
	{
		channel = 125;
	}
	SPI_Write_Byte(RF_CH, channel);
}

uint8_t nrf_getChannel()
{
	return SPI_Read_Byte(RF_CH);
}

void nrf_setAddressWidth(uint8_t width)
{
	if (width < 3)
	{
		width = 3;
	}
	else if (width > 5)
	{
		width = 5;
	}
	SPI_Write_Byte(SETUP_AW, (width - 2) & 0x03);
}

uint8_t nrf_getAddressWidth(void)
{
	return SPI_Read_Byte(SETUP_AW) + 2;
}

void nrf_setCRCLength(uint8_t length)
{
	uint8_t config = SPI_Read_Byte(CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));

	if (length == 0)
	{
		// Do nothing, we turned it off above.
	}
	else if (length == 1)
	{
		config |= (1 << EN_CRC);
	}
	else
	{
		config |= (1 << EN_CRC);
		config |= (1 << CRCO);
	}
	SPI_Write_Byte(CONFIG, config);
}

uint8_t nrf_getCRCLength(void)
{
	uint8_t result = 0;
	
	uint8_t config = SPI_Read_Byte(CONFIG);

	if (config & (1 << EN_CRC) || SPI_Read_Byte(EN_AA))
	{
		if (config & (1 << CRCO))
		{
			result = 2;
		}
		else
		{
			result = 1;
		}
	}

	return result;
}

void nrf_disableCRC(void)
{
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << EN_CRC));
}


void nrf_openTXPipe(const uint8_t* address, uint8_t numBytes, uint8_t enAutoAck, uint8_t enDynAck)
{
	nrf_setTXAddress(address, 5);
	
	if (enAutoAck)
	{
		nrf_openRXPipe(0, address, numBytes, enAutoAck, enDynAck);
	}
}

void nrf_openDynamicTXPipe(const uint8_t* address, uint8_t enAckPayload, uint8_t enDynAck)
{
	nrf_setTXAddress(address, 5);
	nrf_openDynamicRXPipe(0, address, enAckPayload, enDynAck);
}


void nrf_openRXPipe(uint8_t pipe, const uint8_t* address, uint8_t numBytes, uint8_t enAutoAck, uint8_t enDynAck)
{
	nrf_setRXAddress(pipe, address, 5);
	nrf_enableRXAddress(pipe);
	nrf_disableDynamicPayloadLength(pipe);
	nrf_setPayloadLength(pipe, numBytes);
	
	if (enAutoAck)
	{
		nrf_enableAutoAck(pipe);
		if (enDynAck)
		{
			nrf_enableDynamicAck();
		}
	}
	else
	{
		nrf_disableAutoAck(pipe);
	}
}

void nrf_openDynamicRXPipe(uint8_t pipe, const uint8_t* address, uint8_t enAckPayload, uint8_t enDynAck)
{
	nrf_setRXAddress(pipe, address, 5);
	nrf_enableRXAddress(pipe);
	// just to be sure: set num bytes to be received to 32 (0 means pipe not used according to data sheet)
	nrf_setPayloadLength(pipe, 32);
	nrf_enableDynamicPayloadLength(pipe);
	
	if (enAckPayload)
	{
		nrf_enableAckPayload();
	}
	else
	{
		nrf_disableAckPayload();
	}
	
	if (enDynAck)
	{
		nrf_enableDynamicAck();
	}
}

void nrf_closeRXPipe(uint8_t pipe)
{
	nrf_disableRXAddress(pipe);
}



void nrf_setTXAddress(const uint8_t* address, uint8_t addrLen)
{
	uint8_t i;
	PORTB &= ~_BV(CSN);
	
	//Setup p0 pipe address for receiving
	SPI_Write(W_REGISTER + TX_ADDR);
	
	// write address, LSB first
	for (i = addrLen; i > 0; --i)
	{
		SPI_Write(address[i-1]);
	}
	PORTB |= _BV(CSN);
	_delay_us(1);
}

void nrf_setRXAddress(uint8_t pipe, const uint8_t* address, uint8_t addrLen)
{
	// only pipes 0 and 1 are allowed full addresses, other pipes only have different LSB address
	if (pipe < 2)
	{
		uint8_t i;
		PORTB &= ~_BV(CSN);
		
		//Setup pipe address for receiving
		SPI_Write(W_REGISTER + RX_ADDR_P0 + pipe);
		
		// write address, LSB first
		for (i = addrLen; i > 0; --i)
		{
			SPI_Write(address[i-1]);
		}
		PORTB |= _BV(CSN);
		_delay_us(1);
	}
	else if (pipe < 6)
	{
		// Setup pipe address for receiving, only write LSB
		SPI_Write_Byte(RX_ADDR_P0 + pipe, address[addrLen-1]);
	}
}


void nrf_setRetries(uint16_t delay_us, uint8_t count)
{
	if (delay_us < 250)
	{
		delay_us = 250;
	}
	else if (delay_us > 4000)
	{
		delay_us = 4000;
	}
	if (count > 15)
	{
		count = 15;
	}
	
	uint8_t delay = (delay_us / 250) - 1;
	SPI_Write_Byte(SETUP_RETR, (delay << 4) | count);
}

uint8_t nRF_getRetryCount(void)
{
	return SPI_Read_Byte(OBSERVE_TX) & 0x0F;
}

void nrf_setPayloadLength(uint8_t pipe, uint8_t size)
{
	if (pipe > 5)
	{
		return;
	}
	if (size > 32)
	{
		size = 32;
	}
	SPI_Write_Byte(RX_PW_P0 + pipe, size);
}

uint8_t nrf_getPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return 0;
	}
	return SPI_Read_Byte(RX_PW_P0 + pipe);
}

uint8_t nrf_getDynamicPayloadLength()
{
	return SPI_Read_Byte(R_RX_PL_WID);
}

void nrf_setDataRate(uint8_t rate)
{
	// read RF setup and mask out data rate bits
	uint8_t dataRate = SPI_Read_Byte(RF_SETUP) & 0xD7;
	if (rate == 0)
	{
		// 250 kBit/s
		dataRate |= 0x20;
	}
	else if (rate == 1)
	{
		// 1 MBit/s
		dataRate |= 0;
	}
	else
	{
		// 2 MBit/s
		dataRate |= 0x08;
	}
	
	SPI_Write_Byte(RF_SETUP, dataRate);
}

uint8_t nrf_getDataRate(void)
{
	// read RF setup and mask out non data rate bits
	uint8_t dataRate = SPI_Read_Byte(RF_SETUP) & 0x28;
	if (dataRate == 0x20)
	{
		// 250 kBit/s
		return 0;
	}
	else if (dataRate == 0)
	{
		// 1 MBit/s
		return 1;
	}
	else // if (dataRate == 0x08)
	{
		// 2 MBit/s
		return 2;
	}
}

void nrf_setRFOutPower(uint8_t power)
{
	if (power > 3)
	{
		power = 3;
	}
	SPI_Write_Byte(RF_SETUP, (SPI_Read_Byte(RF_SETUP) & ~0x06) | (power << RF_PWR));
}

uint8_t nrf_getRFOutPower(void)
{
	return (SPI_Read_Byte(RF_SETUP) & 0x06) >> RF_PWR;
}

void nrf_enableAutoAck(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_AA, SPI_Read_Byte(EN_AA) | (1 << pipe));
}

void nrf_disableAutoAck(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_AA, SPI_Read_Byte(EN_AA) & ~(1 << pipe));
}


void nrf_enableRXAddress(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_RXADDR, SPI_Read_Byte(EN_RXADDR) | (1 << pipe));
}

void nrf_disableRXAddress(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_RXADDR, SPI_Read_Byte(EN_RXADDR) & ~(1 << pipe));
}


void nrf_enableDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	// dynamic payload lengths and auto ack has to be enabled
	nrf_enableDynamicPayloadLengths();
	nrf_enableAutoAck(pipe);
	
	SPI_Write_Byte(DYNPD, SPI_Read_Byte(DYNPD) | (1 << pipe));
}

void nrf_disableDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(DYNPD, SPI_Read_Byte(DYNPD) & ~(1 << pipe));
}

uint8_t nrf_hasDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return 0;
	}
	return SPI_Read_Byte(DYNPD) & (1 << pipe);
}

void nrf_enableDynamicPayloadLengths()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_DPL));
}

void nrf_disableDynamicPayloadLengths()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_DPL));

	// Disable dynamic payload on all pipes
	SPI_Write_Byte(DYNPD, 0);
}

uint8_t nrf_hasDynamicPayloadLengths(void)
{
	return SPI_Read_Byte(FEATURE) & (1 << EN_DPL);
}


void nrf_enableAckPayload()
{
	// dynamic payload length has to be enabled
	nrf_enableDynamicPayloadLengths();
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_ACK_PAY));
}

void nrf_disableAckPayload()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_ACK_PAY));
}


void nrf_enableDynamicAck()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_DYN_ACK));
}

void nrf_disableDynamicAck()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_DYN_ACK));
}

uint8_t nrf_hasDynamicAck()
{
	return SPI_Read_Byte(FEATURE) & (1 << EN_DYN_ACK);
}



uint8_t SPI_Read_Byte(uint8_t reg)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	//_delay_us(10);
	SPI_Write(R_REGISTER + reg);
	//_delay_us(10);
	reg = SPI_Write(NOP);
	//_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(1);
	return reg;
}

void SPI_Write_Byte(uint8_t reg, uint8_t data)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	//_delay_us(10);
	SPI_Write(W_REGISTER + reg);
	//_delay_us(10);
	SPI_Write(data);
	//_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(1);
}

void SPI_Write_Bytes(uint8_t reg, uint8_t* data, uint8_t len)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	//_delay_us(10);
	SPI_Write(reg);
	//_delay_us(10);
	writePayload(data, len);
	//_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(1);
}


void writePayload(uint8_t* data, uint8_t len)
{
	for (uint8_t i = 0; i < len; ++i)
	{
		SPI_Write(data[i]);
	}
}