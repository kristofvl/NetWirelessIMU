#include "Nrf24l01p.h"



Nrf24l01p::Nrf24l01p(PinName mosi, PinName miso, PinName sck, PinName ncsn, PinName nce)
					: spi(mosi, miso, sck), csn(ncsn), ce(nce)
{
    SPI_Init();
}

Nrf24l01p::~Nrf24l01p()
{
	
}



void Nrf24l01p::init(uint8_t channel, uint8_t dataRate, uint8_t addressWidth, uint8_t CRCLength)
{
	powerUp();
	setChannel(channel);
	setDataRate(dataRate);
	setRFOutPower(3);
	setAddressWidth(addressWidth);
	setCRCLength(CRCLength);
	setRetries(500, 15);
}

void Nrf24l01p::powerUp()
{
	if (!isPoweredUp())
	{
		// CE low - Standby-I
		ce = 0;
		SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PWR_UP));
		// 1.5 ms power up delay w/o external clock, 150 us else.
		// Just to be sure: wait 2 ms (will only be executed at the beginning or very sparsely anyway)
		wait_ms(2);
	}
}

void Nrf24l01p::powerDown()
{
	// CE low - Standby-I
	ce = 0;
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PWR_UP));
}

bool Nrf24l01p::isPoweredUp()
{
	return SPI_Read_Byte(CONFIG) & (1 << PWR_UP);
}


void Nrf24l01p::setModeRX(void)
{
    // CE low - Standby-I
    ce = 0;
    
    SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PRIM_RX));
    
    // flushTX();
    // flushRX();
    
    // Reset IRQ status
    resetIRQFlags();
    
    // Mask TX_DR and MAX_RT interrupts
    // SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
    
    // ce = 1;                             //CE high
    // wait_us(150);
}

void Nrf24l01p::setModeTX(void)
{
    // CE low - Standby-I
    ce = 0;
    
    SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PRIM_RX));
    
    // flushTX();                     //Flush TX FIFO
    // flushRX();
    
    resetIRQFlags();
    
    // Mask TX_DR and MAX_RT interrupts
    // SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
    
    // wait_us(150);
}

void Nrf24l01p::writeTXData(uint8_t* data, uint8_t len, bool getAck)
{
    flushTX();
    resetIRQFlags();
    
    if (getAck)
    {
	    //Transmit payload with ACK enabled
	    SPI_Write_Bytes(W_TX_PAYLOAD, data, len);
	}
	else
	{
		SPI_Write_Bytes(W_TX_PAYLOAD_NOACK, data, len);
	}
    
    // TODO: do not send data directly, call this outside the writeData method
    startSending();
    
    while (isSending());
}

void Nrf24l01p::writeAckData(uint8_t pipe, uint8_t* data, uint8_t len)
{
    // flush needed? (I guess not, should be done outside...)
    flushTX();
    
    if (pipe > 5)
    {
    	return;
    }
    
    SPI_Write_Bytes(W_ACK_PAYLOAD + pipe, data, len);
}

void Nrf24l01p::startSending(void)
{
    wait_us(10);        //Need at least 10us before sending
	ce = 1;             //CE high
    wait_us(12);        //Hold CE high for at least 10us and not longer than 4ms
    ce = 0;             //CE low
}

void Nrf24l01p::readRXData(uint8_t* data, uint8_t &len, uint8_t &pipe)
{
    if (RXFifoEmpty())
    {
    	len = 0;
    	pipe = 0;
    	return;
    }
    
    pipe = getRXPipeNumber();
    
    if (hasDynamicPayloadLength(pipe))
    {
    	len = getDynamicPayloadLength();
	}
	else
	{
		len = getPayloadLength(pipe);
	}
    
    //Pull down chip select
    csn = 0;
    
    //Send command to read RX payload
    spi.write(R_RX_PAYLOAD);
    
    for (uint8_t i = 0; i < len; ++i)
    {
    	data[i] = spi.write(0x00);
    }
    
    csn = 1;
    wait_us(1);
}

uint8_t Nrf24l01p::getStatus(void)
{
    uint8_t status;
    csn = 0;            //CSN low
    status = spi.write(NOP);
    csn = 1;            //CSN high
    wait_us(1);
    return status;
}

void Nrf24l01p::maskIRQ(bool rx_ready, bool tx_done, bool tx_maxRetry)
{
	uint8_t config = SPI_Read_Byte(CONFIG);
    
    // clear the interrupt flags
    config &= ~(1 << MASK_RX_DR | 1 << MASK_TX_DS | 1 << MASK_MAX_RT);
    // set the specified interrupt flags
    config |= ((rx_ready ? 1:0) << MASK_RX_DR) | ((tx_done ? 1:0) << MASK_TX_DS) | ((tx_maxRetry ? 1:0) << MASK_MAX_RT);
    SPI_Write_Byte(CONFIG, config);
}

bool Nrf24l01p::getIRQStatus(bool &rx_ready, bool &tx_done, bool &tx_maxRetry)
{
	uint8_t status = getStatus();

    // Report to the user what happened
    rx_ready = status & (1 << RX_DR);
    tx_done = status & (1 << TX_DS);
    tx_maxRetry = status & (1 << MAX_RT);
    return status & 0x70;
}

void Nrf24l01p::resetIRQFlags(void)
{
    //Reset IRQ-flags in status register
    SPI_Write_Byte(STATUS, 0x70);
}

void Nrf24l01p::flushRX(void)
{
    //wait_us(10);
    csn = 0;            //CSN low
    //wait_us(10);
    spi.write(FLUSH_RX);
    //wait_us(10);
    csn = 1;            //CSN high
    wait_us(1);
}

void Nrf24l01p::flushTX(void)
{
    //wait_us(10);
    csn = 0;            //CSN low
    //wait_us(10);
    spi.write(FLUSH_TX);
    //wait_us(10);
    csn = 1;            //CSN high
    wait_us(1);
}

void Nrf24l01p::reuseTX(void)
{
	csn = 0;
	spi.write(REUSE_TX_PL);
	csn = 1;
    wait_us(1);
}

uint8_t Nrf24l01p::isSending(void)
{
    //Read the current status
    uint8_t status = getStatus();

    // If sending successful (TX_DS) or max retries exceeded (MAX_RT)
    if (status & ((1 << TX_DS) | (1 << MAX_RT)))// || TXFifoEmpty())
    {
        return 0;       //False
    }
    return 1;           //True
}

void Nrf24l01p::startListening(void)
{
	ce = 1;
	// ce high to csn low requires minimum 4 us
	wait_us(5);
}

void Nrf24l01p::stopListening(void)
{
	ce = 0;
}

bool Nrf24l01p::RXFifoFull()
{
    return SPI_Read_Byte(FIFO_STATUS) & (1 << RX_FULL);
}

bool Nrf24l01p::RXFifoEmpty()
{
    return SPI_Read_Byte(FIFO_STATUS) & (1 << RX_EMPTY);
}

bool Nrf24l01p::TXFifoFull()
{
    return SPI_Read_Byte(FIFO_STATUS) & (1 << FIFO_FULL);
}

bool Nrf24l01p::TXFifoEmpty()
{
    return SPI_Read_Byte(FIFO_STATUS) & (1 << TX_EMPTY);
}

bool Nrf24l01p::dataAvailable(void)
{
	return !(SPI_Read_Byte(FIFO_STATUS) & (1 << RX_EMPTY));
}

uint8_t Nrf24l01p::getRXPipeNumber(void)
{
    return (getStatus() >> RX_P_NO) & 0x07;
}


// nrf setup methods

void Nrf24l01p::setChannel(uint8_t channel)
{
    // max channel is 125
    if (channel > 125)
    {
    	channel = 125;
    }
    SPI_Write_Byte(RF_CH, channel);
}

uint8_t Nrf24l01p::getChannel()
{
    return SPI_Read_Byte(RF_CH);
}

void Nrf24l01p::setAddressWidth(uint8_t width)
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

uint8_t Nrf24l01p::getAddressWidth(void)
{
	return SPI_Read_Byte(SETUP_AW) + 2;
}

void Nrf24l01p::setCRCLength(uint8_t length)
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

uint8_t Nrf24l01p::getCRCLength(void)
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

void Nrf24l01p::disableCRC(void)
{
    SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << EN_CRC));
}


void Nrf24l01p::openTXPipe(const uint8_t* address, uint8_t numBytes, bool enAutoAck, bool enDynAck)
{
	setTXAddress(address, 5);
	
	if (enAutoAck)
	{
		openRXPipe(0, address, numBytes, enAutoAck, enDynAck);
	}
}

void Nrf24l01p::openDynamicTXPipe(const uint8_t* address, bool enAckPayload, bool enDynAck)
{
	setTXAddress(address, 5);
	openDynamicRXPipe(0, address, enAckPayload, enDynAck);
}


void Nrf24l01p::openRXPipe(uint8_t pipe, const uint8_t* address, uint8_t numBytes, bool enAutoAck, bool enDynAck)
{
	setRXAddress(pipe, address, 5);
	enableRXAddress(pipe);
	disableDynamicPayloadLength(pipe);
	setPayloadLength(pipe, numBytes);
	
	if (enAutoAck)
	{
		enableAutoAck(pipe);
		if (enDynAck)
		{
			enableDynamicAck();
		}
	}
	else
	{
		disableAutoAck(pipe);
	}
}

void Nrf24l01p::openDynamicRXPipe(uint8_t pipe, const uint8_t* address, bool enAckPayload, bool enDynAck)
{
	setRXAddress(pipe, address, 5);
	enableRXAddress(pipe);
	// just to be sure: set num bytes to be received to 32 (0 means pipe not used according to data sheet)
	setPayloadLength(pipe, 32);
	enableDynamicPayloadLength(pipe);
	
	if (enAckPayload)
	{
		enableAckPayload();
	}
	else
	{
		disableAckPayload();
	}
	
	if (enDynAck)
	{
		enableDynamicAck();
	}
}

void Nrf24l01p::closeRXPipe(uint8_t pipe)
{
	disableRXAddress(pipe);
}

    

void Nrf24l01p::setTXAddress(const uint8_t* address, uint8_t addrLen)
{
    uint8_t i;
    csn = 0;
    
    //Setup p0 pipe address for receiving
    spi.write(W_REGISTER + TX_ADDR);
    
    // write address, LSB first
    for (i = addrLen; i > 0; --i)
    {
        spi.write(address[i-1]);
    }
    csn = 1;
    wait_us(1);
}

void Nrf24l01p::setRXAddress(uint8_t pipe, const uint8_t* address, uint8_t addrLen)
{
    // only pipes 0 and 1 are allowed full addresses, other pipes only have different LSB address
    if (pipe < 2)
    {
	    uint8_t i;
	    csn = 0;
	    
	    //Setup pipe address for receiving
	    spi.write(W_REGISTER + RX_ADDR_P0 + pipe);
	    
	    // write address, LSB first
	    for (i = addrLen; i > 0; --i)
	    {
	        spi.write(address[i-1]);
	    }
	    csn = 1;
    	wait_us(1);
    }
    else if (pipe < 6)
    {
	    // Setup pipe address for receiving, only write LSB
	    SPI_Write_Byte(RX_ADDR_P0 + pipe, address[addrLen-1]);
    }
}


void Nrf24l01p::setRetries(uint16_t delay_us, uint8_t count)
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

uint8_t Nrf24l01p::getRetryCount(void)
{
	return SPI_Read_Byte(OBSERVE_TX) & 0x0F;
}

void Nrf24l01p::setPayloadLength(uint8_t pipe, uint8_t size)
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

uint8_t Nrf24l01p::getPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return 0;
	}
	return SPI_Read_Byte(RX_PW_P0 + pipe);
}

uint8_t Nrf24l01p::getDynamicPayloadLength()
{
	return SPI_Read_Byte(R_RX_PL_WID);
}

void Nrf24l01p::setDataRate(uint8_t rate)
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

uint8_t Nrf24l01p::getDataRate(void)
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

void Nrf24l01p::setRFOutPower(uint8_t power)
{
	if (power > 3)
	{
		power = 3;
	}
	SPI_Write_Byte(RF_SETUP, (SPI_Read_Byte(RF_SETUP) & ~0x06) | (power << RF_PWR));
}

uint8_t Nrf24l01p::getRFOutPower(void)
{
	return (SPI_Read_Byte(RF_SETUP) & 0x06) >> RF_PWR;
}

void Nrf24l01p::enableAutoAck(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_AA, SPI_Read_Byte(EN_AA) | (1 << pipe));
}

void Nrf24l01p::disableAutoAck(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_AA, SPI_Read_Byte(EN_AA) & ~(1 << pipe));
}


void Nrf24l01p::enableRXAddress(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_RXADDR, SPI_Read_Byte(EN_RXADDR) | (1 << pipe));
}

void Nrf24l01p::disableRXAddress(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_RXADDR, SPI_Read_Byte(EN_RXADDR) & ~(1 << pipe));
}


void Nrf24l01p::enableDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	// dynamic payload lengths and auto ack has to be enabled
	enableDynamicPayloadLengths();
	enableAutoAck(pipe);
	
	SPI_Write_Byte(DYNPD, SPI_Read_Byte(DYNPD) | (1 << pipe));
}

void Nrf24l01p::disableDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(DYNPD, SPI_Read_Byte(DYNPD) & ~(1 << pipe));
}

bool Nrf24l01p::hasDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return false;
	}
	return SPI_Read_Byte(DYNPD) & (1 << pipe);
}

void Nrf24l01p::enableDynamicPayloadLengths()
{
    SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_DPL));
}

void Nrf24l01p::disableDynamicPayloadLengths()
{
    SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_DPL));

    // Disable dynamic payload on all pipes
    SPI_Write_Byte(DYNPD, 0);
}

bool Nrf24l01p::hasDynamicPayloadLengths(void)
{
	return SPI_Read_Byte(FEATURE) & (1 << EN_DPL);
}


void Nrf24l01p::enableAckPayload()
{
    // dynamic payload length has to be enabled
    enableDynamicPayloadLengths();
    SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_ACK_PAY));
}

void Nrf24l01p::disableAckPayload()
{
    SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_ACK_PAY));
}


void Nrf24l01p::enableDynamicAck()
{
    SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_DYN_ACK));
}

void Nrf24l01p::disableDynamicAck()
{
    SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_DYN_ACK));
}

bool Nrf24l01p::hasDynamicAck()
{
    return SPI_Read_Byte(FEATURE) & (1 << EN_DYN_ACK);
}



/************************************************************************************
** SPI_Init function:
** - 1MHz
** - Mode 0: ClockPhase = 0, ClockPolarity = 0
*************************************************************************************/

void Nrf24l01p::SPI_Init(void)
{
    spi.format(8,0);    //8 bit data, Mode 0
    spi.frequency(1000000);

    csn = 1;            //CSN high
    ce = 0;             //CE low
}

void Nrf24l01p::SPI_Write_Byte(uint8_t reg, uint8_t data)
{
    //wait_us(10);
    csn = 0;            //CSN low
    //wait_us(10);
    spi.write(W_REGISTER + reg);
    //wait_us(10);
    spi.write(data);
    //wait_us(10);
    csn = 1;            //CSN high
    wait_us(1);
}

void Nrf24l01p::SPI_Write_Bytes(uint8_t reg, uint8_t* data, uint8_t len)
{
    //wait_us(10);
    csn = 0;            //CSN low
    //wait_us(10);
    spi.write(reg);
    //wait_us(10);
    writePayload(data, len);
    //wait_us(10);
    csn = 1;            //CSN high
    wait_us(1);
}

uint8_t Nrf24l01p::SPI_Read_Byte(uint8_t reg)
{
    //wait_us(10);
    csn = 0;            //CSN low
    //wait_us(10);
    spi.write(R_REGISTER + reg);
    //wait_us(10);
    reg = spi.write(NOP);
    //wait_us(10);
    csn = 1;            //CSN high
    wait_us(1);
    return reg;
}

void Nrf24l01p::writePayload(uint8_t* data, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
    	spi.write(data[i]);
    }
}


