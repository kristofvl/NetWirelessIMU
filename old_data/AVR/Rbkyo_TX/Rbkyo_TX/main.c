/*
 * TX.c
 *
 * Created: 1/6/2018 5:41:48 AM
 * Author : rbkyo
 */ 
#include <avr/io.h>
#define F_CPU 16000000
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "nrf.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

#define PAYLOAD_LEN	32

uint8_t payload[32] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		       0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
		       0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24,
		       0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32};

uint8_t nrf24_isSending(void);
uint8_t nrf24_getStatus(void);

void Init_SPI()
{
	//Set the output pin(s) for SPI
	DDRB |= _BV(CE);	//CE
	DDRB |= _BV(CSN);	//CSN
	DDRB |= _BV(MOSI);  	//MOSI
	DDRB |= _BV(SCLK);  	//SCLK

	//Set the input pin(s) for SPI
	DDRB &= ~_BV(MISO); 	//MISO

	
	SPCR |= ((1 << SPE) | (1 << MSTR) | (1 << SPR0));	//Enable SPI as master
	SPCR &= (~_BV(SPI2X) & ~_BV(SPR1)); 		   	//Set clock rate but not too important
	
	PORTB |= _BV(CSN);	//CSN high
	PORTB &= ~_BV(CE);	//CE low
}

unsigned char spi_tranceiver(unsigned char data)
{
	// Load data into the buffer
	SPDR = data;
	
	//Wait until transmission complete
	while(!(SPSR & (1 << SPIF)));   

	//Return received data
	return(SPDR);
}

unsigned char Read_Byte(unsigned char reg)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	spi_tranceiver(R_REGISTER + reg);
	_delay_us(10);
	reg = spi_tranceiver(NOP);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	return reg;
}

void Write_byte(unsigned char reg, unsigned char data)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	spi_tranceiver(W_REGISTER + reg);
	_delay_us(10);
	spi_tranceiver(data);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
}

void Init_nrf(void)
{
	_delay_ms(100);
	
	//Enable auto-acknowledgment for data pipe 0
	Write_byte(EN_AA, 0x01); 
	
	//Enable data pipe 0
	Write_byte(EN_RXADDR, 0x01); 

	//Set address width to 5 bytes
    	Write_byte(SETUP_AW, 0x03);  
	
	//Set channel frequency to 2.505GHz
	Write_byte(RF_CH, 0x69);  
	
	//Set data rate to 2Mbps and 0dB gain
	Write_byte(RF_SETUP, 0x0E);
	
	//Enable W_TX_PAYLOAD_NOACK command
//	Write_byte(FEATURE, 0x01);  
	
	//Set the 5-bytes receiver address as 0x01 0x02 0x03 0x04 0x05
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup p0 pipe address for receiving
	spi_tranceiver(W_REGISTER + RX_ADDR_P0); 
	_delay_us(10);
	spi_tranceiver(0x01);
	_delay_us(10);
	spi_tranceiver(0x02);
	_delay_us(10);
	spi_tranceiver(0x03);
	_delay_us(10);
	spi_tranceiver(0x04);
	_delay_us(10);
	spi_tranceiver(0x05);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	
	//Set the 5-bytes transmitter address as 0x01 0x02 0x03 0x04 0x05	
	_delay_us(10);		
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup the transmitter address
	spi_tranceiver(W_REGISTER + TX_ADDR);
	_delay_us(10);
	spi_tranceiver(0xAA);
	_delay_us(10);
	spi_tranceiver(0xBB);
	_delay_us(10);
	spi_tranceiver(0xCC);
	_delay_us(10);
	spi_tranceiver(0xDD);
	_delay_us(10);
	spi_tranceiver(0xEE);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
		
	//Set the payload width as 32-bytes	
	Write_byte(RX_PW_P0, 0x20); 
	
	//Set the retransmission delay to 750us with 15 retries
//	Write_byte(SETUP_RETR, 0x2F); 
	
	//Boot the nrf as TX and mask the maximum retransmission interrupt(disable)
	//Enable CRC and set the length to 2-bytes
	Write_byte(CONFIG, 0x1E);
	
	_delay_ms(100);
}

void Flush_tx(void)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	spi_tranceiver(FLUSH_TX);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(10);
}

void Flush_rx(void)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);
	_delay_us(10);
	spi_tranceiver(FLUSH_RX);
	_delay_us(10);
	PORTB |= _BV(CSN);
	_delay_us(10);
}

void Payload_TX(uint8_t* data, uint8_t len)
{
	uint8_t i;
	
	for(i = 0; i < len; i++)
	{
		spi_tranceiver(payload[i]);
	}

}

void transmit_data(unsigned char *tdata)
{
	Flush_tx();
	PORTB &= ~_BV(CSN); //CSN low
	_delay_us(10);
	//Transmit payload with ACK enabled
	spi_tranceiver(W_TX_PAYLOAD);
	_delay_us(10);
	Payload_TX(payload, PAYLOAD_LEN);
	_delay_us(10);
	PORTB |= _BV(CSN);  //CSN high
	_delay_us(15);      //Need at least 10us before sending
	PORTB |= _BV(CE);   //CE high
	_delay_us(20);      //Hold CE high for at least 10us and not longer than 4ms
//	PORTB &= ~_BV(CE);  //CE low
//	_delay_ms(1);       //Delay needed for retransmissions before reset
}

uint8_t nrf24_isSending()
{
	uint8_t status;

	/* read the current status */
	status = nrf24_getStatus();
	
	/* if sending successful (TX_DS) or max retries exceeded (MAX_RT). */
	if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
	{
		return 0; /* false */
	}

	return 1; /* true */

}

uint8_t nrf24_getStatus()
{
	uint8_t rv;
	PORTB &= ~_BV(CSN); //CSN low
	rv = spi_tranceiver(NOP);
	PORTB |= _BV(CSN);  //CSN high
	return rv;
}

void reset(void)
{
	_delay_us(10);
	//Reset IRQ-flags in status register
    	Write_byte(STATUS, 0x70);   
	_delay_us(10);
}

//Main function for TX
int main(void)
{
	//Initialize peripherals
	Init_SPI();

	//Initialize devices
	Init_nrf();
	
    	while(1) 
    	{
		transmit_data(payload);
		while(nrf24_isSending());
		reset();
    	}
}