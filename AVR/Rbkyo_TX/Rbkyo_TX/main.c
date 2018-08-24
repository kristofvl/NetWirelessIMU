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
#include "nrf.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

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
//	Write_byte(EN_AA, 0x01); 
	
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
		
	//Set the payload width as 1-byte	
	Write_byte(RX_PW_P0, 0x01); 
	
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

void transmit_data(unsigned char tdata)
{
	Flush_tx();
    	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Transmit payload with ACK enabled
	spi_tranceiver(W_TX_PAYLOAD);
	_delay_us(10);
	spi_tranceiver(tdata);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_ms(10);		//Need 10ms before sending
	PORTB |= _BV(CE);	//CE high
	_delay_us(20);  	//Hold CE high for at least 10us and not longer than 2ms
	PORTB &= ~_BV(CE);	//CE low
	_delay_ms(10); 		//Delay needed for retransmissions before reset
}

void reset(void)
{
	_delay_us(10);
	//Reset IRQ-flags in status register
    	Write_byte(STATUS, 0x70);   
	_delay_us(10);
}

void nRF_Put_String(char *s)
{
	//Loop through entire string
	while(*s)
	{
		transmit_data(*s);
		s++;
	}
}

//Main function for TX
int main(void)
{
	//Initialize peripherals
	Init_SPI();

	//Initialize devices
	Init_nrf();
    	
    	unsigned char my_data;
	
    	while(1) 
    	{
        
//		_delay_ms(1000);	//1s delay 
//		my_data = 0x55;
//		transmit_data(my_data);
		nRF_Put_String("Hmmmmmm...\n");
		reset();

		_delay_ms(10);   	//1s delay
//		my_data = 0xAA;
//		transmit_data(my_data);
		reset();
    	}
}