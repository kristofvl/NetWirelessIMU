/*
 * RX.c
 *
 * Created: 1/6/2018 5:41:48 AM
 * Author : rbkyo
 */ 
#include <avr/io.h>
#define F_CPU 16000000
#define BAUD  57600
#include <avr/sfr_defs.h>
#include <util/setbaud.h>
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

/************************************************************************************
** USART Reference:
** - ATmega32U4 Datasheet - Rev. CORP072610(Pg.186)
** - AVR Microcontroller and Embedded Systems - Mazidi(Pg.395)
** - Embedded C Programming and the Atmel AVR - Barnett(Pg.132)
*************************************************************************************
** To initialize the UART, the following steps are to be followed:
** - Set the Baud rate(use <util/setbaud.h>, which depends on the macros F_CPU & BAUD)
** - Disable double speed(2x) mode
** - Set the no. of data bits(8/9 bits), stop bit(1/2) and parity bit(None/Odd/Even)
** - Set the USART mode(Synchronous/Asynchronous/Asynchronous 2x)
** - Enable Receiver & Transmitter(Set RXEN & TXEN bits in UCSRB register)
*************************************************************************************/
void UART_Init(void)
{
	DDRD |= _BV(1);		//Set TX as output
	DDRD &= ~(_BV(0));	//Set RX as input

	//Set the BAUD rate(Ref. ATmega32U4 Datasheet Pg.189, Table 18-1)
	//To hard-code the Baud rate, Ref. Tables 18-9 to 18-12 in Pgs. 210 - 213
	UBRR1 = ((F_CPU / (16UL * BAUD)) - 1);
	
	//Disables 2x speed
	UCSR1A &= ~(_BV(U2X1));
	
	//Enable 8-bit character size, one stop-bit, no parity & asynchronous mode
	UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10);
	
	//Enable Transmitter & Receiver
	UCSR1B |= _BV(TXEN1) | _BV(RXEN1);
}

/************************************************************************************
** UART_Tx function:
** - Transmits the ADC data via the USB Serial
** - The data is received & displayed in a Hyperterminal
*************************************************************************************/
void UART_Tx(unsigned char data)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);	//Wait until buffer is empty
	UDR1 = data;				//Send data	
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
	
	//Boot the nrf as RX and mask the maximum retransmission interrupt(disable)
	//Enable CRC and set the length to 2-bytes
	Write_byte(CONFIG, 0x1F);
	
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

void reset(void)
{
	_delay_us(10);
	//Reset IRQ-flags in status register
    	Write_byte(STATUS, 0x70);   
	_delay_us(10);
}

void Init_INT6(void)
{
	EICRB &= ~(1 << ISC60) | (1 << ISC61);	//INT6 active when low
	EIMSK |= (1 << INT6);			//Enable INT6
	sei();					//Enable global interrupts
}

ISR(INT6_vect)
{	
	cli();					//Disable global interrupt
	
	PORTB &= ~_BV(CE); 			//Stop listening
	unsigned char my_data = Read_Byte(R_RX_PAYLOAD);
	
	UART_Tx(my_data);			//Send the received data to UART
	
	Flush_rx();				//Flush RX
	reset();				//Reset IRQ
	sei();
	PORTB |= _BV(CE);  			//Start listening again
}

int main(void)
{

	Init_SPI();
	Init_INT6();
	Init_nrf();
	UART_Init();
	Flush_rx();
	reset();
	PORTB |= _BV(CE);			//Start listening
    
 	while (1) 
    	{
    		//Loop forever		
    	}
}
