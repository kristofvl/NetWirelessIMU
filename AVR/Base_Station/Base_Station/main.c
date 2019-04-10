/*
 * Base_Station.c
 *
 * Created: 21-Nov-18 6:18:12 PM
 * Author : Frederic Philips
 */ 
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <stdint.h>
#include <stdlib.h>
#define F_CPU 16000000UL	//16 MHz frequency
#define BAUD  57600
#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nrf.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

#define PAYLOAD_LEN	8
#define ADDR_WIDTH	5

//Global arrays to hold TX and RX Payloads
uint8_t BS_payload_TX[PAYLOAD_LEN] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
uint8_t BS_payload_RX[PAYLOAD_LEN];

//Global arrays Addresses of Base Station and nodes
uint8_t BS_Address[ADDR_WIDTH] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
uint8_t N1_Address[ADDR_WIDTH] = {0x11, 0x12, 0x13, 0x14, 0x15};
uint8_t N2_Address[ADDR_WIDTH] = {0x21, 0x22, 0x23, 0x24, 0x25};

//0 - TX; 1 - RX
//Note: Declare as volatile as this is used in ISR
volatile uint8_t nRF_mode;

void nRF_TX_mode(void);
void nRF_RX_mode(void);
void nRF_Flush_TX(void);
void nRF_Flush_RX(void);
void nRF_Status_Reset(void);
void nRF_Payload_RX(uint8_t *data_out, uint8_t *data_in, uint8_t len);
uint8_t nRF_getStatus(void);
uint8_t nRF_isSending(void);

/************************************************************************************
** AVR_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
*************************************************************************************/
void AVR_Init(void)
{
	_delay_ms(750);		//Short pause after BNO055 Power-On Reset(Mandatory)
	DDRD |= _BV(1);		//Set TX as output
	DDRD &= ~(_BV(0));	//Set RX as input
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
** UART_TX function:
** - Transmits the ADC data via the USB Serial
** - The data is received & displayed in a Hyperterminal
*************************************************************************************/
void UART_TX(unsigned char data)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);	//Wait until buffer is empty
	UDR1 = data;				//Send data	
}

void Init_SPI()
{
	//Set the output pin(s) for SPI
	DDRB |= _BV(CE);	//CE
	DDRB |= _BV(CSN);	//CSN
	DDRB |= _BV(MOSI);  	//MOSI
	DDRB |= _BV(SCLK);  	//SCLK

	//Set the input pin(s) for SPI
	DDRB &= ~_BV(MISO); 	//MISO

	//Enable SPI as master and set clock rate	
	SPCR |= ((1 << SPE) | (1 << MSTR) | (1 << SPR0));	
	SPCR &= (~_BV(SPI2X) & ~_BV(SPR1));
	
	PORTB |= _BV(CSN);	//CSN high
	PORTB &= ~_BV(CE);	//CE low
}

unsigned char SPI_Tranceiver(unsigned char data)
{
	// Load data into the buffer
	SPDR = data;
	
	//Wait until transmission complete
	while(!(SPSR & (1 << SPIF)));   

	//Return received data
	return(SPDR);
}

unsigned char nRF_Read_Byte(unsigned char reg)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	SPI_Tranceiver(R_REGISTER + reg);
	_delay_us(10);
	reg = SPI_Tranceiver(NOP);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	return reg;
}

void nRF_Write_Byte(unsigned char reg, unsigned char data)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	SPI_Tranceiver(W_REGISTER + reg);
	_delay_us(10);
	SPI_Tranceiver(data);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
}

void nRF_Init(void)
{
	//Enable auto-acknowledgment for data pipe 0
	nRF_Write_Byte(EN_AA, 0x01);
	
	//Enable data pipe 0
	nRF_Write_Byte(EN_RXADDR, 0x01);

	//Set address width to 5 bytes
	nRF_Write_Byte(SETUP_AW, 0x03);
	
	//Set channel frequency to 2.505GHz
	nRF_Write_Byte(RF_CH, 0x69);
	
	//Set data rate to 2Mbps and 0dB gain
	nRF_Write_Byte(RF_SETUP, 0x0E);
	
	//Enable W_TX_PAYLOAD_NOACK command
//	nRF_Write_Byte(FEATURE, 0x01);
	
	//Set the 5-bytes receiver address as 0x01 0x02 0x03 0x04 0x05
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup p0 pipe address for receiving
	SPI_Tranceiver(W_REGISTER + RX_ADDR_P0);
	_delay_us(10);
	SPI_Tranceiver(0x11);
	_delay_us(10);
	SPI_Tranceiver(0x12);
	_delay_us(10);
	SPI_Tranceiver(0x13);
	_delay_us(10);
	SPI_Tranceiver(0x14);
	_delay_us(10);
	SPI_Tranceiver(0x15);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	
	//Set the 5-bytes transmitter address as 0x01 0x02 0x03 0x04 0x05
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup the transmitter address
	SPI_Tranceiver(W_REGISTER + TX_ADDR);
	_delay_us(10);
	SPI_Tranceiver(0xAA);
	_delay_us(10);
	SPI_Tranceiver(0xBB);
	_delay_us(10);
	SPI_Tranceiver(0xCC);
	_delay_us(10);
	SPI_Tranceiver(0xDD);
	_delay_us(10);
	SPI_Tranceiver(0xEE);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	
	//Set the payload width as 8-bytes
	nRF_Write_Byte(RX_PW_P0, 0x08);
	
	//Set the retransmission delay to 750us with 15 retries
	nRF_Write_Byte(SETUP_RETR, 0x2F);
	
	//Boot the nrf as TX and enable CRC and set the length to 2-bytes
	nRF_TX_mode();
	
	//Minimum 1.5ms delay required from Power-down to Standby mode
	_delay_ms(10);
}

void nRF_TX_mode(void)
{
	//CE Low - Standby-I mode
	PORTB &= ~_BV(CE);
	
	//Set as TX
	nRF_Write_Byte(CONFIG, nRF_Read_Byte(CONFIG) & ~(1 << PRIM_RX));
	
	//Power-up
	nRF_Write_Byte(CONFIG, nRF_Read_Byte(CONFIG) | (1 << PWR_UP));		
	
	//Flush TX FIFO
	nRF_Flush_TX();
	
	//Reset status
	nRF_Write_Byte(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
	
	//Mask TX_DR and MAX_RT interrupts
	nRF_Write_Byte(CONFIG, nRF_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	
	//Minimum 130us delay required after mode change
	_delay_us(150);
}

void nRF_RX_mode(void)
{
	//CE Low - Standby-I mode
	PORTB &= ~_BV(CE);
	
	//Power-up as set as RX
	nRF_Write_Byte(CONFIG, nRF_Read_Byte(CONFIG) | (1 << PWR_UP) | (1 << PRIM_RX));

	//Flush RX FIFO
	nRF_Flush_RX();
	
	//Reset status
	nRF_Write_Byte(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

	//Mask TX_DR and MAX_RT interrupts
	nRF_Write_Byte(CONFIG, nRF_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	
	//CE High - Starts listening
	PORTB |= _BV(CE);
	
	//Minimum 130us delay required after mode change
	_delay_us(150);
}

void nRF_Flush_TX(void)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	SPI_Tranceiver(FLUSH_TX);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(10);
}

void nRF_Flush_RX(void)
{
	_delay_us(10);
	PORTB &= ~_BV(CSN);
	_delay_us(10);
	SPI_Tranceiver(FLUSH_RX);
	_delay_us(10);
	PORTB |= _BV(CSN);
	_delay_us(10);
}

void Payload_TX(uint8_t* data, uint8_t len)
{
	uint8_t i;
	
	for(i = 0; i < len; i++)
	{
		SPI_Tranceiver(BS_payload_TX[i]);
	}
}

void transmit_data(unsigned char *tdata)
{
	nRF_Flush_TX();
	PORTB &= ~_BV(CSN); //CSN low
	_delay_us(10);
	//Transmit payload with ACK enabled
	SPI_Tranceiver(W_TX_PAYLOAD);
	_delay_us(10);
	Payload_TX(BS_payload_TX, PAYLOAD_LEN);
	_delay_us(10);
	PORTB |= _BV(CSN);  //CSN high
	_delay_us(10);      //Need at least 10us before sending
	PORTB |= _BV(CE);   //CE high
	_delay_us(10);      //Hold CE high for at least 10us and not longer than 4ms
	PORTB &= ~_BV(CE);  //CE low
}

uint8_t nrf24_getStatus()
{
	uint8_t rv;
	PORTB &= ~_BV(CSN); //CSN low
	rv = SPI_Tranceiver(NOP);
	PORTB |= _BV(CSN);  //CSN high
	return rv;
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
	// Pull down chip select 
	PORTB &= ~_BV(CSN); //CSN low
	_delay_us(10);
	// Send command to read RX payload 
	SPI_Tranceiver(R_RX_PAYLOAD);
	_delay_us(10);
	// Read payload 
	nRF_Payload_RX(BS_payload_RX, BS_payload_RX, PAYLOAD_LEN);
	_delay_us(10);
	// Pull up chip select
	PORTB |= _BV(CSN);  //CSN high
	_delay_us(10);
	// Reset status register 
	nRF_Write_Byte(STATUS, (1 << RX_DR));
	mode = 0;	    //Set as TX
}

/* send and receive multiple bytes over SPI */
void nRF_Payload_RX(uint8_t *data_out, uint8_t *data_in, uint8_t len)
{
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		data_in[i] = SPI_Tranceiver(data_out[i]);
		UART_TX(data_in[i]);   //Send the received data to UART
	}
}

void nRF_Status_Reset(void)
{
	_delay_us(10);
	//Reset IRQ-flags in status register
    	nRF_Write_Byte(STATUS, 0x70);   
	_delay_us(10);
}

int main(void)
{
	AVR_Init();
	Init_SPI();
	nRF_Init();
	UART_Init();
	Init_INT6();
	
	//0 - TX; 1 - RX
	mode = 0;
	
	//Disable Interrupt initially
	cli();

	//Endless Loop
	while(1)
	{
		if(mode == 0) //TX
		{
			//Configure as Transmitter
			nRF_TX_mode();
		
//			UART_TX(0x55);   	//Send BP1 to UART
		
			transmit_data(BS_payload_TX);
			while(nrf24_isSending());
			nRF_Status_Reset();
/*
			while(!nrf24_isSending())
			{
				transmit_data(BS_payload_TX);
//				nRF_Status_Reset();
			}
			nRF_Status_Reset();
*/
//			UART_TX(0x66);   	//Send BP2 to UART
		
			//Configure as Receiver
			mode = 1;		//Set as RX
			nRF_RX_mode();
			PORTB |= _BV(CE);	//Start listening again	
			sei();		

//			UART_TX(0x77);   	//Send BP3 to UART			
		}

//		UART_TX(0x88);   		//Send BP4 to UART
	}
}