/*
 * N2_ACK_INT.c
 *
 * Created: 30.12.2018 15:03:43
 * Author : Frederic Philips
 */
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define F_CPU 8000000UL	//8MHz frequency
#define BAUD  9600
#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nrf.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		5

#define NRF_ADDR_LEN	5
#define PAYLOAD_LEN	10

#define TRANSMISSON_OK	0
#define MESSAGE_LOST	1

uint8_t N2_address[NRF_ADDR_LEN] = {0x21, 0x22, 0x23, 0x24, 0x25};
uint8_t BS_address[NRF_ADDR_LEN] = {0x21, 0x22, 0x23, 0x24, 0x25};

uint8_t N2_payload_TX[PAYLOAD_LEN] = {0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30};
uint8_t N2_payload_RX[PAYLOAD_LEN];

volatile uint8_t RX_Payload_cnt;

void nRF_Set_Addr_TX(uint8_t *addrData, uint8_t addrLen);
void nRF_Set_Addr_RX(uint8_t *addrData, uint8_t addrLen);
void nRF_TX_Mode(void);
void nRF_RX_Mode(void);
void nRF_Flush_TX(void);
void nRF_Flush_RX(void);
void nRF_Reset(void);
void nRF_get_Payload(uint8_t *data_out, uint8_t *data_in, uint8_t len);
uint8_t nRF_get_Status(void);
uint8_t nRF_is_Sending(void);

/************************************************************************************
** AVR_Init function:
** - Resets the Clock Prescalar factor to 1x
** - Start-up delay
** - Initializes the I/O peripherals
** - Plays LED sequence
*************************************************************************************/
void AVR_Init(void)
{
	//Set the Clock Prescaler division factor to 1(F_CPU = 8MHz)
	clock_prescale_set (clock_div_1);

	DDRD |= _BV(1);			//Set TX as output
	DDRD &= ~(_BV(0));		//Set RX as input

	//Make LED pins as output
	DDRC |= _BV(6);			//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);			//Makes PORTC, bit 7 as Output

	//Start-up LED sequence loop
	for (int i = 5; i != 0; i--)
	{
		PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
		PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
		_delay_ms(100);		//0.1 second delay

		PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
		PORTC &= ~(_BV(7));	//Turns OFF LED in Port C pin 7
		_delay_ms(100);		//0.1 second delay
	}

	PORTC &= ~(_BV(6));		//Turns OFF LED in Port C pin 6
	PORTC &= ~(_BV(7));		//Turns OFF LED in Port C pin 7

	_delay_ms(750);			//Short pause after BNO055 Power-On Reset(Mandatory)
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
	//Set the BAUD rate(Ref. ATmega32U4 Datasheet Pg.189, Table 18-1)
	//To hard-code the Baud rate, Ref. Tables 18-9 to 18-12 in Pages 210 - 213
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
** - Transmits the TWI data via the USB Serial
** - The data is received & displayed in a Hyperterminal
*************************************************************************************/
void UART_Tx(unsigned char data)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);		//Wait until buffer is empty
	UDR1 = data;					//Send TWI data via UART
}

void UART_Put_String(char *s)
{
	//Loop through entire string
	while(*s)
	{
	    UART_Tx(*s);
	    s++;
	}
}

void SPI_Init()
{
	//Set the output pin(s) for SPI
	DDRB |= _BV(CE);	//CE
	DDRB |= _BV(CSN);	//CSN
	DDRB |= _BV(MOSI);  	//MOSI
	DDRB |= _BV(SCLK);  	//SCLK

	//Set the input pin(s) for SPI
	DDRB &= ~_BV(MISO); 	//MISO

//	SPCR |= ((1 << SPE) | (1 << MSTR) | (1 << SPR0));	//Enable SPI as master
//	SPCR &= (~_BV(SPI2X) & ~_BV(SPR1)); 		   	//Set clock rate but not too important

	//Enable SPI as master
	SPCR |= ((1 << SPE) | (1 << MSTR));

	//F_CPU/4
//	SPCR &= (~_BV(SPR1) & ~_BV(SPR0));
//	SPSR &= ~_BV(SPI2X);

	//F_CPU/16
//	SPCR |= (1 << SPR0);
//	SPCR &= ~_BV(SPR1);
//	SPSR &= ~_BV(SPI2X);

	//F_CPU/64
//	SPCR |= (1 << SPR1);
//	SPCR &= ~_BV(SPR0);
//	SPSR &= ~_BV(SPI2X);

	//F_CPU/128
//	SPCR |= ((1 << SPR1) | (1 << SPR0));
//	SPSR &=  ~_BV(SPI2X);

	//F_CPU/2
//	SPCR &= (~_BV(SPR1) & ~_BV(SPR0));
//	SPSR |= (1 << SPI2X);

	//F_CPU/8
	SPCR |= (1 << SPR0);
	SPCR &= ~_BV(SPR1);
	SPSR |= (1 << SPI2X);

	//F_CPU/32
//	SPCR |= (1 << SPR1);
//	SPCR &= ~_BV(SPR0);
//	SPSR |= (1 << SPI2X);

	//F_CPU/64
//	SPCR |= ((1 << SPR1) | (1 << SPR0));
//	SPSR |= (1 << SPI2X);

	PORTB |= _BV(CSN);	//CSN high
	PORTB &= ~_BV(CE);	//CE low
	_delay_ms(10);		//10ms delay
}

unsigned char SPI_Tranceiver(unsigned char data)
{
	//Load data into the buffer
	SPDR = data;

	//Wait until transmission complete
	while(!(SPSR & (1 << SPIF)));

	//Return received data
	return(SPDR);
}

unsigned char SPI_Read_Byte(unsigned char reg)
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

void SPI_Write_Byte(unsigned char reg, unsigned char data)
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
	SPI_Write_Byte(EN_AA, 0x01);

	//Enable data pipe 0
	SPI_Write_Byte(EN_RXADDR, 0x01);

	//Set address width to 5 bytes
	SPI_Write_Byte(SETUP_AW, 0x03);

	//Set channel frequency to 2.505GHz
	SPI_Write_Byte(RF_CH, 0x69);

	//Set data rate to 2Mbps and 0dB gain
//	SPI_Write_Byte(RF_SETUP, 0x0E);

	//Set data rate to 250kbps and 0dB gain
	SPI_Write_Byte(RF_SETUP, 0x26);

	//Enable W_TX_PAYLOAD_NOACK command
//	SPI_Write_Byte(FEATURE, 0x01);

	//Set the 5-bytes receiver address as 0x21 0x22 0x23 0x24 0x25
	nRF_Set_Addr_RX(N2_address, NRF_ADDR_LEN);

	//Set the 5-bytes transmitter address as 0x21 0x22 0x23 0x24 0x25
	nRF_Set_Addr_TX(BS_address, NRF_ADDR_LEN);

	//Set the payload width as 10-bytes
//	SPI_Write_Byte(RX_PW_P0, 0x08);
	SPI_Write_Byte(RX_PW_P0, 0x0A);

	//Set the retransmission delay to 750us with 15 retries
	SPI_Write_Byte(SETUP_RETR, 0xFF);

	//Boot the nRF as RX and mask the maximum retransmission interrupt(disable)
	//Enable CRC and set the length to 2-bytes
	nRF_RX_Mode();

	_delay_ms(10);		//10ms delay after power-up
}

void nRF_Set_Addr_RX(uint8_t *addrData, uint8_t addrLen)
{
	uint8_t i;

	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup p0 pipe address for receiving
	SPI_Tranceiver(W_REGISTER + RX_ADDR_P0);

	for(i = 0; i < addrLen; i++)
	{
		_delay_us(10);
		SPI_Tranceiver(N2_address[i]);
	}
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
}

void nRF_Set_Addr_TX(uint8_t *addrData, uint8_t addrLen)
{
	uint8_t i;

	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup p0 pipe address for receiving
	SPI_Tranceiver(W_REGISTER + TX_ADDR);

	for(i = 0; i < addrLen; i++)
	{
		_delay_us(10);
		SPI_Tranceiver(BS_address[i]);
	}
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
}

void nRF_TX_Mode(void)
{
	PORTB &= ~_BV(CE);						 //CE low - Standby-I
	//Power-up and set as TX
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PRIM_RX));
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PWR_UP));
	nRF_Flush_TX();							 //Flush TX FIFO
	SPI_Write_Byte(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT)); //Reset status
	//Mask TX_DR and MAX_RT interrupts
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	_delay_us(150);
}

void nRF_RX_Mode(void)
{
	PORTB &= ~_BV(CE); 						 //CE low - Standby-I
	//Power-up as set as RX
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PWR_UP) | (1 << PRIM_RX));
	nRF_Flush_RX();							 //Flush RX FIFO
	SPI_Write_Byte(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT)); //Reset status
	//Mask TX_DR and MAX_RT interrupts
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	PORTB |= _BV(CE);  						 //CE high
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
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	SPI_Tranceiver(FLUSH_RX);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(10);
}

void nRF_send_Payload(uint8_t *data, uint8_t len)
{
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		SPI_Tranceiver(N2_payload_TX[i]);
	}
}

void nRF_TX_Data(unsigned char *tdata)
{
	nRF_Flush_TX();
	PORTB &= ~_BV(CSN); //CSN low
	_delay_us(10);
	//Transmit payload with ACK enabled
	SPI_Tranceiver(W_TX_PAYLOAD);
	_delay_us(10);
	nRF_send_Payload(N2_payload_TX, PAYLOAD_LEN);
	_delay_us(10);
	PORTB |= _BV(CSN);  //CSN high
	_delay_us(10);      //Need at least 10us before sending
	PORTB |= _BV(CE);   //CE high
	_delay_us(10);      //Hold CE high for at least 10us and not longer than 4ms
	PORTB &= ~_BV(CE);  //CE low
}

uint8_t nRF_get_Status()
{
	uint8_t rv;
	PORTB &= ~_BV(CSN); //CSN low
	rv = SPI_Tranceiver(NOP);
	PORTB |= _BV(CSN);  //CSN high
	return rv;
}

uint8_t nRF_is_Sending()
{
	uint8_t status;

	/* read the current status */
	status = nRF_get_Status();

	/* if sending successful (TX_DS) or max retries exceeded (MAX_RT). */
	if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
	{
		return 0; /* false */
	}

	return 1; /* true */
}

uint8_t nRF_send_Status()
{
	uint8_t rv;

	rv = nRF_get_Status();

	/* Transmission went OK */
	if((rv & ((1 << TX_DS))))
	{
		return TRANSMISSON_OK;
	}

	/* Maximum retransmission count is reached */
	/* Last message probably went missing ... */
	else if((rv & ((1 << MAX_RT))))
	{
		return MESSAGE_LOST;
	}

	/* Probably still sending ... */
	else
	{
		return 0xFF;
	}
}

/* Returns the number of retransmissions occurred for the last message */
uint8_t nRF_RT_Count(void)
{
	uint8_t rv;
	rv = SPI_Read_Byte(OBSERVE_TX);
	rv = rv & 0x0F;
	return rv;
}

void INT6_Init(void)
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
	nRF_get_Payload(N2_payload_RX, N2_payload_RX, PAYLOAD_LEN);
	_delay_us(10);
	// Pull up chip select
	PORTB |= _BV(CSN);  //CSN high
	_delay_us(10);
	// Reset status register
	SPI_Write_Byte(STATUS, (1 << RX_DR));
}

//Send and receive multiple bytes over SPI
void nRF_get_Payload(uint8_t *data_out, uint8_t *data_in, uint8_t len)
{
	uint8_t i;

	for(i=0; i<len; i++)
	{
		data_in[i] = SPI_Tranceiver(data_out[i]);
		UART_Tx(data_in[i]);
		if (data_in[i] == 0xAA)
		{
			RX_Payload_cnt++;
		}
	}
}

void nRF_Reset(void)
{
	_delay_us(10);
	//Reset IRQ-flags in status register
	SPI_Write_Byte(STATUS, 0x70);
	_delay_us(10);
}

/************************************************************************************
** Main function:
** - Contains an endless loop
** - Sets the BNO055 in NDOF mode and fetches the quaternion data
*************************************************************************************/
int main(void)
{
	AVR_Init();
	UART_Init();
	SPI_Init();
	nRF_Init();
	INT6_Init();

	//Initialize the received payload count
	RX_Payload_cnt = 0;

//	char String_Data[16];

	nRF_Flush_RX();
	nRF_Reset();
	PORTB |= _BV(CE);			//Start listening

	//Endless Loop
	while(1)
	{
		if (RX_Payload_cnt == PAYLOAD_LEN)
		{
			RX_Payload_cnt = 0;

			//Configure as Transmitter
			nRF_TX_Mode();

			nRF_TX_Data(N2_payload_TX);
			while(nRF_is_Sending());
/*
			//Make analysis on last transmission attempt
			uint8_t TX_Status = nRF_send_Status();

			if(TX_Status == TRANSMISSON_OK)
			{
				UART_Put_String("Transmission went OK\n");
			}

			else if(TX_Status == MESSAGE_LOST)
			{
				UART_Put_String("Message is lost!\n");
			}

			//Retransmission count indicates the transmission quality
			TX_Status = nRF_RT_Count();
			//Convert integer to string, radix=10
			itoa(TX_Status, String_Data, 10);

			UART_Put_String("Retransmission count:");
			UART_Put_String(String_Data);
			UART_Put_String("\n");
*/
			//Configure as Receiver
			nRF_RX_Mode();
			nRF_Flush_RX();
			nRF_Reset();
			PORTB |= _BV(CE);	//Start listening again
			sei();
		}
	}
}

