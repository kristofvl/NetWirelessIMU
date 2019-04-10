/*
 * Test_MAX17043_N1_TX.c
 *
 * Created: 21-Dec-18 11:18:43 PM
 * Author : Frederic Philips
 */ 
 
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <stdint.h>
#include <stdlib.h>
#define F_CPU 1000000UL	//1 MHz frequency
#include <util/delay.h>
#include <avr/interrupt.h>

#include "MAX17043.h"
#include "i2cmaster.h"
#include "nRF.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

#define PAYLOAD_LEN	4

uint8_t N1_payload_TX[PAYLOAD_LEN];
uint8_t N1_payload_RX[PAYLOAD_LEN];

volatile uint8_t RX_Payload_cnt;

void nRF_TX_Mode(void);
void nRF_RX_Mode(void);
void nRF_Flush_TX(void);
void nRF_Flush_RX(void);
void nRF_Reset(void);
void nRF_get_Payload(uint8_t *data_out, uint8_t *data_in, uint8_t len);
uint8_t nRF_get_Status(void);
uint8_t nRF_is_Sending(void);
void MAX17043_Power_On_Reset(void);
void MAX17043_Config(void);
void MAX17043_Read_VCELL(void);
void MAX17043_Read_SOC(void);
void MAX17043_Write_Word(uint8_t reg, uint8_t MSB, uint8_t LSB);
void MAX17043_Read_Word(uint8_t reg, uint8_t *MSB, uint8_t *LSB);

/************************************************************************************
** AVR_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
** - Short pause after BNO055 Power-On Reset(Mandatory)
*************************************************************************************/
void AVR_Init(void)
{
	DDRD |= _BV(1);			//Set TX as output
	DDRD &= ~(_BV(0));		//Set RX as input

	//Make LED pins as output
	DDRC |= _BV(6);			//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);			//Makes PORTC, bit 7 as Output
	
	//Start-up LED sequence loop
	for(int i = 5; i != 0; i--)
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

	//Initialize TWI data
	TWI_data = 0;
}

//MAX17043 Power-on Reset
void MAX17043_Power_On_Reset(void)
{
	MAX17043_Write_Word(MAX17043_COMMAND, MAX17043_POR_H, MAX17043_POR_L);
}

//MAX17043 Configuration
void MAX17043_Config(void)
{
	MAX17043_Write_Word(MAX17043_CONFIG, MAX17043_RCOMP, MAX17043_ALT_30);
}

void MAX17043_Read_VCELL(void)
{
	MAX17043_Read_Word(MAX17043_VCELL, &N1_payload_TX[0], &N1_payload_TX[1]);
}

void MAX17043_Read_SOC(void)
{
	MAX17043_Read_Word(MAX17043_SOC, &N1_payload_TX[2], &N1_payload_TX[3]);
}

void MAX17043_Write_Word(uint8_t reg, uint8_t MSB, uint8_t LSB)
{
	i2c_start_wait(MAX17043_ADDR + I2C_WRITE);	//Set device address and write mode
	i2c_write(reg);					//Access the Command register
	i2c_write(MSB);					//Write 0x54 to the lower byte of Command register
	i2c_write(LSB);					//Write 0x00 to the higher byte of Command register
	i2c_stop();					//Stop the I2C transmission
}

void MAX17043_Read_Word(uint8_t reg, uint8_t *MSB, uint8_t *LSB)
{
	i2c_start_wait(MAX17043_ADDR + I2C_WRITE);	//Set device address and write mode
	i2c_write(reg);					//Access the Command register
	i2c_rep_start(MAX17043_ADDR + I2C_READ);	//Set device address and read mode
	*MSB = i2c_readNak();				//Read MSB
	*LSB = i2c_readNak();				//Read LSB
	i2c_stop();					//Stop the I2C transmission
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

	SPCR |= ((1 << SPE) | (1 << MSTR) | (1 << SPR0));	//Enable SPI as master
	SPCR &= (~_BV(SPI2X) & ~_BV(SPR1)); 		   	//Set clock rate but not too important
	
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
	
	//Set the 5-bytes receiver address as 0x01 0x02 0x03 0x04 0x05
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup p0 pipe address for receiving
	SPI_Tranceiver(W_REGISTER + RX_ADDR_P0);
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
	
	//Set the 5-bytes transmitter address as 0x01 0x02 0x03 0x04 0x05
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup the transmitter address
	SPI_Tranceiver(W_REGISTER + TX_ADDR);
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
	
	//Set the payload width as 8-bytes
	SPI_Write_Byte(RX_PW_P0, 0x08);
	
	//Set the retransmission delay to 750us with 15 retries
	SPI_Write_Byte(SETUP_RETR, 0x2F);
	
	//Boot the nRF as RX and mask the maximum retransmission interrupt(disable)
	//Enable CRC and set the length to 2-bytes
	nRF_RX_Mode();
	
	_delay_ms(10);		//10ms delay after power-up
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

void nRF_send_Payload(uint8_t* data, uint8_t len)
{
	uint8_t i;
	
	for(i = 0; i < len; i++)
	{
		SPI_Tranceiver(N1_payload_TX[i]);
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
	nRF_send_Payload(N1_payload_TX, PAYLOAD_LEN);
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
	nRF_get_Payload(N1_payload_RX, N1_payload_RX, PAYLOAD_LEN);
	_delay_us(10);
	// Pull up chip select
	PORTB |= _BV(CSN);  //CSN high
	_delay_us(10);
	// Reset status register
	SPI_Write_Byte(STATUS, (1 << RX_DR));
}

/* send and receive multiple bytes over SPI */
void nRF_get_Payload(uint8_t *data_out, uint8_t *data_in, uint8_t len)
{
	uint8_t i;

	for(i=0; i<len; i++)
	{
		data_in[i] = SPI_Tranceiver(data_out[i]);
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
	//Initialize AVR and peripherals
	AVR_Init();
	i2c_init();
	SPI_Init();
	nRF_Init();
	INT6_Init();

	//MAX17043 Initialization & Configuration
	MAX17043_Power_On_Reset();
	MAX17043_Config();
	
	//Initialize the received payload count
	RX_Payload_cnt = 0;
	
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
			
			MAX17043_Read_VCELL();
			MAX17043_Read_SOC();
			_delay_ms(1);
			
			nRF_TX_Data(N1_payload_TX);
			while(nRF_is_Sending());
			nRF_Reset();
			
			//Configure as Receiver
			nRF_RX_Mode();
			nRF_Flush_RX();
			PORTB |= _BV(CE);	//Start listening again
			sei();
		}
	}
}