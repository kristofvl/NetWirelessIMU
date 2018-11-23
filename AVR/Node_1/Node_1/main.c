/*
 * Node_1.c
 *
 * Created: 21-Nov-18 6:21:39 PM
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

#include "Test_BNO055.h"
#include "i2cmaster.h"
#include "nrf.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

#define PAYLOAD_LEN	8

uint8_t N1_payload_TX[PAYLOAD_LEN];
uint8_t N1_payload_RX[PAYLOAD_LEN];

uint8_t RX_Payload_cnt;

void nRF_TX_mode(void);
void nRF_RX_mode(void);
void Flush_tx(void);
void Flush_rx(void);
void reset(void);

void Payload_RX(uint8_t *data_out, uint8_t *data_in, uint8_t len);
uint8_t nrf24_getStatus(void);
uint8_t nrf24_dataReady(void);
uint8_t nrf24_rxFifoEmpty(void);
uint8_t nrf24_isSending(void);

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

	//Initialize TWI data
	TWI_data = 0;
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
	
	//Set the 5-bytes transmitter address as 0x01 0x02 0x03 0x04 0x05
	_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(10);
	//Setup the transmitter address
	spi_tranceiver(W_REGISTER + TX_ADDR);
	_delay_us(10);
	spi_tranceiver(0x11);
	_delay_us(10);
	spi_tranceiver(0x12);
	_delay_us(10);
	spi_tranceiver(0x13);
	_delay_us(10);
	spi_tranceiver(0x14);
	_delay_us(10);
	spi_tranceiver(0x15);
	_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	
	//Set the payload width as 8-bytes
	Write_byte(RX_PW_P0, 0x08);
	
	//Set the retransmission delay to 750us with 15 retries
//	Write_byte(SETUP_RETR, 0x2F);
	
	//Boot the nrf as RX and mask the maximum retransmission interrupt(disable)
	//Enable CRC and set the length to 2-bytes
	nRF_RX_mode();
	
	_delay_ms(100);
}

void nRF_TX_mode(void)
{
	PORTB &= ~_BV(CE); //CE low
	Write_byte(CONFIG, 0x1E);
	Flush_tx();
	_delay_us(150);
}

void nRF_RX_mode(void)
{
	PORTB &= ~_BV(CE); //CE low
	Write_byte(CONFIG, 0x1F);
	Flush_rx();
	reset();
	PORTB |= _BV(CE);  //CE high
	_delay_us(150);
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
		spi_tranceiver(N1_payload_TX[i]);
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
	Payload_TX(N1_payload_TX, PAYLOAD_LEN);
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

void nrf24_getData(uint8_t* data)
{
	/* Pull down chip select */
	PORTB &= ~_BV(CSN); //CSN low

	/* Send command to read RX payload */
	spi_tranceiver(R_RX_PAYLOAD);
	
	/* Read payload */
	Payload_RX(data, data, PAYLOAD_LEN);
	
	/* Pull up chip select */
	PORTB |= _BV(CSN);  //CSN high

	/* Reset status register */
	Write_byte(STATUS, (1<<RX_DR));
}

/* send and receive multiple bytes over SPI */
void Payload_RX(uint8_t *data_out, uint8_t *data_in, uint8_t len)
{
	uint8_t i;

	for(i=0; i<len; i++)
	{
		data_in[i] = spi_tranceiver(data_out[i]);
		UART_Tx(data_in[i]);		   //Send the received data to UART
		if (data_in[i] == 0xAA)
		{
			UART_Tx(RX_Payload_cnt);   //Send RX_Payload count to UART
			RX_Payload_cnt++;
		}
	}
}

uint8_t nrf24_getStatus()
{
	uint8_t rv;
	PORTB &= ~_BV(CSN); //CSN low
	rv = spi_tranceiver(NOP);
	PORTB |= _BV(CSN);  //CSN high
	return rv;
}

/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */
uint8_t nrf24_dataReady()
{
	// See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = nrf24_getStatus();

	// We can short circuit on RX_DR, but if it's not set, we still need
	// to check the FIFO for any pending packets
	if (status & (1 << RX_DR))
	{
		return 1;
	}

	return !nrf24_rxFifoEmpty();;
}

/* Checks if receive FIFO is empty or not */
uint8_t nrf24_rxFifoEmpty()
{
	uint8_t fifoStatus;

	fifoStatus = Read_Byte(FIFO_STATUS);
	
	return (fifoStatus & (1 << RX_EMPTY));
}

void reset(void)
{
	_delay_us(10);
	//Reset IRQ-flags in status register
	Write_byte(STATUS, 0x70);
	_delay_us(10);
}

void BNO_Read_Quaternions(void)
{
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_W_LSB_ADDR);	//Access LSB of Quaternion_W value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[0] = i2c_readNak();
//	UART_Tx(N1_payload_TX[0]);
	i2c_stop();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_W_MSB_ADDR);	//Access MSB of Quaternion_W value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[1] = i2c_readNak();
//	UART_Tx(N1_payload_TX[1]);
	i2c_stop();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_X_LSB_ADDR);	//Access LSB of Quaternion_X value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[2] = i2c_readNak();
//	UART_Tx(N1_payload_TX[2]);
	i2c_stop();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_X_MSB_ADDR);	//Access MSB of Quaternion_X value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[3] = i2c_readNak();
//	UART_Tx(N1_payload_TX[3]);
	i2c_stop();
		
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Y_LSB_ADDR);	//Access LSB of Quaternion_Y value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[4] = i2c_readNak();
//	UART_Tx(N1_payload_TX[4]);
	i2c_stop();
		
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Y_MSB_ADDR);	//Access MSB of Quaternion_Y value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[5] = i2c_readNak();
//	UART_Tx(N1_payload_TX[5]);
	i2c_stop();
		
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Z_LSB_ADDR);	//Access LSB of Quaternion_Z value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[6] = i2c_readNak();
//	UART_Tx(N1_payload_TX[6]);
	i2c_stop();
		
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Z_MSB_ADDR);	//Access MSB of Quaternion_Z value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	N1_payload_TX[7] = i2c_readNak();
//	UART_Tx(N1_payload_TX[7]);
	i2c_stop();	
}

/************************************************************************************
** Main function:
** - Contains an endless loop
** - Sets the BNO055 in NDOF mode and fetches the quaternion data
*************************************************************************************/
int main(void)
{
	AVR_Init();
	i2c_init();
	
	Init_SPI();
	Init_nrf();
	UART_Init();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_OPR_MODE_ADDR);
	i2c_write(OPERATION_MODE_NDOF);			//Set operation mode to NDOF
	i2c_stop();
	_delay_ms(10);
	
	Flush_rx();
	reset();
	PORTB |= _BV(CE);				//Start listening
	
	//Initialize the received payload count
	RX_Payload_cnt = 0;

	//Endless Loop
	while(1)
	{	
	        if(nrf24_dataReady())
	        {
		        nrf24_getData(N1_payload_RX);
	        }

	        UART_Tx(0xBB);   	//Send BP6 to UART
		
		if (RX_Payload_cnt == PAYLOAD_LEN)
		{
			UART_Tx(0x55);   //Send BP1 to UART
			
			RX_Payload_cnt = 0;
			
			//Read the Quaternions data from the BNO055
			BNO_Read_Quaternions();

			UART_Tx(0x66);   //Send BP2 to UART
			
			//Configure as Transmitter
			nRF_TX_mode();
//			_delay_us(1000);

			UART_Tx(0x77);   //Send BP3 to UART
			
			//Transmit Quaternion payload
			transmit_data(N1_payload_TX);
			while(nrf24_isSending());
			reset();

			UART_Tx(0x88);   //Send BP4 to UART
				
			//Configure as Receiver
			nRF_RX_mode();
//			_delay_us(150);

			UART_Tx(0x99);   //Send BP5 to UART
		}

		UART_Tx(0x00);   //Send BP5 to UART
	}
}