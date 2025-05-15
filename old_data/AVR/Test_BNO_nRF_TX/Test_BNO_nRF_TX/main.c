/*
 * Test_BNO_nRF_TX.c
 *
 * Created: 15-Aug-18 8:58:16 PM
 * Author : Frederic Philips
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <stdint.h>
#include <stdlib.h>
#define F_CPU 1000000UL	//1 MHz frequency
#include <util/delay.h>

#include "Test_BNO055.h"
#include "i2cmaster.h"
#include "nrf.h"

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

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

	DDRC |= _BV(6);		//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);		//Makes PORTC, bit 7 as Output
	
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

	PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
	PORTC &= ~(_BV(7));	//Turns OFF LED in Port C pin 7

	//Initialize TWI data
	TWI_data = 0;
}

void BNO_Get_ID(void)
{
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_CHIP_ID_ADDR);			//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0xA0
	i2c_stop();
	UART_Tx(TWI_data);

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_ACCEL_REV_ID_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0xFB
	i2c_stop();
	UART_Tx(TWI_data);

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_MAG_REV_ID_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0x32
	i2c_stop();
	UART_Tx(TWI_data);

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_GYRO_REV_ID_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0x0F
	i2c_stop();
	UART_Tx(TWI_data);
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
	Write_byte(SETUP_RETR, 0x2F);
	
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
	_delay_us(15);		//Need at least 10us before sending
	PORTB |= _BV(CE);	//CE high
	_delay_us(20);  	//Hold CE high for at least 10us and not longer than 4ms
	PORTB &= ~_BV(CE);	//CE low
	_delay_ms(1); 		//Delay needed for retransmissions before reset
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

	unsigned char Euler_Raw_LSB;
	unsigned char Euler_Raw_MSB;

	char String_Data[16];

	float angle_scale = 1.0f/16.0f;

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_OPR_MODE_ADDR);
	i2c_write(OPERATION_MODE_NDOF);		//Set operation mode to IMU
	i2c_stop();
	_delay_ms(10);

	//Endless Loop
	while(1)
	{


		i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_H_LSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
		Euler_Raw_LSB = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_H_MSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
		Euler_Raw_MSB = i2c_readNak();
		i2c_stop();

		int16_t Euler_H_Raw = (Euler_Raw_MSB << 8) | (Euler_Raw_LSB);

		float Euler_H = (float)(Euler_H_Raw) * angle_scale;

		itoa(Euler_H, String_Data, 10);			//Convert integer to string, radix=10

		nRF_Put_String("Y: ");
		nRF_Put_String(String_Data);

		i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_R_LSB_ADDR);		//Access LSB of Roll Euler angle
		i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
		Euler_Raw_LSB = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_R_MSB_ADDR);		//Access MSB of Roll Euler angle
		i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
		Euler_Raw_MSB = i2c_readNak();
		i2c_stop();

		int16_t Euler_R_Raw = (Euler_Raw_MSB << 8) | (Euler_Raw_LSB);

		float Euler_R = (float)(Euler_R_Raw) * angle_scale;

		itoa(Euler_R, String_Data, 10);  //convert integer to string, radix=10

		nRF_Put_String(" R: ");
		nRF_Put_String(String_Data);

		i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_P_LSB_ADDR);		//Access LSB of Pitch Euler angle
		i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
		Euler_Raw_LSB = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_P_MSB_ADDR);		//Access LSB of Pitch Euler angle
		i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
		Euler_Raw_MSB = i2c_readNak();
		i2c_stop();

		int16_t Euler_P_Raw = (Euler_Raw_MSB << 8) | (Euler_Raw_LSB);

		float Euler_P = (float)(Euler_P_Raw) * angle_scale;

		itoa(Euler_P, String_Data, 10);  //convert integer to string, radix=10

		nRF_Put_String(" P: ");
		nRF_Put_String(String_Data);
		nRF_Put_String("\n");

//		_delay_ms(500);
	}
}
