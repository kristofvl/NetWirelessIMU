/*
 * BNO_QUAT_UART.c
 *
 * Created: 30-Nov-18 3:29:00 PM
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

#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

int8_t Quat_Payload[8];

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

void BNO_Read_Quaternions(void)
{
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_W_MSB_ADDR);	//Access MSB of Quaternion_W value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[0] = i2c_readNak();
	UART_Tx(Quat_Payload[0]);
	i2c_stop();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_W_LSB_ADDR);	//Access LSB of Quaternion_W value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[1] = i2c_readNak();
	UART_Tx(Quat_Payload[1]);
	i2c_stop();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_X_MSB_ADDR);	//Access MSB of Quaternion_X value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[2] = i2c_readNak();
	UART_Tx(Quat_Payload[2]);
	i2c_stop();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_X_LSB_ADDR);	//Access LSB of Quaternion_X value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[3] = i2c_readNak();
	UART_Tx(Quat_Payload[3]);
	i2c_stop();
	
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Y_MSB_ADDR);	//Access MSB of Quaternion_Y value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[4] = i2c_readNak();
	UART_Tx(Quat_Payload[4]);
	i2c_stop();
	
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Y_LSB_ADDR);	//Access LSB of Quaternion_Y value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[5] = i2c_readNak();
	UART_Tx(Quat_Payload[5]);
	i2c_stop();
	
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Z_MSB_ADDR);	//Access MSB of Quaternion_Z value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[6] = i2c_readNak();
	UART_Tx(Quat_Payload[6]);
	i2c_stop();
	
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_QUATERNION_DATA_Z_LSB_ADDR);	//Access LSB of Quaternion_Z value
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	Quat_Payload[7] = i2c_readNak();
	UART_Tx(Quat_Payload[7]);
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
	UART_Init();

	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_OPR_MODE_ADDR);
	i2c_write(OPERATION_MODE_NDOF);			//Set operation mode to NDOF
	i2c_stop();
	_delay_ms(10);

	//Endless Loop
	while(1)
	{
		BNO_Read_Quaternions();
		_delay_ms(10);		//Wait for 10ms
	}
}

