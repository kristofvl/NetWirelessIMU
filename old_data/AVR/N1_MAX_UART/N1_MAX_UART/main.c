/*
 * N1_MAX_UART.c
 *
 * Created: 22.12.2018 07:19:14
 * Author : fredwin
 */
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/power.h>
#define F_CPU 8000000UL	//8 MHz frequency
#define BAUD  9600
#include <util/setbaud.h>
#include <util/delay.h>

#include "MAX17043.h"
#include "i2cmaster.h"

void MAX17043_Power_On_Reset(void);
void MAX17043_Config(void);
void MAX17043_Read_VCELL(void);
void MAX17043_Read_SOC(void);
void MAX17043_Write_Word(uint8_t reg, uint8_t MSB, uint8_t LSB);
void MAX17043_Read_Word(uint8_t reg, uint8_t *MSB, uint8_t *LSB);

unsigned char VCell_H;
unsigned char VCell_L;
unsigned char SOC_H;
unsigned char SOC_L;

/************************************************************************************
** AVR_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
** - Short pause after BNO055 Power-On Reset(Mandatory)
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

//MAX17043 Power-on Reset
void MAX17043_Power_On_Reset(void)
{
	MAX17043_Write_Word(MAX17043_COMMAND, MAX17043_POR_H, MAX17043_POR_L);
}

//MAX17043 Quick Start
void MAX17043_Quick_Start(void)
{
	MAX17043_Write_Word(MAX17043_MODE, MAX17043_QS_H, MAX17043_QS_L);
}

//MAX17043 Configuration
void MAX17043_Config(void)
{
	MAX17043_Write_Word(MAX17043_CONFIG, MAX17043_RCOMP, MAX17043_ALT_30);
}

void MAX17043_Read_VCELL(void)
{
	MAX17043_Read_Word(MAX17043_VCELL, &VCell_H, &VCell_L);

	UART_Tx(VCell_H);
	UART_Tx(VCell_L);

//	VCell = ((VCell_L | (VCell_H << 8)) >> 4);
//	VCell *= MAX17043_VCELL_RES;

//	return VCell;
}

void MAX17043_Read_SOC(void)
{
	MAX17043_Read_Word(MAX17043_SOC, &SOC_H, &SOC_L);

	UART_Tx(SOC_H);
	UART_Tx(SOC_L);

	//	float SOC_decimal = SOC_L / MAX17043_SOC_RES;

	//	return SOC_H + SOC_decimal;
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
	UART_Init();

	VCell_H = 0;
	VCell_L = 0;
	SOC_H = 0;
	SOC_L = 0;

	//MAX17043 Initialization & Configuration
	MAX17043_Power_On_Reset();
	MAX17043_Config();
	MAX17043_Quick_Start();
	_delay_ms(1000);


	//Endless Loop
	while(1)
	{
//		UART_Put_String("\n Data:");
/*
		i2c_start_wait(MAX17043_ADDR + I2C_WRITE);	//Set device address and write mode
		i2c_write(MAX17043_VCELL);					//Access the Command register
		i2c_rep_start(MAX17043_ADDR + I2C_READ);	//Set device address and read mode
		VCell_H = i2c_readAck();				//Read MSB
		VCell_L = i2c_readNak();				//Read LSB
		i2c_stop();
		UART_Tx(VCell_H);
		UART_Tx(VCell_L);

		_delay_ms(500);

		i2c_start_wait(MAX17043_ADDR + I2C_WRITE);	//Set device address and write mode
		i2c_write(MAX17043_SOC);					//Access the Command register
		i2c_rep_start(MAX17043_ADDR + I2C_READ);	//Set device address and read mode
		SOC_H = i2c_readAck();				//Read MSB
		SOC_L = i2c_readNak();				//Read LSB
		i2c_stop();
		UART_Tx(SOC_H);
		UART_Tx(SOC_L);

		_delay_ms(500);
*/
		MAX17043_Read_VCELL();
		_delay_ms(500);
		MAX17043_Read_SOC();
		_delay_ms(500);
	}

}
