/*
 * Test_BNO055.c
 *
 * Created: 15-May-18 4:31:14 PM
 * Author : Frederic Philips
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#define F_CPU 16000000UL //16 MHz frequency
#define BAUD  9600
#include <util/setbaud.h>
#include <util/delay.h>

#include "Test_BNO055.h"

/************************************************************************************
** AVR_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
*************************************************************************************/
void AVR_Init(void)
{
	_delay_ms(750);		//Short pause after BNO055 Power-On Reset(Mandatory)
	DDRD |= _BV(1);		//Set TX as output
	DDRD |= ~(_BV(0));	//Set RX as input
	
	//Initialize TWI data
	TWI_data = 0;
}

/************************************************************************************
** TWI_Init function:
** Sets the I2C in Fast mode:
** - TWBR = ((F_CPU / F_SCL) - 16) / (2 * (4 ^ TWPS)))
** - Here: F_CPU = 16 MHz, F_SCL = 400 kHz(Fast I2C), TWPS(TWI Pre-Scalar) = 0x00
*************************************************************************************/
void TWI_Init(void)
{
    //Set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;

//	UART_Tx(0x01);	
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
** - Transmits the TWI data via the USB Serial
** - The data is received & displayed in a Hyperterminal
*************************************************************************************/
void UART_Tx(unsigned char data)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);  //Wait until buffer is empty
	UDR1 = data;						   //Send TWI data via UART
}

/************************************************************************************
** TWI_Start function:
** Initializes the TWI and communicates the save device for read/write
** address = Slave_address + read/write
*************************************************************************************/
void TWI_Start(uint8_t address)
{
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	
	while(!(TWCR & (1 << TWINT)));
//	UART_Tx(0x03);
//  loop_until_bit_is_set(TWCR, TWINT);	

	TWDR = address;
    TWCR = _BV(TWINT) | _BV(TWEN);
    while(!(TWCR & (1 << TWINT)));	
}

/************************************************************************************
** TWI_Stop function:
** Stops the TWI transmission
*************************************************************************************/
void TWI_Stop(void)
{
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

/************************************************************************************
** TWI_Write function:
** Writes the data on the TWI bus
*************************************************************************************/
void TWI_Write(uint8_t data)
{
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & (1 << TWINT)));
//    loop_until_bit_is_set(TWCR, TWINT);	
}

/************************************************************************************
** TWI_Read function:
** Reads the data from the TWI bus
*************************************************************************************/
uint8_t TWI_Read(void)
{
//    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & (1 << TWINT)));
//    loop_until_bit_is_set(TWCR, TWINT);	
    return TWDR;	
}

/************************************************************************************
** Send_to_IMU function:
** Writes an 8-bit data to a specific address on the slave device
*************************************************************************************/
void Send_to_IMU(uint8_t devAddr, uint8_t reg, uint8_t byte)
{
	TWI_Start(devAddr + I2C_WRITE);
	TWI_Write(reg);
	TWI_Write(byte);
	
	TWI_Stop();	
}

/************************************************************************************
** Receive_from_IMU function:
** Reads an 8-bit data from a specific address on the slave device
*************************************************************************************/
void Receive_from_IMU(uint8_t devAddr, uint8_t reg)
{
	TWI_Start(devAddr + I2C_WRITE);
	TWI_Write(reg);
	
	TWI_Start(devAddr + I2C_READ);
	TWI_data = TWI_Read();
	
	TWI_Stop();
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
	TWI_Init();
	

	//Endless Loop
	while(1)
	{
		Receive_from_IMU(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR);
		UART_Tx(TWI_data);
		_delay_ms(1000);					   //1 second delay
	}
}

