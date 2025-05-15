/*
 * EG_BNO_MUX.c
 *
 * Created: 31.12.2018 16:13:10
 * Author : Frederic Philips
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

#include "BNO055.h"
#include "i2cmaster.h"

//Total no. of IMU sensors
#define SEN_COUNT	1

//MUX Select pins
#define MUX_S0		4
#define MUX_S1		6
#define MUX_S2		7

void BNO_MUX_Select(uint8_t sen_channel);

//Raw Euler Bytes
uint8_t Euler_H_LSB;
uint8_t Euler_H_MSB;
uint8_t Euler_R_LSB;
uint8_t Euler_R_MSB;
uint8_t Euler_P_LSB;
uint8_t Euler_P_MSB;

//Raw Euler Word
int16_t Euler_H_Raw;
int16_t Euler_R_Raw;
int16_t Euler_P_Raw;

//Normalized Euler Angles
float Euler_H;
float Euler_R;
float Euler_P;

//Store the Euler float values as string
char Euler_H_String[16];
char Euler_R_String[16];
char Euler_P_String[16];

float angle_scale = 1.0f/16.0f;


/************************************************************************************
** AVR_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
*************************************************************************************/
void AVR_Init(void)
{
	//Set the Clock Prescaler division factor to 1(F_CPU = 8MHz)
	clock_prescale_set (clock_div_1);
	
	DDRD |= _BV(1);		//Set TX as output
	DDRD &= ~(_BV(0));	//Set RX as input

	//Make LED pins as output
	DDRC |= _BV(6);			//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);			//Makes PORTC, bit 7 as Output

	//Make MUX Select pins as output
	DDRD |= _BV(MUX_S0);		//Makes PORTD, bit 4 as Output
	DDRD |= _BV(MUX_S1);		//Makes PORTD, bit 6 as Output
	DDRD |= _BV(MUX_S2);		//Makes PORTD, bit 7 as Output

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

	_delay_ms(750);		//Short pause after BNO055 Power-On Reset(Mandatory)
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

void BNO_Init(void)
{
	//Select IMU 1
	BNO_MUX_Select(1);
		
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_CHIP_ID_ADDR);			//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);		//Set device address and read mode
	uint8_t Chip_ID = i2c_readNak();			//Should read 0xA0
	i2c_stop();
	
	if(Chip_ID != BNO055_CHIP_ID)
	{
		UART_Put_String("BNO055 not detected!\n");
	}
	
	else
	{
		UART_Put_String("BNO055 successfully connected\n");
		UART_Tx(Chip_ID);
	}

	//Reset BNO055
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_SYS_TRIGGER_ADDR);
	i2c_write(BNO055_RESET);			//Set operation mode to IMU
	i2c_stop();
	
	_delay_ms(750);					//Pause after reset
/*	
	//Set clock source as external for BNO055
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_SYS_TRIGGER_ADDR);
	i2c_write(BNO055_EXT_CLK);			//Set operation mode to IMU
	i2c_stop();

	_delay_ms(100);					//Pause after setting clock source
*/
	//Set operating mode
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_OPR_MODE_ADDR);
	i2c_write(OPERATION_MODE_NDOF);			//Set operation mode to IMU
	i2c_stop();
	
	_delay_ms(100);					//Pause after setting operating mode
}


void BNO_MUX_Select(uint8_t sen_channel)
{
	switch (sen_channel)
	{
		//Select IMU 1	-->  Y5
		case 1:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD |= _BV(MUX_S2);		//S2 High
			break;

		//Select IMU 2	-->  Y4
		case 2:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD |= _BV(MUX_S2);		//S2 High
			break;

		//Select IMU 3	-->  Y2
		case 3:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD |= _BV(MUX_S1);		//S1 High
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		//Select IMU 4	-->  Y1
		case 4:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		//Select IMU 5	-->  Y0
		case 5:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		//Select IMU 6	-->  Y3
		case 6:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD |= _BV(MUX_S1);		//S1 High
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		default:
			UART_Put_String("Wrong Channel!\n");
	}
}

void BNO_get_ID(void)
{
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_CHIP_ID_ADDR);			//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0xA0
	i2c_stop();
	UART_Tx(TWI_data);

	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_ACCEL_REV_ID_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0xFB
	i2c_stop();
	UART_Tx(TWI_data);

	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_MAG_REV_ID_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0x32
	i2c_stop();
	UART_Tx(TWI_data);

	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_GYRO_REV_ID_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);		//Set device address and read mode
	TWI_data = i2c_readNak();			//Should read 0x0F
	i2c_stop();
	UART_Tx(TWI_data);
}

void BNO_get_Calib_Stat(void)
{
	i2c_start_wait(BNO055_ADDRESS+I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_CALIB_STAT_ADDR);		//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS+I2C_READ);		//Set device address and read mode
	uint8_t Cal_Stat_Data = i2c_readNak();		//Should read 0x0F
	i2c_stop();
	
	char String_Data[16];
	unsigned char Sys_Calib;
	unsigned char Gyr_Calib;
	unsigned char Acc_Calib;
	unsigned char Mag_Calib;
	
	UART_Put_String("[CALIB] ");
	UART_Put_String("SYS: ");
	Sys_Calib = (Cal_Stat_Data & 0xC0) >> 6;
	itoa(Sys_Calib, String_Data, 10);			//Convert integer to string, radix=10
	UART_Put_String(String_Data);
	
	UART_Put_String("GYR: ");
	Gyr_Calib = (Cal_Stat_Data & 0x30) >> 4;
	itoa(Gyr_Calib, String_Data, 10);			//Convert integer to string, radix=10
	UART_Put_String(String_Data);
	
	UART_Put_String("ACC: ");
	Acc_Calib = (Cal_Stat_Data & 0x0C) >> 2;
	itoa(Acc_Calib, String_Data, 10);			//Convert integer to string, radix=10
	UART_Put_String(String_Data);
	
	UART_Put_String("MAG: ");
	Mag_Calib = (Cal_Stat_Data & 0x03);
	itoa(Mag_Calib, String_Data, 10);			//Convert integer to string, radix=10
	UART_Put_String(String_Data);
	UART_Put_String("\n");
}

void BNO_get_Euler(void)
{
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_EULER_H_LSB_ADDR);		//Access LSB of Heading Euler angle
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	Euler_H_LSB = i2c_readAck();			//Read Euler_H LSB
	Euler_H_MSB = i2c_readAck();			//Read Euler_H MSB
	Euler_R_LSB = i2c_readAck();			//Read Euler_R LSB
	Euler_R_MSB = i2c_readAck();			//Read Euler_R MSB
	Euler_P_LSB = i2c_readAck();			//Read Euler_P LSB
	Euler_P_MSB = i2c_readNak();			//Read Euler_P MSB
	i2c_stop();

	//Combine the raw Euler bytes to word
	Euler_H_Raw = (Euler_H_MSB << 8) | (Euler_H_LSB);
	Euler_R_Raw = (Euler_R_MSB << 8) | (Euler_R_LSB);
	Euler_P_Raw = (Euler_P_MSB << 8) | (Euler_P_LSB);

	//Normalize the Euler Angles
	Euler_H = (float)(Euler_H_Raw) * angle_scale;
	Euler_R = (float)(Euler_R_Raw) * angle_scale;
	Euler_P = (float)(Euler_P_Raw) * angle_scale;

	//Convert integer to string, radix=10
	itoa(Euler_H, Euler_H_String, 10);
	itoa(Euler_R, Euler_R_String, 10);
	itoa(Euler_P, Euler_P_String, 10);

	//Send the Euler angles via UART
	UART_Put_String("EH: ");
	UART_Put_String(Euler_H_String);
	UART_Put_String(" ER: ");
	UART_Put_String(Euler_R_String);
	UART_Put_String(" EP: ");
	UART_Put_String(Euler_P_String);
	UART_Put_String("\n");
}

void BNO_get_Mag_Data(void)
{
	//Store the Mag float values as string
	char Mag_X_String[16];
	char Mag_Y_String[16];
	char Mag_Z_String[16];
	
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_MAG_DATA_X_LSB_ADDR);		//Access LSB of Magnetometer X-axis reading
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	uint8_t Mag_X_LSB = i2c_readAck();		//Read Mag_X LSB
	uint8_t Mag_X_MSB = i2c_readAck();		//Read Mag_X MSB
	uint8_t Mag_Y_LSB = i2c_readAck();		//Read Mag_Y LSB
	uint8_t Mag_Y_MSB = i2c_readAck();		//Read Mag_Y MSB
	uint8_t Mag_Z_LSB = i2c_readAck();		//Read Mag_Z LSB
	uint8_t Mag_Z_MSB = i2c_readNak();		//Read Mag_Z MSB
	i2c_stop();

	//Combine the raw Mag bytes to word
	int16_t Mag_X_Raw = (Mag_X_MSB << 8) | (Mag_X_LSB);
	int16_t Mag_Y_Raw = (Mag_Y_MSB << 8) | (Mag_Y_LSB);
	int16_t Mag_Z_Raw = (Mag_Z_MSB << 8) | (Mag_Z_LSB);

	//Convert integer to string, radix=10
	itoa(Mag_X_Raw, Mag_X_String, 10);
	itoa(Mag_Y_Raw, Mag_Y_String, 10);
	itoa(Mag_Z_Raw, Mag_Z_String, 10);

	//Send the Euler angles via UART
	UART_Put_String("MX: ");
	UART_Put_String(Mag_X_String);
	UART_Put_String(" MY: ");
	UART_Put_String(Mag_Y_String);
	UART_Put_String(" MZ: ");
	UART_Put_String(Mag_Z_String);
	UART_Put_String("  ");	
} 

/*
void BNO_get_Quaternion(void)
{

}
*/
void UART_Put_String(char *s)
{
	//Loop through entire string
	while(*s)
	{
		UART_Tx(*s);
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
	UART_Init();
	i2c_init();
	BNO_Init();
	
	unsigned char Euler_Raw_LSB;
	unsigned char Euler_Raw_MSB;

	char String_Data[16];	

	//Endless Loop
	while(1)
	{

//		BNO_get_Calib_Stat();
		BNO_get_Mag_Data();
		BNO_get_Euler();
/*
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

		UART_Put_String("Euler Heading: ");
		UART_Put_String(String_Data);

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

		UART_Put_String(" Euler Roll: ");
		UART_Put_String(String_Data);

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

		itoa(Euler_P, String_Data, 10);			//Convert integer to string, radix=10

		UART_Put_String(" Euler Pitch: ");
		UART_Put_String(String_Data);
		UART_Put_String("\n");
*/
		_delay_ms(500);
	}
}

