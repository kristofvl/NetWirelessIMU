/*
 * Test_ADC_UART.c
 *
 * Created: 07-May-18 4:29:30 PM
 * Author : Frederic Philips
 */ 

/************************************************************************************
** For more details regarding avr-libc v2.0.0 library modules, visit the link below:
** https://www.nongnu.org/avr-libc/user-manual/modules.html
************************************************************************************/
#include <avr/io.h>
#define F_CPU 16000000UL //16 MHz frequency
#define BAUD  9600
#include <util/setbaud.h>
#include <util/delay.h>

//Function declarations
void AVR_Init(void);
void ADC_Init(void);
void UART_Init(void);
unsigned char ADC_Pot(void);
void UART_Tx(unsigned char data);

//Global variables
unsigned char POT_Value;

/************************************************************************************
** AVR_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
*************************************************************************************/
void AVR_Init(void)
{
	_delay_ms(500);		//Short pause
	DDRD |= _BV(1);		//Set TX as output
	DDRD &= ~(_BV(0));	//Set RX as input
}

/************************************************************************************
** ADC Reference:
** - ATmega32U4 Datasheet - Rev. CORP072610(Pg.292)
** - AVR Microcontroller and Embedded Systems - Mazidi(Pg.463)
** - Embedded C Programming and the Atmel AVR - Barnett(Pg.167)
*************************************************************************************
** [Note: By default, the successive approximation circuitry requires an input clock 
** frequency between 50kHz and 200kHz to get maximum resolution.]
** To initialize the ADC, the following steps are to be followed:
** - Select a pre-scalar value to determine the operating frequency(16MHz/128 = 125kHz).
** - Select the voltage reference(Internal - AVCC / External - AREF)
** - Select ADC data resolution
** - Enable the ADC
*************************************************************************************/
void ADC_Init(void)
{
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); //Pre-scalar = 128
	ADMUX  |= _BV(REFS0);							//AVCC as reference
	ADMUX  |= _BV(ADLAR);							//8-bit resolution
	ADCSRA |= _BV(ADEN);							//Enable ADC
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
** ADC_Pot function:
** - Resets the ADC Multiplexer
** - Selects the ADC channel
** - Starts the conversion & waits for the new data
** - Returns the latest 8-bit converted data
*************************************************************************************/
unsigned char ADC_Pot(void)
{
	//Reset ADC Multiplexer
	//Select ADC_0
	ADMUX &= 0b11100000;			

	ADCSRA |= _BV(ADSC);				 //Start ADC conversion
	loop_until_bit_is_set(ADCSRA, ADIF); //Wait until conversion is complete
	ADCSRA |= _BV(ADIF);				 //Set ADC interrupt flag again
	return(ADCH);						 //Return the 8-bit converted value
}

/************************************************************************************
** UART_Tx function:
** - Transmits the ADC data via the USB Serial
** - The data is received & displayed in a Hyperterminal
*************************************************************************************/
void UART_Tx(unsigned char data)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);  //Wait until buffer is empty
	UDR1 = data;						   //Send 'H'	
}

/************************************************************************************
** Main function:
** - Contains an endless loop
** - Fetches the ADC data and transmits it serially @1Hz
*************************************************************************************/
int main(void)
{
	AVR_Init();
	ADC_Init();
	UART_Init();

	//Endless Loop
	while(1)
	{
		POT_Value = ADC_Pot();
		UART_Tx(POT_Value);		
		_delay_ms(1000);					   //1 second delay
	}
}
