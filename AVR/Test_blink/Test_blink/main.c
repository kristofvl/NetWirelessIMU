/*
 * Test_blink.c
 *
 * Created: 25-Apr-18 6:57:29 PM
 * Author : Frederic Philips
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#define F_CPU 8000000UL		//8 MHz clock speed
#include <util/delay.h>

int main(void)
{
	//Set the Clock Prescaler division factor to 1(F_CPU = 8MHz)
	clock_prescale_set(clock_div_1);
		
	DDRC |= _BV(6);		//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);		//Makes PORTC, bit 7 as Output
	
	//Infinite loop
	while(1)
	{
		PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
		PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
		_delay_ms(100);		//1 second delay
		
		PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
		PORTC &= ~(_BV(7));	//Turns OFF LED in Port C pin 7
		_delay_ms(100);		//1 second delay
	}
}