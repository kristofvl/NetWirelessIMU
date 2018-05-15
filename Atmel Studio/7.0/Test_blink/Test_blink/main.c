/*
 * Test_blink.c
 *
 * Created: 25-Apr-18 6:57:29 PM
 * Author : fred phil
 */ 

#ifndef F_CPU
	#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>

int main(void)
{
	DDRC |= _BV(6); //makes PORTC, bit 6 as Output
	DDRC |= _BV(7); //makes PORTC, bit 7 as Output
	while(1) //infinite loop
	{
		PORTC &= ~(_BV(6)); //Turns OFF All LEDs
		PORTC |= _BV(7); //Turns ON All LEDs
		_delay_ms(500); //1 second delay
		
		PORTC |= _BV(6); //Turns ON All LEDs
		PORTC &= ~(_BV(7)); //Turns OFF All LEDs
		_delay_ms(500); //1 second delay
	}
}

