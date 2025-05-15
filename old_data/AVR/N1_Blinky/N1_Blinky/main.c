/*
 * N1_Blinky.c
 *
 * Created: 22.12.2018 06:43:32
 * Author : fredwin
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#define F_CPU 1000000UL	//1 MHz clock speed
#include <util/delay.h>

int main(void)
{
	DDRC |= _BV(6);		//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);		//Makes PORTC, bit 7 as Output
	
	//Infinite loop
	while(1)
	{
		PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
		PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
		_delay_ms(500);		//1 second delay
		
		PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
		PORTC &= ~(_BV(7));	//Turns OFF LED in Port C pin 7
		_delay_ms(500);		//1 second delay
	}
}

