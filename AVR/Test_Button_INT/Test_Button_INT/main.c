/*
 * Test_Button_INT.c
 *
 * Created: 26-Jul-18 12:28:40 PM
 * Author : Frederic Philips
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>

ISR(INT6_vect)
{	
	cli();			//Disable global interrupt
	
	_delay_ms(50);	//Switch debounce delay
	
	PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
	_delay_ms(1000);		//1 second delay
	PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
	
	sei();			//Enable global interrupt	
}

int main(void)
{
	DDRC |= _BV(6);		//Makes PORTC, bit 6 as Output
	
	EICRB &= ~(1 << ISC60) | (1 << ISC61);	//INT6 active when low
	EIMSK |= (1 << INT6);			//Enable INT6
	sei();					//Enable global interrupts

	//Loop forever
	for(;;)
	{
	}
}