/*
 * EG_Final.c
 *
 * Created: 15-Mar-19 1:18:24 AM
 * Author : Jochen Kempfle
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "config.h"

#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nrf.h"
#include "NRF24L01p.h"
#include "SPI.h"
#include "BNO055.h"
#include "i2cmaster.h"



uint8_t BS_address[NRF_ADDR_LEN] = {0x21, 0x22, 0x23, 0x24, 0x25}; // <------- delete

// Addresses should have "interesting" bits with variety, not just all bits ones or zeros.
// base station's data address to communicate with single node and receive data. Address is set to (BASE DATA xx)
uint8_t BS_data_address[5] = {0xBA, 0x5E, 0xDA, 0x7A, 0xFF};

// base station's broadcast address to communicate with all nodes. At the moment only used in the beginning to set data address. Address is set to (BASE CAST 3D)
uint8_t BS_broadcast_address[5] = {0xBA, 0x5E, 0xCA, 0x57, 0x3D};


uint8_t payload_RX[PAYLOAD_MAX_LEN];

uint8_t payload_TX1[PAYLOAD_MAX_LEN];
uint8_t payload_TX2[PAYLOAD_MAX_LEN];
uint8_t payload_TX3[PAYLOAD_MAX_LEN];


uint8_t payload_TX_broadcast[3] = { 0x70, 0xBA, 0x5E};


//Function Prototypes
void AVR_Init(void);
void UART_Init(void);
void UART_Tx(unsigned char data);
void UART_Put_String(char *s);


/************************************************************************************
** AVR_Init function:
** - Resets the Clock Prescaler factor to 1x
** - Start-up delay
** - Initializes the I/O peripherals
** - Plays LED sequence
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

	//Start-up LED sequence loop -> implements short pause after BNO055 Power-On Reset (mandatory)
	for (int i = 6; i != 0; i--)
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

	_delay_ms(100);		// again, short pause to ensure mandatory BNO055 Power-On Reset time
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



void INT6_Init(void)
{
	EICRB &= ~(1 << ISC60) | (1 << ISC61);	//INT6 active when low
	EIMSK |= (1 << INT6);			//Enable INT6
	sei();					//Enable global interrupts
}

ISR(INT6_vect)
{
	cli();					//Disable global interrupt

	nrf_stopListening();

	uint8_t len, pipe;
	nrf_readRXData(payload_RX, &len, &pipe);

	// RX_Payload_cnt = len;
	// received++;
	
	// add some short delay, otherwise strangely the receiver can not receive data
	_delay_us(100);
	
	// Reset status register
	SPI_Write_Byte(STATUS, (1 << RX_DR));
}



void initPackets(uint8_t mode, uint8_t sensorId)
{
	// Packet structure
	//**********************************************************************************************************************************************************************************************************
	// first two bytes of first packet (packet ID = 1) are synchronization bytes
	// packet 1:  2 sync bytes      2 data descriptor bytes    data
	//            0xAB 0xCD         0x.. 0x..                  0x.....................
	// packet 2:  no sync bytes     2 data descriptor bytes    data
	//                              0x.. 0x..                  0x.....................
	// packet 3:  no sync bytes     2 data descriptor bytes    data
	//                              0x.. 0x..                  0x.....................
	//**********************************************************************************************************************************************************************************************************
	
	// Data descriptor structure
	//**********************************************************************************************************************************************************************************************************
	// Node ID						4 bit					former ID1, equals this device's address. Not really required, can be resolved in base station by device address
	// (IMU ID)						3 bit [NOT USED]		former ID2. Which IMU of this device, e.g. thumb or index. Not used, not every IMU sample has an ID field anymore.
	// Device ID					3 bit					is this device a single node, glove v1, or glove v2? May be important, difference is the amount
	//														of different sensors and data alignment in the sent packets. Fixed Node ID could resolve this.
	// Mode							3 bit					quaternion or quaternion + linAcc or maybe acc + rot + mag, Debug info, or similar.
	// Sample ID					2 bit					describes Sample ID of the whole glove at certain time, packet 1, 2, 3 belong together when same Sample ID is the same
	// Packet ID					2 bit					packet 1, 2, or 3. If required it could be extended to packet 4+, but this is not so easy to implement
	
	// num bytes/samples in this packet.					Not really required -> hardcoded
	//**********************************************************************************************************************************************************************************************************
	
	
	// synchronization bytes in first packet
	payload_TX1[0] = 0xAB;
	payload_TX1[1] = 0xCD;
	
	// data descriptor at start of each packet (packet 1 starts after 2 sync bytes)
	payload_TX1[2] = sensorId << 4 | DEVICE_ID;
	payload_TX1[3] = mode << 5 | 0x01;
	
	payload_TX2[0] = sensorId << 4 | DEVICE_ID;
	payload_TX2[1] = mode << 5 | 0x02;
	
	payload_TX3[0] = sensorId << 4 | DEVICE_ID;
	payload_TX3[1] = mode << 5 | 0x03;
}

void updatePacketsSampleID()
{
	// add 1 (i.e. 0x04 = 1 << 2) to previous sample ID and mask out overflow bits
	uint8_t sampleID = (payload_TX1[3] + 0x04) & 0x0C;
	
	payload_TX1[3] = (payload_TX1[3] & 0xF3) | sampleID;
	payload_TX2[1] = (payload_TX2[1] & 0xF3) | sampleID;
	payload_TX3[1] = (payload_TX3[1] & 0xF3) | sampleID;
}


uint8_t modeIsValid(uint8_t mode)
{
	// TODO: enhance (is a bit simplified for now)
	return mode < 2;
}



void process_quat_linAcc()
{
	uint8_t sensorId = 0;
	
	// packet 1    - remember: before first packet's data, there are two sync bytes, so start data at payload_TX1 + 4
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX1 + 4, payload_TX1 + 10);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX1 + 16, payload_TX1 + 22);

	// flush RX to enable packet sending and write data (glove v1: 28 bytes, glove v2: 28 bytes)
	nrf_flushRX();
	nrf_writeAckData(0, payload_TX1, 28);

	
	// packet 2 (glove v1: 26 bytes, glove v2: 32 bytes)
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX2 + 2, payload_TX2 + 8);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX2 + 14, payload_TX2 + 20);
	
#if DEVICE_ID == GLOVE_V1
	nrf_writeAckData(0, payload_TX2, 26);
	
#elif DEVICE_ID == GLOVE_V2
	// split 5th sensor data across TX2 and TX3 packets
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX2 + 26, payload_TX3 + 2);
	nrf_writeAckData(0, payload_TX2, 32);
#endif
	
	
	// packet 3 (glove v1: 26 bytes, glove v2: 32 bytes)
#if DEVICE_ID == GLOVE_V1
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 2, payload_TX3 + 8);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 14, payload_TX3 + 20);
	nrf_writeAckData(0, payload_TX3, 26);
	
#elif DEVICE_ID == GLOVE_V2
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 8, payload_TX3 + 14);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 20, payload_TX3 + 26);
	nrf_writeAckData(0, payload_TX3, 32);
#endif
	
}

void process_quat()
{
	uint8_t sensorId = 0;
	
	// packet 1    - remember: before first packet's data, there are two sync bytes, so start data at payload_TX1 + 4
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 4);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 10);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 16);
#if DEVICE_ID == GLOVE_V2
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 22);
#endif
	
	// flush RX to enable packet sending and write data (glove v1: 22 bytes, glove v2: 28 bytes)
	nrf_flushRX();
#if DEVICE_ID == GLOVE_V1
	nrf_writeAckData(0, payload_TX1, 22);
#elif DEVICE_ID == GLOVE_V2
	nrf_writeAckData(0, payload_TX1, 28);
#endif
	
	// packet 2
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX2 + 2);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX2 + 8);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX2 + 14);
	
	// flush RX to enable packet sending and write data (20 bytes)
	nrf_flushRX();
	nrf_writeAckData(0, payload_TX2, 20);
}


/*
void getDataAddress()
{
	uint8_t rxLen = 0;
	uint8_t rxPipe;
	uint8_t rx, tx_done, max_retry;
	
	// reset timer
	TCNT1 = 0;
	
	// TODO: send random value from BNO to validate base station recognized this device
	
	while(1)
	{
		if (nrf_getIRQStatus(&rx, &tx_done, &max_retry))
		{
			rxLen = 0;
			nrf_resetIRQFlags();
			if (rx)
			{
				// next tx_done will signal current ack payload was transmitted
				// do not flush TX until ack packet was sent (at least 500 us or so)
				nrf_stopListening();

				while (nrf_dataAvailable())
				{
					nrf_readRXData(payload_RX, &rxLen, &rxPipe);
				}
				
				nrf_startListening();
				
				// check payload and change mode if required
				uint8_t newMode = payload_RX[0];
				if (newMode != mode && modeIsValid(newMode))
				{
					mode = newMode;
					initPackets(mode);
				}
			}
			if (tx_done)
			{
				// last ack packet was received by PTX
			}
		}
}
*/

/************************************************************************************
** Main function:
** - Contains an endless loop
** - Sets the BNO055 in NDOF mode and fetches the quaternion data
*************************************************************************************/
int main(void)
{
	// Disable global interrupt
	cli();
	
	// initialization
	AVR_Init();
	i2c_init();
	UART_Init();
	SPI_Init();
	nrf_init(0x69, DR_1M, NRF_ADDR_LEN, 1);
	
	// could also just open a dynamic RX pipe, but this way we also have the TX address set and RX pipe will be opened anyway
	nrf_openDynamicTXPipe(BS_broadcast_address, 1, 0);
	//nrf_openDynamicTXPipe(BS_address, 1, 0);
	
	//INT6_Init();
	
	// finally initialize BNO (needs power-on reset time + takes a lot of setup time)
	BNO_Init();
	
	
	
	// nrf setup: Configure as receiver
	nrf_setModeRX();
	nrf_maskIRQ(1, 1, 1);
	
	
	
	uint8_t rxLen = 0;
	uint8_t rxPipe;
	uint8_t rx, tx_done, max_retry;
	
	
	uint8_t sensorId = 0;
	uint8_t sessionId = 0;
	
	// operation mode
	// default: quaternion only, mode = 1 -> quaternion + lin. acceleration
	uint8_t mode = 0;
	initPackets(mode, sensorId);
	
	
	
	PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
	PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
	_delay_ms(200);
	
	uint8_t doReceive = 1;
	
	nrf_flushAll();
	nrf_resetIRQFlags();
	
	nrf_writeAckData(0, payload_TX_broadcast, 3);
	
	nrf_startListening();
	_delay_us(150);
	
	
	// receive this sensor node's address (which equals its ID)
	while (1)
	{
		rxLen = 0;
		if (nrf_getIRQStatus(&rx, &tx_done, &max_retry))
		{
			nrf_resetIRQFlags();
			if (rx && doReceive)
			{

				while (nrf_dataAvailable())
				{
					nrf_readRXData(payload_RX, &rxLen, &rxPipe);
				}
				
				// check payload and change mode if required
				uint8_t newMode = payload_RX[0];
				if (newMode != mode && modeIsValid(newMode))
				{
					mode = newMode;
					initPackets(mode, sensorId);
				}
				
				sensorId = payload_RX[1];
				sessionId = payload_RX[2];
				
				doReceive = 0;
				
			}
			if (tx_done)
			{
				// last ack packet was received by PTX
				nrf_stopListening();
				break;
			}
		}
	}
	
	
	initPackets(mode, sensorId);
	
	
	PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
	_delay_ms(500);
	
	for (int i = 0; i < sensorId + 1; ++i)
	{
		PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
		_delay_ms(200);
		PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
		_delay_ms(200);
	}
	
	PORTC &= ~_BV(6);	//Turns OFF LED in Port C pin 6
	
	
	
	
	BS_data_address[4] = sensorId;
	nrf_openDynamicTXPipe(BS_data_address, 1, 0);
	
	nrf_flushAll();
	
	nrf_startListening();
	
	// timer
	TCCR1B |= _BV(CS11);
	
	// reset timer
	TCNT1 = 0;
	
	
	
	// Endless Loop
	while (1)
	{
		
		if (mode == 1)
		{
			// process quaternions + linear acceleration
			process_quat_linAcc();
		}
		else
		{
			// default: only process quaternions
			process_quat();
		}
		
		// TODO: maybe just delay the amount of us left (10000 - TCNT1), ensure TCNT1 < 10000
		// wait until 10 ms have passed (to keep sampling time of ~100 Hz)
		while (TCNT1 < 10000)
		{
			_delay_us(1);
		}
		
		// increase sample ID to indicate next sample is processed and sent
		updatePacketsSampleID();
		
		// reset timer
		TCNT1 = 0;
		
		rxLen = 0;
		if (nrf_getIRQStatus(&rx, &tx_done, &max_retry))
		{
			nrf_resetIRQFlags();
			if (rx)
			{
				// next tx_done will signal current ack payload was transmitted
				// do not flush TX until ack packet was sent (at least 500 us or so)
				// nrf_stopListening();

				while (nrf_dataAvailable())
				{
					nrf_readRXData(payload_RX, &rxLen, &rxPipe);
				}
				
				// nrf_startListening();
				
				// check payload and change mode if required
				uint8_t newMode = payload_RX[0];
				if (newMode != mode && modeIsValid(newMode))
				{
					mode = newMode;
					initPackets(mode, sensorId);
				}
			}
			if (tx_done)
			{
				// last ack packet was received by PTX
			}
		}
	}
}