/*
 * Test_nRF24L01_RX.c
 *
 * Created: 07-Jun-18 10:16:04 AM
 * Author : Frederic Philips
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#define F_CPU 16000000UL //16 MHz frequency
#define BAUD  9600
#include <util/setbaud.h>
#include <util/delay.h>

//SPI macros
#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define MOSI	 2
#define MISO	 3
#define SCLK	 1
#define CSN      0
#define CE		 4

//nRF control macros
#define CE_LOW()		SPI_PORT &= ~(_BV(CE))
#define CE_HIGH()		SPI_PORT |= _BV(CE)
#define CSN_LOW()		SPI_PORT &= ~(_BV(CSN))
#define CSN_HIGH()		SPI_PORT |= _BV(CSN)

//nRF read/write macros
#define NRF24L01_READ	0x00
#define NRF24L01_WRITE	0x20
#define NRF24L01_NOP	0x00

void AVR_Init(void);
void SPI_Init(void);
uint8_t SPI_Write_Byte(uint8_t byte);
void UART_Init(void);
void nRF24L01_Init(void);
void nRF24L01_Config(void);
void nRF24L01_Write_Reg(uint8_t reg, uint8_t data);
void nRF24L01_Write_Regs(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t nRF24L01_Read_Reg(uint8_t reg);
void nRF24L01_Read_Regs(uint8_t reg, uint8_t *data, uint8_t len);
void nRF24L01_Payload_RX();
void UART_Tx(unsigned char data);

//Define 5-byte unique addresses for TX and RX
uint8_t TX_address[5] = {0x55, 0x55, 0x55, 0x55, 0x55};
uint8_t RX_address[5] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
	
//Define arrays for TX/RX payloads(Max. 32 bytes each)
uint8_t TX_payload[32];
uint8_t RX_payload[32];
	
/* nRF24L01 max. clock speed = 8MHz
** Note: From ATMega32U4 datasheet:
** To ensure correct sampling of the clock signal, the frequency 
** of the SPI clock should never exceed F_CPU/4.
*/
void SPI_Init(void)
{
	//Initialize the SPI hardware pins
	SPI_DDR |= _BV(MOSI) | _BV(SCLK);
	SPI_DDR &= ~(_BV(MISO));
	
	//Enable SPI in Master mode with data-rate @F_CPU/4
	SPCR |= _BV(SPE) | _BV(MSTR);
}

/*
** Note: 
** nRF requires typically 5.3ms settling time after POR.
** When nRF24L01 is in power down mode it must settle for 1.5ms before it can enter the TX or RX modes. 
** If an external clock is used this delay is reduced to 150?s.
** CE should be held low/high for a minimum of 10µs.
** The settling time must be controlled by the MCU.
*/
void nRF24L01_Init(void)
{
	//Initialize the nRF control pins
	SPI_DDR |= _BV(CSN) | _BV(CE);
	
	//Set CE low and CSN high
	CE_LOW();
	CSN_HIGH();
	
	_delay_us(10); //Delay for CE settling time
}

void nRF24L01_Config(void)
{
	
}

//SPI write one byte of data and return the read byte(cyclic)
uint8_t SPI_Write_Byte(uint8_t byte)
{
	//Load byte in SPI data register
	SPDR = byte;
	
	//Wait for transmission to complete
	while (!(SPSR & (1 << SPIF)));
	
	//Return the received data
	return SPDR;
}

/***********************************************************
***********************************************************/
void nRF24L01_Write_Reg(uint8_t reg, uint8_t data)
{	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	SPI_Write_Byte(NRF24L01_WRITE + reg); //Set write access to register
	SPI_Write_Byte(data);				  //Write data to register
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;	
}

void nRF24L01_Write_Regs(uint8_t reg, uint8_t *data, uint8_t len)
{
	int8_t wByte_cnt = 0;				  //Reset the write byte count
	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	SPI_Write_Byte(NRF24L01_WRITE + reg); //Set write access to register

	for(wByte_cnt = 0; wByte_cnt < len; wByte_cnt++)
	{
		SPI_Write_Byte(data[wByte_cnt]);  //Write data to register
	}
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;
}

uint8_t nRF24L01_Read_Reg(uint8_t reg)
{	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	SPI_Write_Byte(NRF24L01_READ + reg);		 //Set read access to register
	uint8_t data = SPI_Write_Byte(NRF24L01_NOP); //Read data from register
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;	
	
	return data;								 //Return the read data
}

void nRF24L01_Read_Regs(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t rByte_cnt = 0; //Reset the read byte count
	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	SPI_Write_Byte(NRF24L01_READ + reg);				//Set read access to register
	
	for(rByte_cnt = 0; rByte_cnt < len; rByte_cnt++)
	{
		data[rByte_cnt] = SPI_Write_Byte(NRF24L01_NOP); //Read data from register
	}
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH; //high CSN
}
/***********************************************************
***********************************************************/

int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
		
    }
}

