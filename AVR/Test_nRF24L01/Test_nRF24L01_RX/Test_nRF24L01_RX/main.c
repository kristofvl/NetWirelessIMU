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
#define SPI_PORT	PORTB
#define SPI_DDR		DDRB
#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN		0
#define CE		4

//nRF control macros
#define CE_LOW()	SPI_PORT &= ~(_BV(CE))
#define CE_HIGH()	SPI_PORT |= _BV(CE)
#define CSN_LOW()	SPI_PORT &= ~(_BV(CSN))
#define CSN_HIGH()	SPI_PORT |= _BV(CSN)

void AVR_Init(void);
void SPI_Init(void);
uint8_t SPI_Write_Byte(uint8_t byte);
void UART_Init(void);
void nRF24L01_Init(void);
void nRF24L01_Config(void);
void nRF24L01_DR_Setup(uint8_t data_rate);
void nRF24L01_OP_Power(uint8_t op_power);
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

** When nRF24L01 is in power down mode it must settle for 1.5ms 
** before it can enter the TX or RX modes. 

** If an external clock is used this delay is reduced to 150µs.

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
	
	_delay_us(10);	//Delay for CE settling time
}

void nRF24L01_Config(void)
{
	/************************************************************************/
	/*                             RF Setup                                 */
	/************************************************************************/
	
	//Set the RF channel to 105 i.e. @2.505 GHz
	nRF24L01_Write_Reg(NRF24L01_REG_RF_CH, NRF24L01_CHANNEL);
	
	//Set the data rate to 2 MBPS
	nRF24L01_DR_Setup(NRF24L01_DR_2_MBPS);
	
	//Set the output power to 0dBM
	nRF24L01_OP_Power(NRF24L01_OP_POWER_POS_0dBM);
	
	//LNA setting is unavailable in nRF24L01+
	
	/************************************************************************/
	/*                          Payload config                              */
	/************************************************************************/
	
	//Enable data-pipe 0 with a payload length of 1 byte
	nRF24L01_Write_Reg(NRF24L01_REG_RX_PW_P0, NRF24L01_PAYLOAD_LEN);
	
	//Disable dynamic payload
	nRF24L01_Write_Reg(NRF24L01_REG_DYNPD, 0x00);
	
	/************************************************************************/
	/*                          Address config                              */
	/************************************************************************/
	
	//Address width set to 5 bytes
	nRF24L01_Write_Reg(NRF24L01_REG_SETUP_AW, NRF24L01_ADDRESS_WIDTH);
	
	//Set the 5-byte receiver address from RX_address array
	nRF24L01_Write_Regs(NRF24L01_REG_RX_ADDR_P0, RX_address, 5);
	
	//Set the 5-byte transmitter address from TX_address array
	nRF24L01_Write_Regs(NRF24L01_REG_TX_ADDR, TX_address, 5);
	
	/************************************************************************/
	/*			   Data verification                            */
	/************************************************************************/
	
	//Enable auto-acknowledgment for the data-pipe in use(data-pipe 0)
	nRF24L01_Write_Reg(NRF24L01_REG_EN_AA, _BV(NRF24L01_REG_ENAA_P0));
	
	//Disable auto-retry
	nRF24L01_Write_Reg(NRF24L01_REG_SETUP_RETR, NRF24L01_RETRY_COUNT);
	
	//Enable CRC and set the length to 1 byte
	
	
	/************************************************************************/
	/*			   Transceiver mode                             */
	/************************************************************************/
	
}

void nRF24L01_DR_Setup(uint8_t data_rate)
{
	uint8_t nRF_DR_value = nRF24L01_Read_Reg(NRF24L01_REG_RF_SETUP);
	switch(data_rate)
	{
		//250 KBPS
		case NRF24L01_DR_250_KBPS:
			nRF_DR_value &= ~(_BV(NRF24L01_REG_RF_DR_HIGH));
			nRF_DR_value |= _BV(NRF24L01_REG_RF_DR_LOW);
			break;
		
		//1 MBPS
		case NRF24L01_DR_1_MBPS:
			nRF_DR_value &= ~(_BV(NRF24L01_REG_RF_DR_HIGH));
			nRF_DR_value &= ~(_BV(NRF24L01_REG_RF_DR_LOW));
			break;
		
		//2 MBPS
		case NRF24L01_DR_2_MBPS:
			nRF_DR_value |= _BV(NRF24L01_REG_RF_DR_HIGH);
			nRF_DR_value &= ~(_BV(NRF24L01_REG_RF_DR_LOW));
			break;
	}
	
	nRF24L01_Write_Reg(NRF24L01_REG_RF_SETUP, nRF_DR_value);
}

void nRF24L01_OP_Power(uint8_t op_power)
{
	uint8_t nRF_power_value = nRF24L01_Read_Reg(NRF24L01_REG_RF_SETUP);
	switch(op_power)
	{
		//-18dBM
		case NRF24L01_OP_POWER_NEG_18dBM:
			nRF_power_value &= ~(_BV(NRF24L01_REG_RF_PWR_0));
			nRF_power_value &= ~(_BV(NRF24L01_REG_RF_PWR_1));
			break;
		
		//-12dBM
		case NRF24L01_OP_POWER_NEG_12dBM:
			nRF_power_value |= _BV(NRF24L01_REG_RF_PWR_0);
			nRF_power_value &= ~(_BV(NRF24L01_REG_RF_PWR_1));
			break;
		
		//-6dBM
		case NRF24L01_OP_POWER_NEG_6dBM:
			nRF_power_value &= ~(_BV(NRF24L01_REG_RF_PWR_0));
			nRF_power_value |= _BV(NRF24L01_REG_RF_PWR_1);
			break;
			
		//+0dBM
		case NRF24L01_OP_POWER_POS_0dBM:
			nRF_power_value |= _BV(NRF24L01_REG_RF_PWR_0);
			nRF_power_value |= _BV(NRF24L01_REG_RF_PWR_1);
			break;		
	}
	
	nRF24L01_Write_Reg(NRF24L01_REG_RF_SETUP, nRF_power_value);	
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
	
	//Set write access to register
	SPI_Write_Byte(NRF24L01_CMD_W_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	//Write data to register		
	SPI_Write_Byte(data);				
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;	
}

void nRF24L01_Write_Regs(uint8_t reg, uint8_t *data, uint8_t len)
{
	//Reset the write byte count
	int8_t wByte_cnt = 0;				
	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	//Set write access to register
	SPI_Write_Byte(NRF24L01_CMD_W_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));

	for(wByte_cnt = 0; wByte_cnt < len; wByte_cnt++)
	{
		//Write data to register
		SPI_Write_Byte(data[wByte_cnt]);
	}
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;
}

uint8_t nRF24L01_Read_Reg(uint8_t reg)
{	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	//Set read access to register
	SPI_Write_Byte(NRF24L01_CMD_R_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	//Send NOP byte to read data from register
	uint8_t reg_data = SPI_Write_Byte(NRF24L01_CMD_NOP);	
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;	
	
	//Return the read data
	return reg_data;					
}

void nRF24L01_Read_Regs(uint8_t reg, uint8_t *data, uint8_t len)
{
	//Reset the read byte count
	uint8_t rByte_cnt = 0;				
	
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	//Set read access to register
	SPI_Write_Byte(NRF24L01_CMD_R_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	
	for(rByte_cnt = 0; rByte_cnt < len; rByte_cnt++)
	{
		//Send NOP byte to read data from register
		uint8_t data[rByte_cnt] = SPI_Write_Byte(NRF24L01_CMD_NOP);
	}
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;
}
/***********************************************************
***********************************************************/

void nRF24L01_Payload_RX()
{
	//Pull CSN low to start SPI communication
	CSN_LOW;
	
	
	
	//Pull CSN high to stop SPI communication
	CSN_HIGH;	
}

int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
		
    }
}

