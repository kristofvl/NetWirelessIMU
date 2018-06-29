/*
 * Test_nRF24L01_RX.h
 *
 * Created: 07-Jun-18 10:16:39 AM
 *  Author: Frederic Philips
 */ 


#ifndef TEST_NRF24L01_RX_H_
#define TEST_NRF24L01_RX_H_

/************************************************************************/
/*                           nRF24L01 setup                             */
/************************************************************************/	
//Transmission channel(0 to 124)
#define NRF24L01_CHANNEL		105	//Channel 105 i.e. @2.505 GHz

//Data rate
#define NRF24L01_DR_250_KBPS		0	//250 KBPS
#define NRF24L01_DR_1_MBPS		1	//1 MBPS
#define NRF24L01_DR_2_MBPS		2	//2 MBPS

//Output power
#define NRF24L01_OP_POWER_NEG_18dBM	0	//-18dBM
#define NRF24L01_OP_POWER_NEG_12dBM	1	//-12dBM
#define NRF24L01_OP_POWER_NEG_6dBM	2	//-6dBM
#define NRF24L01_OP_POWER_POS_0dBM	3	//+0dBM

//Payload length(1 - 32 bytes)
#define NRF24L01_PAYLOAD_LEN		0x01	//1 byte

//Address width
#define NRF24L01_ADDRESS_WIDTH		0x03	//5 bytes

//Retry count
#define NRF24L01_RETRY_COUNT		0x00	//Retry disabled

//nRF24L01 register map
#define NRF24L01_REG_CONFIG		0x00
#define NRF24L01_REG_EN_AA		0x01
#define NRF24L01_REG_EN_RXADDR		0x02
#define NRF24L01_REG_SETUP_AW		0x03
#define NRF24L01_REG_SETUP_RETR		0x04
#define NRF24L01_REG_RF_CH		0x05
#define NRF24L01_REG_RF_SETUP		0x06
#define NRF24L01_REG_STATUS		0x07
#define NRF24L01_REG_OBSERVE_TX		0x08
#define NRF24L01_REG_CD			0x09
#define NRF24L01_REG_RX_ADDR_P0		0x0A
#define NRF24L01_REG_RX_ADDR_P1		0x0B
#define NRF24L01_REG_RX_ADDR_P2		0x0C
#define NRF24L01_REG_RX_ADDR_P3		0x0D
#define NRF24L01_REG_RX_ADDR_P4		0x0E
#define NRF24L01_REG_RX_ADDR_P5		0x0F
#define NRF24L01_REG_TX_ADDR		0x10
#define NRF24L01_REG_RX_PW_P0		0x11
#define NRF24L01_REG_RX_PW_P1		0x12
#define NRF24L01_REG_RX_PW_P2		0x13
#define NRF24L01_REG_RX_PW_P3		0x14
#define NRF24L01_REG_RX_PW_P4		0x15
#define NRF24L01_REG_RX_PW_P5		0x16
#define NRF24L01_REG_FIFO_STATUS	0x17
#define NRF24L01_REG_FEATURE		0x1D
#define NRF24L01_REG_DYNPD		0x1C

//Bit Mnemonics
#define NRF24L01_REG_MASK_RX_DR		6
#define NRF24L01_REG_MASK_TX_DS		5
#define NRF24L01_REG_MASK_MAX_RT	4
#define NRF24L01_REG_EN_CRC		3
#define NRF24L01_REG_CRCO		2
#define NRF24L01_REG_PWR_UP		1
#define NRF24L01_REG_PRIM_RX		0
#define NRF24L01_REG_ENAA_P5		5
#define NRF24L01_REG_ENAA_P4		4
#define NRF24L01_REG_ENAA_P3		3
#define NRF24L01_REG_ENAA_P2		2
#define NRF24L01_REG_ENAA_P1		1
#define NRF24L01_REG_ENAA_P0		0
#define NRF24L01_REG_ERX_P5		5
#define NRF24L01_REG_ERX_P4		4
#define NRF24L01_REG_ERX_P3		3
#define NRF24L01_REG_ERX_P2		2
#define NRF24L01_REG_ERX_P1		1
#define NRF24L01_REG_ERX_P0		0
#define NRF24L01_REG_AW			0
#define NRF24L01_REG_ARD		4
#define NRF24L01_REG_ARC		0
#define NRF24L01_REG_CONT_WAVE		7
#define NRF24L01_REG_RF_DR_LOW		5
#define NRF24L01_REG_PLL_LOCK		4
#define NRF24L01_REG_RF_DR_HIGH		3
#define NRF24L01_REG_RF_PWR_1		2
#define NRF24L01_REG_RF_PWR_0		1
#define NRF24L01_REG_LNA_HCURR		0
#define NRF24L01_REG_RX_DR		6
#define NRF24L01_REG_TX_DS		5
#define NRF24L01_REG_MAX_RT		4
#define NRF24L01_REG_RX_P_NO		1
#define NRF24L01_REG_TX_FULL		0
#define NRF24L01_REG_PLOS_CNT		4
#define NRF24L01_REG_ARC_CNT		0
#define NRF24L01_REG_TX_REUSE		6
#define NRF24L01_REG_FIFO_FULL		5
#define NRF24L01_REG_TX_EMPTY		4
#define NRF24L01_REG_RX_FULL		1
#define NRF24L01_REG_RX_EMPTY		0
#define NRF24L01_REG_RPD		9
#define NRF24L01_REG_RF_PWR_LOW		1
#define NRF24L01_REG_RF_PWR_HIGH	2

//nRF24L01 commands
#define NRF24L01_CMD_R_REGISTER		0x00
#define NRF24L01_CMD_W_REGISTER		0x20
#define NRF24L01_CMD_REGISTER_MASK	0x1F
#define NRF24L01_CMD_R_RX_PAYLOAD	0x61
#define NRF24L01_CMD_W_TX_PAYLOAD	0xA0
#define NRF24L01_CMD_FLUSH_TX		0xE1
#define NRF24L01_CMD_FLUSH_RX		0xE2
#define NRF24L01_CMD_REUSE_TX_PL	0xE3
#define NRF24L01_CMD_NOP		0xFF

/*
//CE and CSN port definitions
#define NRF24L01_DDR DDRB
#define NRF24L01_PORT PORTB
#define NRF24L01_CE PB0
#define NRF24L01_CSN PB1


//CE and CSN functions
#define nrf24l01_CSNhi NRF24L01_PORT |= (1<<NRF24L01_CSN);
#define nrf24l01_CSNlo NRF24L01_PORT &= ~(1<<NRF24L01_CSN);
#define nrf24l01_CEhi NRF24L01_PORT |=  (1<<NRF24L01_CE);
#define nrf24l01_CElo NRF24L01_PORT &= ~(1<<NRF24L01_CE);
*/

//crc setup
#define NRF24L01_RF24_CRC_DISABLED 1
#define NRF24L01_RF24_CRC_8 2
#define NRF24L01_RF24_CRC_16 3
#define NRF24L01_RF24_CRC NRF24L01_RF24_CRC_16





//auto ack enabled
#define NRF24L01_ACK 1

//auto retransmit delay and count
#define NRF24L01_RETR (0b0100 << NRF24L01_REG_ARD) | (0b0111 << NRF24L01_REG_ARC) //1500uS, 15 times

//enable / disable pipe
#define NRF24L01_ENABLEDP0 1 //pipe 0
#define NRF24L01_ENABLEDP1 1 //pipe 1
#define NRF24L01_ENABLEDP2 1 //pipe 2
#define NRF24L01_ENABLEDP3 1 //pipe 3
#define NRF24L01_ENABLEDP4 1 //pipe 4
#define NRF24L01_ENABLEDP5 1 //pipe 5

//address size
#define NRF24L01_ADDRSIZE 5



#endif /* TEST_NRF24L01_RX_H_ */