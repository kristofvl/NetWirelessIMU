/*
 * MAX17043.h
 *
 * Created: 24-May-18 8:42:04 AM
 *  Author: Frederic Philips
 */


#ifndef _MAX17043_H_
#define _MAX17043_H_

//MAX17043 TWI Address
//0x36 - 00110110
//(0x36 << 1) - 01101100 - 0x6C
#define MAX17043_ADDR		(0x36 << 1)

//MAX17043 Resolution
#define MAX17043_VCELL_RES	0.00125f
#define MAX17043_SOC_RES	256.0f

//MAX17043 Register Map
#define MAX17043_VCELL		0x02
#define MAX17043_SOC		0x04
#define MAX17043_MODE		0x06
#define MAX17043_IC_VER		0x08
#define MAX17043_CONFIG 	0x0C
#define MAX17043_COMMAND	0xFE

//MAX17043 mnemonics
#define MAX17043_POR_L		0x54
#define MAX17043_POR_H		0x00
#define MAX17043_QS_H		0x40
#define MAX17043_QS_L		0x00
#define MAX17043_RCOMP		0x97

//MAX17043 Alert Threshold
#define MAX17043_ALT_30		0b00010
#define MAX17043_ALT_20		0b01100
#define MAX17043_ALT_10		0b10110

//Store TWI data
uint8_t TWI_data;

#endif /* _MAX17043_H_ */