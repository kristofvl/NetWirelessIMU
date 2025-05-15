/*
 * config.h
 *
 * Created: 27.03.2020 01:38:46
 *  Author: Jochen
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

#define F_CPU 8000000UL	//8MHz frequency
#define BAUD  9600

#define NRF_ADDR_LEN		5
#define PAYLOAD_MAX_LEN		32
#define PAYLOAD_LEN			20
#define PAYLOAD_QUAT_LEN	10


#define GLOVE_V1			0x01
#define GLOVE_V2			0x02


#define NODE_ID				0x01			// must be unique for each node/device
#define DEVICE_ID			GLOVE_V1		// this device's type/version: 0x00 -> standard node; 0x01 -> glove v1; 0x02 -> glove v2


#if DEVICE_ID == GLOVE_V1
  #define MAX_IMU_COUNT		6
#elif DEVICE_ID == GLOVE_V2
  #define MAX_IMU_COUNT		7
#else
  #define MAX_IMU_COUNT		0
#endif

#endif /* CONFIG_H_ */