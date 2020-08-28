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

#define TRANSMISSON_OK	0
#define MESSAGE_LOST	1

#define NODE_ID		0x02
#define IMU_ID		0x01
#define MAX_IMU_COUNT	6


#endif /* CONFIG_H_ */