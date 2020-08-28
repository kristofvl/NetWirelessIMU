/*
 * SPI.h
 *
 * Created: 27.03.2020 00:03:41
 *  Author: Jochen
 */ 


#ifndef SPI_H_
#define SPI_H_


#define MOSI		2
#define MISO		3
#define SCLK		1
#define CSN			0
#define CE			5

void SPI_Init();
unsigned char SPI_Write(unsigned char data);




#endif /* SPI_H_ */