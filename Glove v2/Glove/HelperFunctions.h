#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include <stdint.h>



void EEPROM_write(unsigned int uiAddress, unsigned char ucData);

unsigned char EEPROM_read(unsigned int uiAddress);



#endif // HELPERFUNCTIONS_H_