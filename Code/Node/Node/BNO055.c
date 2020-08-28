/*
 * BNO055.c
 *
 * Created: 28.03.2020 08:31:10
 *  Author: Jochen
 */ 

#include "config.h"
#include <util/delay.h>

#include "BNO055.h"
#include "i2cmaster.h"


uint8_t Chip_ID;

void BNO_Init(void)
{
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_CHIP_ID_ADDR);			//Access the Chip ID register
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	Chip_ID = i2c_readNak();			//Should read 0xA0
	i2c_stop();
/*
	#if UART_DEBUG == 1
		if(Chip_ID != BNO055_CHIP_ID)
		{
			UART_Put_String("BNO055 not detected!\n");
		}

		else
		{
			UART_Put_String("BNO055 successfully connected\n");
			UART_Tx(Chip_ID);
		}
	#endif
*/
	//Reset BNO055
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_SYS_TRIGGER_ADDR);
	i2c_write(BNO055_RESET);			//Set operation mode to IMU
	i2c_stop();

	_delay_ms(750);					//Pause after reset
/*
	//Set clock source as external for BNO055
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_SYS_TRIGGER_ADDR);
	i2c_write(BNO055_EXT_CLK);			//Set operation mode to IMU
	i2c_stop();

	_delay_ms(100);					//Pause after setting clock source
*/
	//Set operating mode
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_OPR_MODE_ADDR);
	i2c_write(OPERATION_MODE_NDOF);			//Set operation mode to IMU
	i2c_stop();

	// Pause after setting operating mode
	_delay_ms(100);
}

void BNO_Read_Quaternion(uint8_t* buffer)
{
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_QUATERNION_DATA_W_LSB_ADDR);	//Access LSB of Quaternion W data
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	buffer[0] = i2c_readAck();		//Read Quaternion_W LSB
	buffer[1] = i2c_readAck();		//Read Quaternion_W MSB
	buffer[2] = i2c_readAck();		//Read Quaternion_X LSB
	buffer[3] = i2c_readAck();		//Read Quaternion_X MSB
	buffer[4] = i2c_readAck();		//Read Quaternion_Y LSB
	buffer[5] = i2c_readAck();		//Read Quaternion_Y MSB
	buffer[6] = i2c_readAck();		//Read Quaternion_Z LSB
	buffer[7] = i2c_readNak();		//Read Quaternion_Z MSB
	i2c_stop();
}

void BNO_Read_Quaternion_LinAcc(uint8_t* buffer)
{
	// start reading from BNO
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_QUATERNION_DATA_W_LSB_ADDR);	//Access LSB of Quaternion W data
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	buffer[0] = i2c_readAck();		//Read Quaternion_W LSB
	buffer[1] = i2c_readAck();		//Read Quaternion_W MSB
	buffer[2] = i2c_readAck();		//Read Quaternion_X LSB
	buffer[3] = i2c_readAck();		//Read Quaternion_X MSB
	buffer[4] = i2c_readAck();		//Read Quaternion_Y LSB
	buffer[5] = i2c_readAck();		//Read Quaternion_Y MSB
	buffer[6] = i2c_readAck();		//Read Quaternion_Z LSB
	buffer[7] = i2c_readAck();		//Read Quaternion_Z MSB
	
	buffer[8] = i2c_readAck();		//Read lin_acc_X LSB
	buffer[9] = i2c_readAck();		//Read lin_acc_X MSB
	buffer[10] = i2c_readAck();		//Read lin_acc_Y LSB
	buffer[11] = i2c_readAck();		//Read lin_acc_Y MSB
	buffer[12] = i2c_readAck();		//Read lin_acc_Z LSB
	buffer[13] = i2c_readNak();		//Read lin_acc_Z MSB
	i2c_stop();
}


void BNO_Read_Acc_Mag_Gyr(uint8_t* buffer)
{
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_ACCEL_DATA_X_LSB_ADDR);	//Access LSB of Accelerometer X data
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	buffer[2] = i2c_readAck();		//Read Accelerometer_X LSB
	buffer[3] = i2c_readAck();		//Read Accelerometer_X MSB
	buffer[4] = i2c_readAck();		//Read Accelerometer_Y LSB
	buffer[5] = i2c_readAck();		//Read Accelerometer_Y MSB
	buffer[6] = i2c_readAck();		//Read Accelerometer_Z LSB
	buffer[7] = i2c_readAck();		//Read Accelerometer_Z MSB
	buffer[8] = i2c_readAck();		//Read Magnetometer_X LSB
	buffer[9] = i2c_readAck();		//Read Magnetometer_X MSB
	buffer[10] = i2c_readAck();		//Read Magnetometer_Y LSB
	buffer[11] = i2c_readAck();		//Read Magnetometer_Y MSB
	buffer[12] = i2c_readAck();		//Read Magnetometer_Z LSB
	buffer[13] = i2c_readAck();		//Read Magnetometer_Z MSB
	buffer[14] = i2c_readAck();		//Read Gyroscope_X LSB
	buffer[15] = i2c_readAck();		//Read Gyroscope_X MSB
	buffer[16] = i2c_readAck();		//Read Gyroscope_Y LSB
	buffer[17] = i2c_readAck();		//Read Gyroscope_Y MSB
	buffer[18] = i2c_readAck();		//Read Gyroscope_Z LSB
	buffer[19] = i2c_readNak();		//Read Gyroscope_Z MSB
	i2c_stop();
//	_delay_ms(5);
}