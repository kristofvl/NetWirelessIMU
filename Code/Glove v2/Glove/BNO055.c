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




void show_err()
{
	_delay_ms(200);		//0.2 second delay
	PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
	_delay_ms(750);		//0.75 second delay
	PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
}

void show_succ()
{
	_delay_ms(200);		//0.2 second delay
	PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
	_delay_ms(200);		//0.2 second delay
	PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
}


uint8_t available_mask;

uint8_t BNO_is_available(uint8_t id)
{
	return (available_mask >> id) & 0x01;
}


void BNO_Init(void)
{
	PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
	
	unsigned char i2c_err = 0;
	available_mask = 0;
	
	
	for (int i = 0; i < MAX_IMU_COUNT; ++i)
	{
		uint8_t chip_ID = 0;
		
		// select sensor
		BNO_MUX_Select(i);
		PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
		i2c_err = i2c_start_wait_for(BNO055_ADDRESS + I2C_WRITE, 10);	//Set device address and read mode
		if (i2c_err)
		{
			show_err();
			continue;
		}
		PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
		
		i2c_write(BNO055_CHIP_ID_ADDR);			//Access the Chip ID register
		i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
		chip_ID = i2c_readNak();			//Should read 0xA0
		i2c_stop();
		
		// could not read from BNO
		if (chip_ID != 0xA0)
		{
			show_err();
			continue;
		}
		
		//Reset BNO055
		i2c_err = i2c_start_wait_for(BNO055_ADDRESS + I2C_WRITE, 10);	//Set device address and write mode
		if (i2c_err)
		{
			show_err();
			continue;
		}
		i2c_write(BNO055_SYS_TRIGGER_ADDR);
		i2c_write(BNO055_RESET);			//Set operation mode to IMU
		i2c_stop();
		
		available_mask |= 1 << i;
		
		show_succ();
	}

	// Pause after reset
	_delay_ms(750);
	
	/*
	//Set clock source as external for BNO055
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and read mode
	i2c_write(BNO055_SYS_TRIGGER_ADDR);
	i2c_write(BNO055_EXT_CLK);			//Set operation mode to IMU
	i2c_stop();

	_delay_ms(100);					//Pause after setting clock source
	*/

	for (int i = 0; i < MAX_IMU_COUNT; ++i)
	{
		if (!BNO_is_available(i))
		{
			continue;
		}
		// select sensor
		BNO_MUX_Select(i);
		
		
		// TODO(JK): ensure fingers are treated the same for glove v1 and v2 (should be i = [1,2,3,4,5])
		// set axis mapping for fingers
		if (i > 0 && i < 6)
		{
			// fingers are rotated by 90� around z-axis
			i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
			i2c_write(BNO055_AXIS_MAP_CONFIG_ADDR);
			i2c_write(0x21); // z -> z, y -> x, x -> -y
			i2c_stop();
			
			// pause after changing axes
			_delay_ms(10);
			
			// adjust sign (y -> -y)
			i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
			i2c_write(BNO055_AXIS_MAP_SIGN_ADDR);
			i2c_write(0x04);
			i2c_stop();
			
			// pause after changing axes and sign
			_delay_ms(10);
		}
		
		//Set operating mode
		i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
		i2c_write(BNO055_OPR_MODE_ADDR);
		i2c_write(OPERATION_MODE_NDOF);			//Set operation mode to IMU
		i2c_stop();
	}
	
	// Pause after setting operating mode
	_delay_ms(100);
	
	PORTC &= ~(_BV(7));	//Turns OFF LED in Port C pin 7
}

void BNO_Read_Quaternion(uint8_t id, uint8_t* buffer)
{
	if (!BNO_is_available(id))
	{
		return;
	}
	// select sensor device (ensure id <= MAX_IMU_COUNT, not checked here for performance reasons)
	BNO_MUX_Select(id);
	
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
	buffer[7] = i2c_readNak();		//Read Quaternion_Z MSB
	i2c_stop();
}





void BNO_Read_Quaternion_Compressed(uint8_t id, uint8_t* buffer)
{
	if (!BNO_is_available(id))
	{
		return;
	}
	// select sensor device (ensure id <= 6, not checked here for performance reasons)
	BNO_MUX_Select(id);
	
	union QuatBuffer buf;
	
	// start reading from BNO
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_QUATERNION_DATA_W_LSB_ADDR);	//Access LSB of Quaternion W data
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	buf.buffer[0] = i2c_readAck();		//Read Quaternion_W LSB
	buf.buffer[1] = i2c_readAck();		//Read Quaternion_W MSB
	
	// check if w negative
	if (buf.vec[0] < 0)
	{
		buf.buffer[2] = i2c_readAck();		//Read Quaternion_X LSB
		buf.buffer[3] = i2c_readAck();		//Read Quaternion_X MSB
		buf.buffer[4] = i2c_readAck();		//Read Quaternion_Y LSB
		buf.buffer[5] = i2c_readAck();		//Read Quaternion_Y MSB
		buf.buffer[6] = i2c_readAck();		//Read Quaternion_Z LSB
		buf.buffer[7] = i2c_readNak();		//Read Quaternion_Z MSB
		i2c_stop();
		
		// invert x, y, and z
		buf.vec[1] *= -1;
		buf.vec[2] *= -1;
		buf.vec[3] *= -1;
		
		// finally write to dedicated buffer
		buffer[0] = buf.buffer[2];
		buffer[1] = buf.buffer[3];
		buffer[2] = buf.buffer[4];
		buffer[3] = buf.buffer[5];
		buffer[4] = buf.buffer[6];
		buffer[5] = buf.buffer[7];
	}
	else
	{
		// otherwise read as usual directly into buffer
		buffer[0] = i2c_readAck();		//Read Quaternion_X LSB
		buffer[1] = i2c_readAck();		//Read Quaternion_X MSB
		buffer[2] = i2c_readAck();		//Read Quaternion_Y LSB
		buffer[3] = i2c_readAck();		//Read Quaternion_Y MSB
		buffer[4] = i2c_readAck();		//Read Quaternion_Z LSB
		buffer[5] = i2c_readNak();		//Read Quaternion_Z MSB
		i2c_stop();
	}
}

void BNO_Read_Quaternion_LinAcc(uint8_t id, uint8_t* buffer)
{
	if (!BNO_is_available(id))
	{
		return;
	}
	// select sensor device (ensure id <= 5, not checked here for performance reasons)
	BNO_MUX_Select(id);
	
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


void BNO_Read_Quaternion_LinAcc_Compressed(uint8_t id, uint8_t* buffer_quat, uint8_t* buffer_linAcc)
{
	if (!BNO_is_available(id))
	{
		return;
	}
	// select sensor device (ensure id <= 5, not checked here for performance reasons)
	BNO_MUX_Select(id);
	
	union QuatBuffer buf;
	
	// start reading from BNO
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_QUATERNION_DATA_W_LSB_ADDR);	//Access LSB of Quaternion W data
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	buf.buffer[0] = i2c_readAck();		//Read Quaternion_W LSB
	buf.buffer[1] = i2c_readAck();		//Read Quaternion_W MSB
	
	// check if w negative
	if (buf.vec[0] < 0)
	{
		buf.buffer[2] = i2c_readAck();		//Read Quaternion_X LSB
		buf.buffer[3] = i2c_readAck();		//Read Quaternion_X MSB
		buf.buffer[4] = i2c_readAck();		//Read Quaternion_Y LSB
		buf.buffer[5] = i2c_readAck();		//Read Quaternion_Y MSB
		buf.buffer[6] = i2c_readAck();		//Read Quaternion_Z LSB
		buf.buffer[7] = i2c_readAck();		//Read Quaternion_Z MSB
		
		buffer_linAcc[0] = i2c_readAck();			//Read lin_acc_X LSB
		buffer_linAcc[1] = i2c_readAck();			//Read lin_acc_X MSB
		buffer_linAcc[2] = i2c_readAck();			//Read lin_acc_Y LSB
		buffer_linAcc[3] = i2c_readAck();			//Read lin_acc_Y MSB
		buffer_linAcc[4] = i2c_readAck();			//Read lin_acc_Z LSB
		buffer_linAcc[5] = i2c_readNak();			//Read lin_acc_Z MSB
		i2c_stop();
		
		// invert x, y, and z
		buf.vec[1] *= -1;
		buf.vec[2] *= -1;
		buf.vec[3] *= -1;
		
		// finally write to dedicated buffer
		buffer_quat[0] = buf.buffer[2];
		buffer_quat[1] = buf.buffer[3];
		buffer_quat[2] = buf.buffer[4];
		buffer_quat[3] = buf.buffer[5];
		buffer_quat[4] = buf.buffer[6];
		buffer_quat[5] = buf.buffer[7];
	}
	else
	{
		// otherwise read as usual directly into buffer
		buffer_quat[0] = i2c_readAck();		//Read Quaternion_X LSB
		buffer_quat[1] = i2c_readAck();		//Read Quaternion_X MSB
		buffer_quat[2] = i2c_readAck();		//Read Quaternion_Y LSB
		buffer_quat[3] = i2c_readAck();		//Read Quaternion_Y MSB
		buffer_quat[4] = i2c_readAck();		//Read Quaternion_Z LSB
		buffer_quat[5] = i2c_readNak();		//Read Quaternion_Z MSB
		
		buffer_linAcc[0] = i2c_readAck();			//Read lin_acc_X LSB
		buffer_linAcc[1] = i2c_readAck();			//Read lin_acc_X MSB
		buffer_linAcc[2] = i2c_readAck();			//Read lin_acc_Y LSB
		buffer_linAcc[3] = i2c_readAck();			//Read lin_acc_Y MSB
		buffer_linAcc[4] = i2c_readAck();			//Read lin_acc_Z LSB
		buffer_linAcc[5] = i2c_readNak();			//Read lin_acc_Z MSB
		i2c_stop();
	}
	
}


void BNO_Read_Acc_Mag_Gyr(uint8_t id, uint8_t* buffer)
{
	if (!BNO_is_available(id))
	{
		return;
	}
	// select sensor device (ensure id <= 5, not checked here for performance reasons)
	BNO_MUX_Select(id);
	
	i2c_start_wait(BNO055_ADDRESS + I2C_WRITE);	//Set device address and write mode
	i2c_write(BNO055_ACCEL_DATA_X_LSB_ADDR);	//Access LSB of Accelerometer X data
	i2c_rep_start(BNO055_ADDRESS + I2C_READ);	//Set device address and read mode
	buffer[0] = i2c_readAck();		//Read Accelerometer_X LSB
	buffer[1] = i2c_readAck();		//Read Accelerometer_X MSB
	buffer[2] = i2c_readAck();		//Read Accelerometer_Y LSB
	buffer[3] = i2c_readAck();		//Read Accelerometer_Y MSB
	buffer[4] = i2c_readAck();		//Read Accelerometer_Z LSB
	buffer[5] = i2c_readAck();		//Read Accelerometer_Z MSB
	buffer[6] = i2c_readAck();		//Read Magnetometer_X LSB
	buffer[7] = i2c_readAck();		//Read Magnetometer_X MSB
	buffer[8] = i2c_readAck();		//Read Magnetometer_Y LSB
	buffer[9] = i2c_readAck();		//Read Magnetometer_Y MSB
	buffer[10] = i2c_readAck();		//Read Magnetometer_Z LSB
	buffer[11] = i2c_readAck();		//Read Magnetometer_Z MSB
	buffer[12] = i2c_readAck();		//Read Gyroscope_X LSB
	buffer[13] = i2c_readAck();		//Read Gyroscope_X MSB
	buffer[14] = i2c_readAck();		//Read Gyroscope_Y LSB
	buffer[15] = i2c_readAck();		//Read Gyroscope_Y MSB
	buffer[16] = i2c_readAck();		//Read Gyroscope_Z LSB
	buffer[17] = i2c_readNak();		//Read Gyroscope_Z MSB
	i2c_stop();
}



void BNO_MUX_Select(uint8_t sen_channel)
{
	switch (sen_channel)
	{
		//Select IMU 0	-->  Y5
		case 0:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD |= _BV(MUX_S2);		//S2 High
			break;

		//Select IMU 1	-->  Y4
		case 1:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD |= _BV(MUX_S2);		//S2 High
			break;

		//Select IMU 2	-->  Y2
		case 2:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD |= _BV(MUX_S1);		//S1 High
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		//Select IMU 3	-->  Y1
		case 3:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		//Select IMU 4	-->  Y0
		case 4:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD &= ~(_BV(MUX_S1));	//S1 Low
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;

		//Select IMU 5	-->  Y3
		case 5:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD |= _BV(MUX_S1);		//S1 High
			PORTD &= ~(_BV(MUX_S2));	//S2 Low
			break;
			
		//Select IMU 6	-->  Y6
		case 6:
			PORTD &= ~(_BV(MUX_S0));	//S0 Low
			PORTD |= _BV(MUX_S1);		//S1 High
			PORTD |= _BV(MUX_S2);		//S2 High
			break;
			
		//Select IMU 7	-->  Y7
		case 7:
			PORTD |= _BV(MUX_S0);		//S0 High
			PORTD |= _BV(MUX_S1);		//S1 High
			PORTD |= _BV(MUX_S2);		//S2 High
			break;
			

		default:
			break;
			// UART_Put_String("Wrong Channel!\n");
	}
	// delay for a short period of time to give mux time to settle (might not be necessary)
	_delay_us(1);
}