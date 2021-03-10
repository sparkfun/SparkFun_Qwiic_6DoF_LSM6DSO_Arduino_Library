/******************************************************************************
Pedometer.ino

Original Library written for the LSM6DS3 by Marshall Taylor @ SparkFun Electronics
Updated to modern SparkFun practices for the LSM6DS0 by Elias Santistevan @ SparkFun Electronics
March, 2021
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DS0
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DS0_Arduino_Library

Description:
This sketch counts steps taken.

Run the sketch and open a serial window at 115200 baud.  The sketch will display
the number of steps taken since reset.  Lightly tap the sensor on something at the
rate of walking to simulate having the device in your pocket.  Note that you must
take 7 regularly spaced steps before the counter starts reporting.

Push the reset button to reset the device and count.

The configuration is determined by reading the LSM6DS0 datasheet and application
note, then driving hex values to the registers of interest to set the appropriate
bits.  The sketch is based of the "LowLevelExampe" sketch.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS0.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DS0Core myIMU( I2C_MODE, 0x6B );
//LSM6DS0Core myIMU( SPI_MODE, 10 );

void setup()
{
	Serial.begin(115200);
	delay(1000); //relax...
	Serial.println("Processor came out of reset.\n");

  Wire.begin();
	//Call .beginCore() to configure the IMU
	if( myIMU.beginCore() != 0 )
	{
		Serial.print("Error at beginCore().\n");
	}
	else
	{
		Serial.print("\nbeginCore() passed.\n");
	}

	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite = 0;  //Temporary variable

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	//  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

	// //Now, write the patched together data
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	errorAccumulator += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

	
	// Enable embedded functions -- ALSO clears the pdeo step count
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
	// Enable pedometer algorithm
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);
	// Step Detector interrupt driven to INT1 pin
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10 );
	
	if( errorAccumulator )
	{
		Serial.println("Problem configuring the device.");
	}
	else
	{
		Serial.println("Device O.K.");
	}	
	delay(200);
}

void loop()
{
	uint8_t readDataByte = 0;
	uint16_t stepsTaken = 0;
	//Read the 16bit value by two 8bit operations
	myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
	stepsTaken = ((uint16_t)readDataByte) << 8;
	
	myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
	stepsTaken |= readDataByte;
	
	//Display steps taken
	Serial.print("Steps taken: ");
	Serial.println(stepsTaken);

	//Wait 1 second
	delay(1000);
	
}
