/******************************************************************************
SparkFunLSM6DS0.cpp
LSM6DS0 Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS0_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS0_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23
0
This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

//See SparkFunLSM6DS0.h for additional topology notes.

#include "SparkFunLSM6DS0.h"

//****************************************************************************//
//
//  LSM6DS0Core functions.
//
//  Construction arguments:
//  ( uint8_t busType, uint8_t inputArg ),
//
//    where inputArg is address for I2C_MODE and chip select pin
//    number for SPI_MODE
//
//  For SPI, construct LSM6DS0Core myIMU(SPI_MODE, 10);
//  For I2C, construct LSM6DS0Core myIMU(I2C_MODE, 0x6B);
//
//  Default construction is I2C mode, address 0x6B.
//
//****************************************************************************//
LSM6DS0Core::LSM6DS0Core( uint8_t busType, uint8_t inputArg) : commInterface(I2C_MODE), I2CAddress(0x6B), chipSelectPin(10)
{
	commInterface = busType;
	if( commInterface == I2C_MODE )
	{
		I2CAddress = inputArg;
    _i2cPort = &Wire;
	}
	if( commInterface == SPI_MODE )
	{
    _spiPort = &SPI; 
		chipSelectPin = inputArg;
	}

}

status_t LSM6DS0Core::beginCore(void)
{
	status_t returnError = IMU_SUCCESS;
  uint32_t spiPortSpeed = 10000000;

	switch (commInterface) {

	case I2C_MODE:
		break;

	case SPI_MODE:
		// Data is read and written MSb first.
		// Maximum SPI frequency is 10MHz, could divide by 2 here:

#ifdef __AVR__
    mySpiSettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE1);
#endif
		// MODE0 for Teensy 3.1 operation
#ifdef __MK20DX256__
    mySpiSettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE0);
#endif
		
#ifdef ESP32
    mySpiSettings = SPISettings(spiPortSpeed, SPI_MSBFIRST, SPI_MODE1);
#endif

		// initalize the  data ready and chip select pins:
		pinMode(chipSelectPin, OUTPUT);
		digitalWrite(chipSelectPin, HIGH);
		break;
	default:
		break;
	}

	//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ )
	{
		temp++;
	}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LSM6DS0_ACC_GYRO_WHO_AM_I_REG);
	if( readCheck != 0x6C )
	{
		returnError = IMU_HW_ERROR;
	}

	return returnError;

}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LSM6DS0Core::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;
	uint8_t tempFFCounter = 0;

	switch (commInterface) {

	case I2C_MODE:
		_i2cPort->beginTransmission(I2CAddress);
		_i2cPort->write(offset);
		if( _i2cPort->endTransmission() != 0 )
		{
			returnError = IMU_HW_ERROR;
		}
		else  //OK, all worked, keep going
		{
			// request 6 bytes from slave device
			_i2cPort->requestFrom(I2CAddress, length);
			while ( (_i2cPort->available()) && (i < length))  // slave may send less than requested
			{
				c = _i2cPort->read(); // receive a byte as character
				*outputPointer = c;
				outputPointer++;
				i++;
			}
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
    _spiPort->beginTransaction(mySpiSettings);
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		_spiPort->transfer(offset | 0x80);  //Ored with "read request" bit
		while ( i < length ) // slave may send less than requested
		{
			c = _spiPort->transfer(0x00); // receive a byte as character
			if( c == 0xFF )
			{
				//May have problem
				tempFFCounter++;
			}
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		if( tempFFCounter == i )
		{
			//Ok, we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
    _spiPort->endTransaction();
		break;

	default:
		break;
	}

	return returnError;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS0Core::readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	status_t returnError = IMU_SUCCESS;

	switch (commInterface) {

	case I2C_MODE:
		_i2cPort->beginTransmission(I2CAddress);
		_i2cPort->write(offset);
		if( _i2cPort->endTransmission() != 0 )
		{
			returnError = IMU_HW_ERROR;
		}
		_i2cPort->requestFrom(I2CAddress, numBytes);
		while ( _i2cPort->available() ) // slave may send less than requested
		{
			result = _i2cPort->read(); // receive a byte as a proper uint8_t
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
    _spiPort->beginTransaction(mySpiSettings); 
		// send the device the register you want to read:
		_spiPort->transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		result = _spiPort->transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
    _spiPort->endTransaction(); 
		
		if( result == 0xFF )
		{
			//we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}
		break;

	default:
		break;
	}

	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS0Core::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	uint8_t myBuffer[2];
	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = static_cast<int16_t>(myBuffer[0]) | static_cast<int16_t>(myBuffer[1] << 8);
	
	*outputPointer = output;
	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t LSM6DS0Core::writeRegister(uint8_t offset, uint8_t dataToWrite) {
	status_t returnError = IMU_SUCCESS;
	switch (commInterface) {
	case I2C_MODE:
		//Write the byte
		_i2cPort->beginTransmission(I2CAddress);
		_i2cPort->write(offset);
		_i2cPort->write(dataToWrite);
		if( _i2cPort->endTransmission() != 0 )
		{
			returnError = IMU_HW_ERROR;
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
    _spiPort->beginTransaction(mySpiSettings); 
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		_spiPort->transfer(offset);
		// send a value of 0 to read the first byte returned:
		_spiPort->transfer(dataToWrite);
		// decrement the number of bytes left to read:
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
    _spiPort->endTransaction(); 
		break;
		
		//No way to check error on this write (Except to read back but that's not reliable)

	default:
		break;
	}

	return returnError;
}

status_t LSM6DS0Core::embeddedPage( void )
{
  uint8_t tempVal; 
  readRegister(&tempVal,  LSM6DS0_ACC_GYRO_RAM_ACCESS);
  tempVal = tempVal | 0x80;
	status_t returnError = writeRegister( LSM6DS0_ACC_GYRO_RAM_ACCESS, tempVal );
	
	return returnError;
}

status_t LSM6DS0Core::basePage( void )
{
	status_t returnError = writeRegister( LSM6DS0_ACC_GYRO_RAM_ACCESS, 0x00 );
	
	return returnError;
}


//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LSM6DS0::LSM6DS0( uint8_t busType, uint8_t inputArg ) : LSM6DS0Core( busType, inputArg )
{
	//Construct with these default settings

	settings.gyroEnabled = 1;  //Can be 0 or 1
	settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
	settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
	settings.gyroAccelDecimation = 1;  //set 1 for on /1

	settings.accelEnabled = 1;
	settings.accelODROff = 1;
	settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
	settings.accelSampleRate = 416;  //Hz.  Can be: 1.6 (16), 12.5 (125), 26, 52, 104, 208, 416, 833, 1660, 3330, 6660
	settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO

	settings.tempEnabled = 1;

	//Select interface mode
	settings.commMode = 1;  //Can be modes 1, 2 or 3

	//FIFO control data
	settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
	//settings.fifoSampleRate = 10;  //default 10Hz
	settings.fifoModeWord = 0;  //Default off

	allOnesCounter = 0;
	nonSuccessCounter = 0;

}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t LSM6DS0::begin()
{
	uint8_t dataToWrite = 0;  //Temporary variable

	status_t returnError = beginCore();
  if( returnError != IMU_SUCCESS ) 
    return returnError;

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	if ( settings.accelEnabled == 1) {
    //Range
		switch (settings.accelRange) {
		case 2:
			dataToWrite |= LSM6DS0_ACC_FS_XL_2g;
			break;
		case 4:
			dataToWrite |= LSM6DS0_ACC_FS_XL_4g;
			break;
		case 8:
			dataToWrite |= LSM6DS0_ACC_FS_XL_8g;
			break;
		default:  //set default case to 16(max)
		case 16:
			dataToWrite |= LSM6DS0_ACC_FS_XL_16g;
			break;
		}
		// Accelerometer ODR
		switch (settings.accelSampleRate) {
		case 16:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_1_6Hz;
			break;
		case 125:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_12_5Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite |= LSM6DS0_ACC_ODR_XL_6660Hz;
			break;
		}
	}

  // Write Accelerometer Settings....
	writeRegister(LSM6DS0_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Setup the gyroscope**********************************************
	dataToWrite = 0; // Clear variable

	if ( settings.gyroEnabled == 1) {
		switch (settings.gyroRange) {
		case 125:
			dataToWrite |= LSM6DS0_ACC_GYRO_FS_125_ENABLED;
			break;
		case 245:
			dataToWrite |= LSM6DS0_ACC_GYRO_FS_G_245dps;
			break;
		case 500:
			dataToWrite |= LSM6DS0_ACC_GYRO_FS_G_500dps;
			break;
		case 1000:
			dataToWrite |= LSM6DS0_ACC_GYRO_FS_G_1000dps;
			break;
		default:  //Default to full 2000DPS range
		case 2000:
			dataToWrite |= LSM6DS0_ACC_GYRO_FS_G_2000dps;
			break;
		}
		switch (settings.gyroSampleRate) { 
		case 125:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_12_5Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_1660Hz;
			break;
		case 3330:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_3330Hz;
			break;
		case 6660:
			dataToWrite |= LSM6DS0_GYRO_ODR_G_6660Hz;
			break;
		}
	}
	
  // Write the gyroscope settings. 
	writeRegister(LSM6DS0_ACC_GYRO_CTRL2_G, dataToWrite);
  setBlockDataUpdate(true);

	return returnError;
}
// Address:0x12 CTRL3_C , bit[6] default value is: 0x00
// This function sets the BDU (Block Data Update) bit. Use when not employing
// the FIFO buffer.
bool LSM6DS0::setBlockDataUpdate(bool enable){

  status_t returnError = writeRegister(LSM6DS0_ACC_GYRO_CTRL3_C, 0x40);  			

  if( returnError != IMU_SUCCESS )
    return false;
  else 
    return true;


}
//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LSM6DS0::readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUTX_L_A );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}

float LSM6DS0::readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t LSM6DS0::readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUTY_L_A );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS0::readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t LSM6DS0::readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUTZ_L_A );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS0::readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float LSM6DS0::calcAccel( int16_t input )
{
	float output = static_cast<float>(input) * 0.061 * (settings.accelRange >> 1) / 1000;
	return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t LSM6DS0::readRawGyroX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUTX_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS0::readFloatGyroX( void )
{
	float output = calcGyro(readRawGyroX());
	return output;
}

int16_t LSM6DS0::readRawGyroY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUTY_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS0::readFloatGyroY( void )
{
	float output = calcGyro(readRawGyroY());
	return output;
}

int16_t LSM6DS0::readRawGyroZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUTZ_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS0::readFloatGyroZ( void )
{
	float output = calcGyro(readRawGyroZ());
	return output;
}

float LSM6DS0::calcGyro( int16_t input )
{
	uint8_t gyroRangeDivisor = settings.gyroRange / 125;
	if ( settings.gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	float output = static_cast<float>(input) * 4.375 * (gyroRangeDivisor) / 1000;
	return output;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LSM6DS0::readRawTemp( void )
{
	int16_t output;
	readRegisterInt16( &output, LSM6DS0_ACC_GYRO_OUT_TEMP_L );
	return output;
}  

float LSM6DS0::readTempC( void )
{
	float output = static_cast<float>( readRawTemp() ) / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;

}

float LSM6DS0::readTempF( void )
{
	float output = static_cast<float>( readRawTemp() ) / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;

}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LSM6DS0::fifoBegin( void ) {
	//CONFIGURE THE VARIOUS FIFO SETTINGS
	//
	//
	//This section first builds a bunch of config words, then goes through
	//and writes them all.

	//Split and mask the threshold
	uint8_t thresholdLByte = (settings.fifoThreshold & 0x007F) >> 1;
	uint8_t thresholdHByte = (settings.fifoThreshold & 0x00F0) >> 7;

	//CONFIGURE FIFO_CTRL4
	uint8_t tempFIFO_CTRL4;
  readRegister(&tempFIFO_CTRL4, LSM6DS0_ACC_GYRO_FIFO_CTRL4);
  // Clear fifoMode bits
  tempFIFO_CTRL4 &= 0xF8;
  // Merge bits
  tempFIFO_CTRL4 |= settings.fifoModeWord;
	if (settings.gyroFifoEnabled == 1 | settings.accelFifoEnabled == 1)
	{
		//Decimation is calculated as max rate between accel and gyro
    //Clear decimation bits
    tempFIFO_CTRL4 &= 0x3F; 
    // Merge bits
		tempFIFO_CTRL4 |= (settings.gyroAccelDecimation << 6);
  }

	//Write the data
	//Serial.println(thresholdLByte, HEX);
	writeRegister(LSM6DS0_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
  uint8_t tempVal;
  tempVal = readRegister(&tempVal, LSM6DS0_ACC_GYRO_FIFO_CTRL2);
  // Mask threshold bytes
  tempVal &= 0xFE;
  // Merge bytes
  tempVal |= thresholdHByte; 
	writeRegister(LSM6DS0_ACC_GYRO_FIFO_CTRL2, tempVal);

	//Serial.println(thresholdHByte, HEX);
	writeRegister(LSM6DS0_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);

}

void LSM6DS0::fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x1000 ) == 0 ) {
		fifoRead();
	}

}

fifoData LSM6DS0::fifoRead( void ) {
	//Pull the last data from the fifo
  uint8_t tempTagByte; 
  uint8_t tempAccumulator;  
  fifoData tempFifoData; 
  

  readRegister(&tempTagByte, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_TAG);

  tempFifoData.fifoTag = tempTagByte;

  readRegister(&tempAccumulator, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_X_H);
  tempFifoData.xData = (static_cast<uint16_t>(tempAccumulator) << 8);
  readRegister(&tempAccumulator, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_X_L);
  tempFifoData.xData |= tempAccumulator;
  readRegister(&tempAccumulator, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Y_H);
  tempFifoData.yData = (static_cast<uint16_t>(tempAccumulator) << 8);
  readRegister(&tempAccumulator, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Y_L);
  tempFifoData.yData |= tempAccumulator;
  readRegister(&tempAccumulator, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Z_H);
  tempFifoData.zData = (static_cast<uint16_t>(tempAccumulator) << 8);
  readRegister(&tempAccumulator, LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Z_L);
  tempFifoData.zData |= tempAccumulator;

  return tempFifoData;
  
}

uint16_t LSM6DS0::fifoGetStatus( void ) {
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS0_ACC_GYRO_FIFO_STATUS1);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS0_ACC_GYRO_FIFO_STATUS2);
	tempAccumulator |= (tempReadByte << 8);

	return tempAccumulator;  

}
void LSM6DS0::fifoEnd( void ) {
	// turn off the fifo
	writeRegister(LSM6DS0_ACC_GYRO_FIFO_STATUS1, 0x00);  //Disable
}

