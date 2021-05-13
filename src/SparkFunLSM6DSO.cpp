/******************************************************************************
SparkFunLSM6DSO.cpp
LSM6DSO Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DSO_Breakout
https://github.com/sparkfun/SparkFun_LSM6DSO_Arduino_Library

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

//See SparkFunLSM6DSO.h for additional topology notes.

#include "SparkFunLSM6DSO.h"

//****************************************************************************//
//
//  LSM6DSOCore functions.
//
//  Construction arguments:
//  ( uint8_t busType, uint8_t inputArg ),
//
//    where inputArg is address for I2C_MODE and chip select pin
//    number for SPI_MODE
//
//  For SPI, construct LSM6DSOCore myIMU(SPI_MODE, 10);
//  For I2C, construct LSM6DSOCore myIMU(I2C_MODE, 0x6B);
//
//  Default construction is I2C mode, address 0x6B.
//
//****************************************************************************//
LSM6DSOCore::LSM6DSOCore( uint8_t busType, uint8_t inputArg) : commInterface(I2C_MODE), I2CAddress(0x6B), chipSelectPin(10)
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

status_t LSM6DSOCore::beginCore(void)
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
	readRegister(&readCheck, WHO_AM_I_REG);
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
//    numBytes -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LSM6DSOCore::readRegisterRegion(uint8_t outputPointer[] , uint8_t offset, uint8_t numBytes)
{
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space

	switch( commInterface ){

	case I2C_MODE:

		_i2cPort->beginTransmission(I2CAddress);
		_i2cPort->write(offset);
		if( _i2cPort->endTransmission(false) != 0 )
			return IMU_HW_ERROR;

    _i2cPort->requestFrom(I2CAddress, numBytes);
    for(size_t i = 0; i < numBytes; i++){
       outputPointer[i] =  _i2cPort->read(); 
    }

    if( _i2cPort->endTransmission() != 0 )
      return IMU_HW_ERROR;
    else
      return IMU_SUCCESS;

	case SPI_MODE:

    _spiPort->beginTransaction(mySpiSettings);
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		_spiPort->transfer(offset | 0x80);  //Ored with "read request" bit

		for(size_t i = 0; i < numBytes; i++ ) {
			outputPointer[i] = _spiPort->transfer(0x00); // receive a byte as character
		}

		digitalWrite(chipSelectPin, HIGH);
    _spiPort->endTransaction();

    return IMU_SUCCESS;

  }
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
status_t LSM6DSOCore::readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result;
	status_t returnError; 

	switch (commInterface) {

	case I2C_MODE:

		_i2cPort->beginTransmission(I2CAddress);
		_i2cPort->write(offset);
		if( _i2cPort->endTransmission() != 0 )
			returnError = IMU_HW_ERROR;

		_i2cPort->requestFrom(static_cast<uint8_t>(I2CAddress), static_cast<uint8_t>(1));
    *outputPointer = _i2cPort->read(); // receive a byte as a proper uint8_t
    if( _i2cPort->endTransmission() != 0) 
      return IMU_HW_ERROR;
    else
      return IMU_SUCCESS;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
    _spiPort->beginTransaction(mySpiSettings); 
		// send the device the register you want to read:
		_spiPort->transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		*outputPointer = _spiPort->transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
    _spiPort->endTransaction(); 
		
      return IMU_SUCCESS; 
	}

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
status_t LSM6DSOCore::readRegisterInt16(int16_t* outputPointer, uint8_t offset) 
{
	uint8_t myBuffer[2];
	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = myBuffer[0] | static_cast<uint16_t>(myBuffer[1] << 8);
	
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
status_t LSM6DSOCore::writeRegister(uint8_t offset, uint8_t dataToWrite) {

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

status_t LSM6DSOCore::enableEmbeddedFunctions(bool enable)
{
  uint8_t tempVal; 
  readRegister(&tempVal, FUNC_CFG_ACCESS);
  
  if( enable )
    tempVal |= 0x80;  
  else
    tempVal |= 0x7F; 

	status_t returnError = writeRegister( FUNC_CFG_ACCESS, tempVal );
	
	return returnError;
}

//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LSM6DSO::LSM6DSO( uint8_t busType, uint8_t inputArg ) : LSM6DSOCore( busType, inputArg )
{
	//Construct with these default settings

	settings.gyroEnabled = 1;  //Can be 0 or 1
	settings.gyroRange = 500;   //Max deg/s.  Can be: 125, 250, 500, 1000, 2000
	settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
	settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
	settings.gyroAccelDecimation = 1;  //Set to include gyro in FIFO

	settings.accelEnabled = 1;
	settings.accelRange = 8;      //Max G force readable.  Can be: 2, 4, 8, 16
	settings.accelSampleRate = 416;  //Hz.  Can be: 1.6 (16), 12.5 (125), 26, 52, 104, 208, 416, 833, 1660, 3330, 6660
	settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO

	settings.tempEnabled = 1;

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
status_t LSM6DSO::begin()
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
			dataToWrite |= FS_XL_2g;
			break;
		case 4:
			dataToWrite |= FS_XL_4g;
			break;
		case 8:
			dataToWrite |= FS_XL_8g;
			break;
		default:  //set default case to 16(max)
		case 16:
			dataToWrite |= FS_XL_16g;
			break;
		}
		// Accelerometer ODR
		switch (settings.accelSampleRate) {
		case 16:
			dataToWrite |= ODR_XL_1_6Hz;
			break;
		case 125:
			dataToWrite |= ODR_XL_12_5Hz;
			break;
		case 26:
			dataToWrite |= ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite |= ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite |= ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite |= ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite |= ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite |= ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite |= ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite |= ODR_XL_6660Hz;
			break;
		}
	}

  // Write Accelerometer Settings....
	writeRegister(CTRL1_XL, dataToWrite);

	//Setup the gyroscope**********************************************
	dataToWrite = 0; // Clear variable

	if ( settings.gyroEnabled == 1) {
		switch (settings.gyroRange) {
		case 125:
			dataToWrite |=  FS_125_ENABLED;
			break;
		case 245:
			dataToWrite |=  FS_G_245dps;
			break;
		case 500:
			dataToWrite |=  FS_G_500dps;
			break;
		case 1000:
			dataToWrite |=  FS_G_1000dps;
			break;
		default:  //Default to full 2000DPS range
		case 2000:
			dataToWrite |=  FS_G_2000dps;
			break;
		}
		switch (settings.gyroSampleRate) { 
		case 125:
			dataToWrite |= GYRO_ODR_12_5Hz;
			break;
		case 26:
			dataToWrite |= GYRO_ODR_26Hz;
			break;
		case 52:
			dataToWrite |= GYRO_ODR_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= GYRO_ODR_104Hz;
			break;
		case 208:
			dataToWrite |= GYRO_ODR_208Hz;
			break;
		case 416:
			dataToWrite |= GYRO_ODR_416Hz;
			break;
		case 833:
			dataToWrite |= GYRO_ODR_833Hz;
			break;
		case 1660:
			dataToWrite |= GYRO_ODR_1660Hz;
			break;
		case 3330:
			dataToWrite |= GYRO_ODR_3330Hz;
			break;
		case 6660:
			dataToWrite |= GYRO_ODR_6660Hz;
			break;
		}
	}
	
  // Write the gyroscope settings. 
	writeRegister(CTRL2_G, dataToWrite);

	return returnError;
}

// Address: 0x1E , bit[2:0]: default value is: 0x00
// Checks if there is new accelerometer, gyro, or temperature data.
uint8_t LSM6DSO::getDataReady(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, STATUS_REG);
  
  if( returnError != IMU_SUCCESS )
    return static_cast<uint8_t>(returnError);
  else
    return regVal; 
}

// Address:0x12 CTRL3_C , bit[6] default value is: 0x00
// This function sets the BDU (Block Data Update) bit. Use when not employing
// the FIFO buffer.
bool LSM6DSO::setBlockDataUpdate(bool enable){

  status_t returnError = writeRegister(CTRL3_C, 0x40);  			

  if( returnError != IMU_SUCCESS )
    return false;
  else 
    return true;


}


bool LSM6DSO::setHighPerfAccel(bool enable){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL6_C);
  if( returnError != IMU_SUCCESS )
    return false; 

  if( enable )
    regVal |=  HIGH_PERF_ACC_ENABLE; 
  else
    regVal |=  HIGH_PERF_ACC_DISABLE; 

  returnError = writeRegister(CTRL6_C, regVal);
  if( returnError != IMU_SUCCESS )
    return false; 
  else
    return true;
}

bool LSM6DSO::setHighPerfGyro(bool enable){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL7_G);
  if( returnError != IMU_SUCCESS )
    return false; 

  if( enable )
    regVal |=  HIGH_PERF_GYRO_ENABLE; 
  else
    regVal |=  HIGH_PERF_GYRO_DISABLE; 

  returnError = writeRegister(CTRL7_G, regVal);
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
int16_t LSM6DSO::readRawAccelX( void ) {

	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, OUTX_L_A );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
			allOnesCounter++;
		else
			nonSuccessCounter++;
	}
	return output;
}

float LSM6DSO::readFloatAccelX( void ) {
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t LSM6DSO::readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, OUTY_L_A );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
			allOnesCounter++;
		else
			nonSuccessCounter++;
	}
	return output;
}

float LSM6DSO::readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t LSM6DSO::readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, OUTZ_L_A );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
			allOnesCounter++;
		else
			nonSuccessCounter++;
	}
	return output;
}

float LSM6DSO::readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float LSM6DSO::calcAccel( int16_t input )
{
  uint8_t accelRange; 
  uint8_t scale;
  float output;

  readRegister(&accelRange, CTRL1_XL);
  scale = (accelRange >> 1) & 0x01;
  accelRange = (accelRange >> 2) & (0x03);  
  
  if( scale == 0 ) {
    switch( accelRange ){
      case 0:// Register value 0: 2g
        output = (static_cast<float>(input) * (.061)) / 1000;
        break;
      case 1: //Register value 1 : 16g
        output = (static_cast<float>(input) * (.488)) / 1000;
        break;
      case 2: //Register value 2 : 4g
        output = (static_cast<float>(input) * (.122)) / 1000;
        break;
      case 3://Register value 3: 8g
        output = (static_cast<float>(input) * (.244)) / 1000;
        break;
    }
  }

  if( scale == 1 ){
    switch( accelRange ){
      case 0: //Register value 0: 2g
        output = (static_cast<float>(input) * (0.061)) / 1000;
        break;
      case 1://Register value 1: 2g
        output = (static_cast<float>(input) * (0.061)) / 1000;
        break;
      case 2://Register value 2: 4g
        output = (static_cast<float>(input) * (.122)) / 1000;
        break;
      case 3://Register value 3: 8g
        output = (static_cast<float>(input) * (.244)) / 1000;
        break;
    }
  }

	return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t LSM6DSO::readRawGyroX( void ) {

	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, OUTX_L_G );

	if( errorLevel != IMU_SUCCESS ) {
		if( errorLevel == IMU_ALL_ONES_WARNING )
			allOnesCounter++;
		else
			nonSuccessCounter++;
	}

	return output;
}

float LSM6DSO::readFloatGyroX( void ) {

	float output = calcGyro(readRawGyroX());
	return output;
}

int16_t LSM6DSO::readRawGyroY( void ) {

	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, OUTY_L_G );

	if( errorLevel != IMU_SUCCESS ) {
		if( errorLevel == IMU_ALL_ONES_WARNING )
			allOnesCounter++;
		else
			nonSuccessCounter++;
	}

	return output;
}

float LSM6DSO::readFloatGyroY( void ) {
  
	float output = calcGyro(readRawGyroY());
	return output;
}

int16_t LSM6DSO::readRawGyroZ( void ) {

	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, OUTZ_L_G );

	if( errorLevel != IMU_SUCCESS ) {
		if( errorLevel == IMU_ALL_ONES_WARNING )
			allOnesCounter++;
		else
			nonSuccessCounter++;
	}

	return output;
}

float LSM6DSO::readFloatGyroZ( void ) {

	float output = calcGyro(readRawGyroZ());
	return output;

}

float LSM6DSO::calcGyro( int16_t input ) {

	uint8_t gyroRange;  
  uint8_t fullScale;
  float output; 

  readRegister(&gyroRange, CTRL2_G) ;
  fullScale = (gyroRange >> 1) & 0x01; 
  gyroRange = (gyroRange >> 2) & 0x03; 

  if( fullScale )
    output = (static_cast<float>(input) * 4.375)/1000;
  else {
    switch( gyroRange ){
      case 0:
        output = (static_cast<float>(input) * 8.75)/1000;
        break;
      case 1:
        output = (static_cast<float>(input) * 17.50)/1000;
        break;
      case 2:
        output = (static_cast<float>(input) * 35)/1000;
        break;
      case 3:
        output = (static_cast<float>(input) * 70)/1000;
        break;
    }
  }

  return output;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LSM6DSO::readRawTemp( void )
{
	int16_t output;
	readRegisterInt16( &output, OUT_TEMP_L );
	return output;
}  

float LSM6DSO::readTempC( void )
{
	int16_t temp = (readRawTemp()); 
  int8_t msbTemp = (temp & 0xFF00) >> 8;  
  float tempFloat = static_cast<float>(msbTemp);
  float lsbTemp =  temp & 0x00FF;

  lsbTemp /= 256;
  
  tempFloat += lsbTemp; 
	tempFloat += 25; //Add 25 degrees to remove offset

	return tempFloat;

}

float LSM6DSO::readTempF( void )
{
	float output = readTempC(); 
	output = (output * 9) / 5 + 32;

	return output;

}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LSM6DSO::fifoBegin( void ) {
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
  readRegister(&tempFIFO_CTRL4, FIFO_CTRL4);
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
	writeRegister(FIFO_CTRL1, thresholdLByte);
  uint8_t tempVal;
  tempVal = readRegister(&tempVal, FIFO_CTRL2);
  // Mask threshold bytes
  tempVal &= 0xFE;
  // Merge bytes
  tempVal |= thresholdHByte; 
	writeRegister(FIFO_CTRL2, tempVal);

	//Serial.println(thresholdHByte, HEX);
	writeRegister(FIFO_CTRL4, tempFIFO_CTRL4);

}

void LSM6DSO::fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x1000 ) == 0 ) {
		fifoRead();
	}

}

fifoData LSM6DSO::fifoRead( void ) {
	//Pull the last data from the fifo
  uint8_t tempTagByte; 
  uint8_t tempAccumulator;  
  fifoData tempFifoData; 
  

  readRegister(&tempTagByte, FIFO_DATA_OUT_TAG);

  tempFifoData.fifoTag = tempTagByte;

  readRegister(&tempAccumulator, FIFO_DATA_OUT_X_H);
  tempFifoData.xData = (static_cast<uint16_t>(tempAccumulator) << 8);
  readRegister(&tempAccumulator, FIFO_DATA_OUT_X_L);
  tempFifoData.xData |= tempAccumulator;
  readRegister(&tempAccumulator, FIFO_DATA_OUT_Y_H);
  tempFifoData.yData = (static_cast<uint16_t>(tempAccumulator) << 8);
  readRegister(&tempAccumulator, FIFO_DATA_OUT_Y_L);
  tempFifoData.yData |= tempAccumulator;
  readRegister(&tempAccumulator, FIFO_DATA_OUT_Z_H);
  tempFifoData.zData = (static_cast<uint16_t>(tempAccumulator) << 8);
  readRegister(&tempAccumulator, FIFO_DATA_OUT_Z_L);
  tempFifoData.zData |= tempAccumulator;

  return tempFifoData;
  
}

uint16_t LSM6DSO::fifoGetStatus( void ) {
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, FIFO_STATUS1);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, FIFO_STATUS2);
	tempAccumulator |= (tempReadByte << 8);

	return tempAccumulator;  

}
void LSM6DSO::fifoEnd( void ) {
	// turn off the fifo
	writeRegister(FIFO_STATUS1, 0x00);  //Disable
}

