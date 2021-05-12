/******************************************************************************
SparkFunLSM6DS0.h
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

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LSM6DS0IMU_H__
#define __LSM6DS0IMU_H__

#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

#define I2C_MODE 0
#define SPI_MODE 1

// Return values 
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} status_t;

//This is the core operational class of the driver.
//  LSM6DS0Core contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LSM6DS0 which inherits
//  this class.

class LSM6DS0Core
{
public:
	LSM6DS0Core( uint8_t );
	LSM6DS0Core( uint8_t, uint8_t );
	~LSM6DS0Core() = default;
	
	status_t beginCore( void );
	
	//The following utilities read and write to the IMU

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t );
	
	//readRegister reads one 8-bit register
	status_t readRegister(uint8_t*, uint8_t);
	
	//Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//  Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t*, uint8_t offset );
	
	//Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);
	
	//Change to embedded page
	status_t embeddedPage( void );
	
	//Change to base page
	status_t basePage( void );

  SPISettings mySpiSettings; 
	
private:
	
	//Communication stuff
	uint8_t commInterface;
	uint8_t I2CAddress;
	uint8_t chipSelectPin;

  TwoWire *_i2cPort;
  SPIClass *_spiPort;

};

//This struct holds the settings the driver uses to do calculations
struct SensorSettings {
public:


	//Gyro settings
	uint8_t gyroEnabled;
	uint16_t gyroRange;
	uint16_t gyroSampleRate;
	uint16_t gyroBandWidth;

	uint8_t gyroFifoEnabled;
	uint8_t gyroAccelDecimation;

	//Accelerometer settings
	uint8_t accelEnabled;
	uint8_t accelODROff;
	uint16_t accelRange;
	uint16_t accelSampleRate;
	uint16_t accelBandWidth;
	
	uint8_t accelFifoEnabled;
	
	//Temperature settings
	uint8_t tempEnabled;
	
	//Non-basic mode settings
	uint8_t commMode;
	
	//FIFO control data
	uint16_t fifoThreshold;
	int16_t fifoSampleRate;
	uint8_t fifoModeWord;
	
};

struct fifoData{ 
public:
  uint8_t fifoTag;
  uint16_t xData; 
  uint16_t yData; 
  uint16_t zData; 
};



//This is the highest level class of the driver.
//
//class LSM6DS0 inherits the core and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

class LSM6DS0 : public LSM6DS0Core
{
public:
	//IMU settings
	SensorSettings settings;

	//Error checking
	uint16_t allOnesCounter;
	uint16_t nonSuccessCounter;

	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
	LSM6DS0( uint8_t busType = I2C_MODE, uint8_t inputArg = 0x6B );
	~LSM6DS0() = default;
	
	//Call to apply SensorSettings
	status_t begin(void);

  bool setBlockDataUpdate(bool);
	//Returns the raw bits from the sensor cast as 16-bit signed integers
	int16_t readRawAccelX( void );
	int16_t readRawAccelY( void );
	int16_t readRawAccelZ( void );
	int16_t readRawGyroX( void );
	int16_t readRawGyroY( void );
	int16_t readRawGyroZ( void );

	//Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX( void );
	float readFloatAccelY( void );
	float readFloatAccelZ( void );
	float readFloatGyroX( void );
	float readFloatGyroY( void );
	float readFloatGyroZ( void );

	//Temperature related methods
	int16_t readRawTemp( void );
	float readTempC( void );
	float readTempF( void );

	//FIFO stuff
	void fifoBegin( void );
	void fifoClear( void );
	fifoData fifoRead( void );
	uint16_t fifoGetStatus( void );
	void fifoEnd( void );
	
	float calcGyro( int16_t );
	float calcAccel( int16_t );

private:

};






/************** Device Register  *******************/
#define LSM6DS0_ACC_GYRO_TEST_PAGE  			0x00
#define LSM6DS0_ACC_GYRO_RAM_ACCESS  			0x01

// #define LSM6DS0_ACC_GYRO_SENSOR_SYNC_TIME  		0X04 Removed in LSM6DS0: 0x03- 0x06 Reserved NOT USED in CPP
// #define LSM6DS0_ACC_GYRO_SENSOR_SYNC_EN  		0X05 NOT USED in CPP

// Increment all by one due to RESERVED registers
#define LSM6DS0_ACC_GYRO_FIFO_CTRL1  			0x07
#define LSM6DS0_ACC_GYRO_FIFO_CTRL2  			0x08
#define LSM6DS0_ACC_GYRO_FIFO_CTRL3  			0x09
#define LSM6DS0_ACC_GYRO_FIFO_CTRL4  			0x0A

#define COUNTER_BDR_REG1 0x0B // Added 
#define COUNTER_BDR_REG2 0x0C // Added

#define LSM6DS0_ACC_GYRO_INT1_CTRL  			0x0D
#define LSM6DS0_ACC_GYRO_INT2_CTRL  			0x0E
#define LSM6DS0_ACC_GYRO_WHO_AM_I_REG  			0x0F
#define LSM6DS0_ACC_GYRO_CTRL1_XL  			0x10
#define LSM6DS0_ACC_GYRO_CTRL2_G  			0x11
#define LSM6DS0_ACC_GYRO_CTRL3_C  			0x12
#define LSM6DS0_ACC_GYRO_CTRL4_C  			0x13
#define LSM6DS0_ACC_GYRO_CTRL5_C  			0x14
#define LSM6DS0_ACC_GYRO_CTRL6_G  			0x15
#define LSM6DS0_ACC_GYRO_CTRL7_G  			0x16
#define LSM6DS0_ACC_GYRO_CTRL8_XL  			0x17
#define LSM6DS0_ACC_GYRO_CTRL9_XL  			0x18
#define LSM6DS0_ACC_GYRO_CTRL10_C  			0x19

// #define LSM6DS0_ACC_GYRO_MASTER_CONFIG  		0X1A NOT USED IN CPP
#define LSM6DS0_ALL_INT_SRC  		0x1A // Added - output only i.e. READ
#define LSM6DS0_ACC_GYRO_WAKE_UP_SRC  			0x1B
#define LSM6DS0_ACC_GYRO_TAP_SRC  			0x1C
#define LSM6DS0_ACC_GYRO_D6D_SRC  			0x1D
#define LSM6DS0_ACC_GYRO_STATUS_REG  			0x1E
#define LSM6DS0_ACC_GYRO_OUT_TEMP_L  			0x20
#define LSM6DS0_ACC_GYRO_OUT_TEMP_H  			0x21
#define LSM6DS0_ACC_GYRO_OUTX_L_G  			0x22
#define LSM6DS0_ACC_GYRO_OUTX_H_G  			0x23
#define LSM6DS0_ACC_GYRO_OUTY_L_G  			0x24
#define LSM6DS0_ACC_GYRO_OUTY_H_G  			0x25
#define LSM6DS0_ACC_GYRO_OUTZ_L_G  			0x26
#define LSM6DS0_ACC_GYRO_OUTZ_H_G  			0x27

//#define LSM6DS0_ACC_GYRO_OUTX_L_XL  			0X28 // ----------OLD NAME -------
//#define LSM6DS0_ACC_GYRO_OUTX_H_XL  			0X29
//#define LSM6DS0_ACC_GYRO_OUTY_L_XL  			0X2A
//#define LSM6DS0_ACC_GYRO_OUTY_H_XL  			0X2B
//#define LSM6DS0_ACC_GYRO_OUTZ_L_XL  			0X2C
//#define LSM6DS0_ACC_GYRO_OUTZ_H_XL  			0X2D // ------------^^------------

#define LSM6DS0_ACC_GYRO_OUTX_L_A  			0x28 // ----------NEW NAME -------
#define LSM6DS0_ACC_GYRO_OUTX_H_A  			0x29
#define LSM6DS0_ACC_GYRO_OUTY_L_A  			0x2A
#define LSM6DS0_ACC_GYRO_OUTY_H_A  			0x2B
#define LSM6DS0_ACC_GYRO_OUTZ_L_A  			0x2C
#define LSM6DS0_ACC_GYRO_OUTZ_H_A  			0x2D // ------------^^------------

//  #define LSM6DS0_ACC_GYRO_SENSORHUB1_REG  		0X2E Reserved 0x2E - 0x34 NOT  USED in CPP ------------
//  #define LSM6DS0_ACC_GYRO_SENSORHUB2_REG  		0X2F
//  #define LSM6DS0_ACC_GYRO_SENSORHUB3_REG  		0X30
//  #define LSM6DS0_ACC_GYRO_SENSORHUB4_REG  		0X31
//  #define LSM6DS0_ACC_GYRO_SENSORHUB5_REG  		0X32
//  #define LSM6DS0_ACC_GYRO_SENSORHUB6_REG  		0X33
//  #define LSM6DS0_ACC_GYRO_SENSORHUB7_REG  		0X34 //  --------------^^----------------------

#define LSM6DS0_EMB_FUNC_STATUS_MP  		0x35
#define LSM6DS0_FSM_FUNC_STATUS_A_MP  		0x36
#define LSM6DS0_FSM_FUNC_STATUS_B_MP  		0x37
#define STATUS_MASTER_MAINPAGE  		0x39

//#define LSM6DS0_ACC_GYRO_SENSORHUB8_REG  		0x35 //------Replaced NOT USED---------
//#define LSM6DS0_ACC_GYRO_SENSORHUB9_REG  		0x36
//#define LSM6DS0_ACC_GYRO_SENSORHUB10_REG  		0x37
//#define LSM6DS0_ACC_GYRO_SENSORHUB11_REG  		0x38
//#define LSM6DS0_ACC_GYRO_SENSORHUB12_REG  		0x39// ---------^^------------

#define LSM6DS0_ACC_GYRO_FIFO_STATUS1  			0x3A
#define LSM6DS0_ACC_GYRO_FIFO_STATUS2  			0x3B
//#define LSM6DS0_ACC_GYRO_FIFO_STATUS3  			0x3C //------Replaced ------NOT // USED
//#define LSM6DS0_ACC_GYRO_FIFO_STATUS4  			0x3D NOT USED

//#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_L  		0x3E  FIND REPLACEMENT
//#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_H  		0x3F ////--------^^-------------FIND REPLACEMENT

#define LSM6DS0_ACC_GYRO_TIMESTAMP0_REG  		0x40
#define LSM6DS0_ACC_GYRO_TIMESTAMP1_REG  		0x41
#define LSM6DS0_ACC_GYRO_TIMESTAMP2_REG  		0x42

#define LSM6DS0_ACC_GYRO_TIMESTAMP3_REG  		0x43 // Added

// #define LSM6DS0_ACC_GYRO_STEP_COUNTER_L  		0x4B Reserved 0x44-55 NOT USED
// #define LSM6DS0_ACC_GYRO_STEP_COUNTER_H  		0x4C NOT USED
// #define LSM6DS0_ACC_GYRO_FUNC_SRC  			0x53 NOT USED

#define LSM6DS0_ACC_GYRO_TAP_CFG0  			0x56 // Added
#define LSM6DS0_ACC_GYRO_TAP_CFG1  			0x57 // Changed to 0x57
#define LSM6DS0_ACC_GYRO_TAP_CFG2  			0x58 // Added
#define LSM6DS0_ACC_GYRO_TAP_THS_6D  			0x59
#define LSM6DS0_ACC_GYRO_INT_DUR2  			0x5A
#define LSM6DS0_ACC_GYRO_WAKE_UP_THS  			0x5B
#define LSM6DS0_ACC_GYRO_WAKE_UP_DUR  			0x5C
#define LSM6DS0_ACC_GYRO_FREE_FALL  			0x5D
#define LSM6DS0_ACC_GYRO_MD1_CFG  			0x5E
#define LSM6DS0_ACC_GYRO_MD2_CFG  			0x5F

/************** Access Device RAM  *******************/

#define LSM6DS0_ACC_GYRO_I3C_BUS_AVB         0x62 //Added
#define LSM6DS0_ACC_GYRO_INTERNAL_FREQ_FINE         0x63 //Added
//#define LSM6DS0_ACC_GYRO_ADDR0_TO_RW_RAM         0x62 Removed
//#define LSM6DS0_ACC_GYRO_ADDR1_TO_RW_RAM         0x63 Removed

// #define LSM6DS0_ACC_GYRO_DATA_TO_WR_RAM          0x64 // ---- Reserved ----
// #define LSM6DS0_ACC_GYRO_DATA_RD_FROM_RAM        0x65 // ------^^----------
 
#define LSM6DS0_ACC_GYRO_INT_OIS         0x6F //Added
#define LSM6DS0_ACC_GYRO_CTRL1_OIS         0x70 //Added
#define LSM6DS0_ACC_GYRO_CTRL2_OIS         0x71 //Added
#define LSM6DS0_ACC_GYRO_CTRL3_OIS         0x72 //Added
#define LSM6DS0_ACC_GYRO_X_OFS_USR         0x73 //Added
#define LSM6DS0_ACC_GYRO_Y_OFS_USR         0x74 //Added
#define LSM6DS0_ACC_GYRO_Z_OFS_USR         0x75 //Added

#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_TAG         0x78 //Added
#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_X_L         0x79 //Added
#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_X_H         0x7A //Added
#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Y_L         0x7B //Added
#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Y_H         0x7C //Added
#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Z_L         0x7D //Added
#define LSM6DS0_ACC_GYRO_FIFO_DATA_OUT_Z_H         0x7E //Added

#define LSM6DS0_ACC_GYRO_RAM_SIZE                4096

/************** Embedded functions register mapping  *******************/
#define LSM6DS0_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DS0_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DS0_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DS0_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DS0_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DS0_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DS0_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DS0_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DS0_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DS0_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DS0_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DS0_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DS0_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DS0_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F
#define LSM6DS0_ACC_GYRO_CONFIG_TILT_IIR              0x10
#define LSM6DS0_ACC_GYRO_CONFIG_TILT_ACOS             0x11
#define LSM6DS0_ACC_GYRO_CONFIG_TILT_WTIME            0x12
#define LSM6DS0_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DS0_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DS0_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DS0_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DS0_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DS0_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DS0_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DS0_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DS0_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DS0_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DS0_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DS0_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DS0_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DS0_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DS0_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DS0_ACC_GYRO_MAG_OFFZ_H                   0x32

// Fifo Tags - not a complete list. 
typedef enum {
  GYROSCOPE_DATA = 0x01,
  ACCELEROMETER_DATA,
  TEMPERATURE_DATA,
  TIMESTAMP_DATA,
  CFG_CHANGE_DATA,
  ACCELERTOMETER_DATA_T_2,
  ACCELERTOMETER_DATA_T_1,
  ACCELERTOMETER_DATA_2xC,
  ACCELERTOMETER_DATA_3xC,
  GYRO_DATA_T_2,
  GYRO_DATA_T_1,
  GYRO_DATA_2xC,
  GYRO_DATA_3xC,
} LSM6DS0_FIFO_TAGS_t; 

/*******************************************************************************
* Register      : RAM_ACCESS
* Address       : 0x01
* Bit Group Name: PROG_RAM1
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PROG_RAM1_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_PROG_RAM1_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_PROG_RAM1_t;

/*******************************************************************************
* Register      : RAM_ACCESS
* Address       : 0x01
* Bit Group Name: CUSTOMROM1
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_CUSTOMROM1_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_CUSTOMROM1_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_CUSTOMROM1_t;

/*******************************************************************************
* Register      : RAM_ACCESS
* Address       : 0x01
* Bit Group Name: RAM_PAGE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_RAM_PAGE_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_RAM_PAGE_ENABLED 		 = 0x80
} LSM6DS0_ACC_GYRO_RAM_PAGE_t;


/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0x09
* Bit Group Name: BDR_GY
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_GYRO_FIFO_BDR_NOT_BATCHED = 0x00,
	LSM6DS0_GYRO_FIFO_BDR_12_5        = 0x01,
	LSM6DS0_GYRO_FIFO_BDR_26          = 0x02,
	LSM6DS0_GYRO_FIFO_BDR_52          = 0x03,
	LSM6DS0_GYRO_FIFO_BDR_104         = 0x04,
	LSM6DS0_GYRO_FIFO_BDR_208         = 0x05,
	LSM6DS0_GYRO_FIFO_BDR_417         = 0x06,
	LSM6DS0_GYRO_FIFO_BDR_833         = 0x07,
	LSM6DS0_GYRO_FIFO_BDR_1667        = 0x08,
	LSM6DS0_GYRO_FIFO_BDR_3333        = 0x09,
	LSM6DS0_GYRO_FIFO_BDR_6667        = 0x0A,
	LSM6DS0_GYRO_FIFO_BDR_6_5         = 0x0B
} LSM6DS0_ACC_GYRO_BDR_GY_FIFO_t;

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0x09
* Bit Group Name: BDR_Xl
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_FIFO_BDR_NOT_BATCHED = 0x00,
	LSM6DS0_ACC_FIFO_BDR_12_5        = 0x01,
	LSM6DS0_ACC_FIFO_BDR_26          = 0x02,
	LSM6DS0_ACC_FIFO_BDR_52          = 0x03,
	LSM6DS0_ACC_FIFO_BDR_104         = 0x04,
	LSM6DS0_ACC_FIFO_BDR_208         = 0x05,
	LSM6DS0_ACC_FIFO_BDR_417         = 0x06,
	LSM6DS0_ACC_FIFO_BDR_833         = 0x07,
	LSM6DS0_ACC_FIFO_BDR_1667        = 0x08,
	LSM6DS0_ACC_FIFO_BDR_3333        = 0x09,
	LSM6DS0_ACC_FIFO_BDR_6667        = 0x0A,
	LSM6DS0_ACC_FIFO_BDR_6_5         = 0x0B
} LSM6DS0_ACC_GYRO_BDR_XL_FIFO_t;


/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0x0A
* Bit Group Name: DEC_TS_BATCH
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DEC_FIFO_XL_G_DATA_NOT_IN_FIFO 	= 0x00,
	LSM6DS0_ACC_GYRO_DEC_FIFO_XL_G_DECIMATION_BY_1 		= 0x01, //Default
	LSM6DS0_ACC_GYRO_DEC_FIFO_XL_G_DECIMATION_BY_8 		= 0x02,
	LSM6DS0_ACC_GYRO_DEC_FIFO_XL_G_DECIMATION_BY_32 	= 0x03
} LSM6DS0_ACC_GYRO_DEC_FIFO_XL_G_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0x0A
* Bit Group Name: ODR_T_BATCH_
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_TEMPERATURE_ODR_BATCH_NOT_IN_FIFO = 0x00,
	LSM6DS0_TEMPERATURE_ODR_BATCH_RATE_1_6    = 0x01,
	LSM6DS0_TEMPERATURE_ODR_BATCH_RATE_12_5   = 0x02,
	LSM6DS0_TEMPERATURE_ODR_BATCH_RATE_52     = 0x03
} LSM6DS0_TEMPERATURE_ODR_BATCH_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0x0A
* Bit Group Name: FIFO_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FIFO_MODE_DISABLED       = 0x00,
	LSM6DS0_ACC_GYRO_FIFO_MODE_STOP_WHEN_FULL = 0x01,
	LSM6DS0_ACC_GYRO_FIFO_MODE_CONT_TO_FIFO   = 0x03,
	LSM6DS0_ACC_GYRO_FIFO_MODE_BYPASS_TO_CONT = 0x04,
	LSM6DS0_ACC_GYRO_FIFO_MODE_CONTINUOUS     = 0x06,
	LSM6DS0_ACC_GYRO_FIFO_MODE_BYPASS_TO_FIFO = 0x07
} LSM6DS0_ACC_GYRO_ODR_FIFO_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: ORIENT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_ORIENT_XYZ 		 = 0x00,
	LSM6DS0_ACC_GYRO_ORIENT_XZY 		 = 0x01,
	LSM6DS0_ACC_GYRO_ORIENT_YXZ 		 = 0x02,
	LSM6DS0_ACC_GYRO_ORIENT_YZX 		 = 0x03,
	LSM6DS0_ACC_GYRO_ORIENT_ZXY 		 = 0x04,
	LSM6DS0_ACC_GYRO_ORIENT_ZYX 		 = 0x05,
} LSM6DS0_ACC_GYRO_ORIENT_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: SIGN_Z_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIGN_Z_G_POSITIVE 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIGN_Z_G_NEGATIVE 		 = 0x08,
} LSM6DS0_ACC_GYRO_SIGN_Z_G_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: SIGN_Y_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIGN_Y_G_POSITIVE 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIGN_Y_G_NEGATIVE 		 = 0x10,
} LSM6DS0_ACC_GYRO_SIGN_Y_G_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: SIGN_X_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIGN_X_G_POSITIVE 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIGN_X_G_NEGATIVE 		 = 0x20,
} LSM6DS0_ACC_GYRO_SIGN_X_G_t;

/*******************************************************************************
* Register      : REFERENCE_G
* Address       : 0x0C
* Bit Group Name: REF_G
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_REF_G_MASK  	0xFF
#define  	LSM6DS0_ACC_GYRO_REF_G_POSITION  	0

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_DRDY_XL_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_DRDY_XL_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_INT1_DRDY_XL_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_DRDY_G_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_DRDY_G_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_INT1_DRDY_G_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_BOOT_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_BOOT_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_INT1_BOOT_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_FTH_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_FTH_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_INT1_FTH_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_OVR_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_OVR_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_INT1_OVR_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_FSS5
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_FSS5_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_FSS5_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_INT1_FSS5_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_SIGN_MOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_SIGN_MOT_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_SIGN_MOT_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_INT1_SIGN_MOT_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_PEDO
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_PEDO_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_PEDO_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_INT1_PEDO_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_DRDY_XL_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_DRDY_XL_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_INT2_DRDY_XL_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_DRDY_G_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_DRDY_G_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_INT2_DRDY_G_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_FTH_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_FTH_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_INT2_FTH_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_OVR_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_OVR_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_INT2_OVR_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_FSS5
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_FSS5_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_FSS5_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_INT2_FSS5_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_SIGN_MOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_SIGN_MOT_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_SIGN_MOT_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_INT2_SIGN_MOT_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_PEDO
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_PEDO_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_PEDO_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_INT2_PEDO_t;

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0x0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_WHO_AM_I_BIT_MASK  	0xFF
#define  	LSM6DS0_ACC_GYRO_WHO_AM_I_BIT_POSITION  	0


/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0x10
* Bits          : [1]
* Bit Group Name: LPF2_XL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_LPF2_XL_DISABLE = 0x00,
	LSM6DS0_ACC_LPF2_XL_EN      = 0x02
} LSM6DS0_ACC_LPF2_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0x10
* Bits          : [3:2]
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_FS_XL_2g 		 = 0x00,
  LSM6DS0_ACC_FS_XL_16g  	 = 0x04,
	LSM6DS0_ACC_FS_XL_4g 		 = 0x08,
	LSM6DS0_ACC_FS_XL_8g 		 = 0x0C,
} LSM6DS0_ACC_FS_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0x10
* Bits          : [7:4]
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_ODR_XL_POWER_DOWN = 0x00, // Low Power only
	LSM6DS0_ACC_ODR_XL_1_6Hz      = 0xB0, // Low Power only
	LSM6DS0_ACC_ODR_XL_12_5Hz     = 0x10, // Low Power only
	LSM6DS0_ACC_ODR_XL_26Hz       = 0x20, // Low Power only
	LSM6DS0_ACC_ODR_XL_52Hz       = 0x30, // Low Power only 
	LSM6DS0_ACC_ODR_XL_104Hz      = 0x40, // Normal Mode
	LSM6DS0_ACC_ODR_XL_208Hz      = 0x50, // Normal Mode
	LSM6DS0_ACC_ODR_XL_416Hz      = 0x60, // High performance
	LSM6DS0_ACC_ODR_XL_833Hz      = 0x70, // High Performance 
	LSM6DS0_ACC_ODR_XL_1660Hz    = 0x80, // High Performance
	LSM6DS0_ACC_ODR_XL_3330Hz    = 0x90, // High Performance
	LSM6DS0_ACC_ODR_XL_6660Hz    = 0xA0 // High Performance
} LSM6DS0_ACC_ODR_XL_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0x11
* Bit           : [1]
* Bit Group Name: FS_125
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FS_125_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_FS_125_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_FS_125_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0x11
* Bit           : [3:2]
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FS_G_245dps 		 = 0x00,
	LSM6DS0_ACC_GYRO_FS_G_500dps 		 = 0x04,
	LSM6DS0_ACC_GYRO_FS_G_1000dps 		= 0x08,
	LSM6DS0_ACC_GYRO_FS_G_2000dps 		= 0x0C,
} LSM6DS0_ACC_GYRO_FS_G_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0x11
* Bit           : [7:4]
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_GYRO_ODR_G_DISABLE = 0x00, // Low Power only
	LSM6DS0_GYRO_ODR_G_12_5Hz  = 0x10, // Low Power only
	LSM6DS0_GYRO_ODR_G_26Hz    = 0x20, // Low Power only
	LSM6DS0_GYRO_ODR_G_52Hz    = 0x30, // Low Power only
	LSM6DS0_GYRO_ODR_G_104Hz   = 0x40, // Normal Mode
	LSM6DS0_GYRO_ODR_G_208Hz   = 0x50, // Normal Mode
	LSM6DS0_GYRO_ODR_G_416Hz   = 0x60, // High performance
	LSM6DS0_GYRO_ODR_G_833Hz   = 0x70, // High Performance
	LSM6DS0_GYRO_ODR_G_1660Hz  = 0x80, // High Performance
	LSM6DS0_GYRO_ODR_G_3330Hz  = 0x90, // High Performance
	LSM6DS0_GYRO_ODR_G_6660Hz  = 0xA0 // High Performance
} LSM6DS0_GYRO_ODR_G_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: SW_RESET
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SW_RESET_NORMAL_MODE 		 = 0x00,
	LSM6DS0_ACC_GYRO_SW_RESET_RESET_DEVICE 		 = 0x01,
} LSM6DS0_ACC_GYRO_SW_RESET_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_BLE_LSB 		 = 0x00,
	LSM6DS0_ACC_GYRO_BLE_MSB 		 = 0x02,
} LSM6DS0_ACC_GYRO_BLE_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: IF_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_IF_INC_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_IF_INC_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_IF_INC_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIM_4_WIRE 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIM_3_WIRE 		 = 0x08,
} LSM6DS0_ACC_GYRO_SIM_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PP_OD_PUSH_PULL 		 = 0x00,
	LSM6DS0_ACC_GYRO_PP_OD_OPEN_DRAIN 		 = 0x10,
} LSM6DS0_ACC_GYRO_PP_OD_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: INT_ACT_LEVEL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT_ACT_LEVEL_ACTIVE_HI 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT_ACT_LEVEL_ACTIVE_LO 		 = 0x20,
} LSM6DS0_ACC_GYRO_INT_ACT_LEVEL_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_BDU_CONTINUOS 		 = 0x00,
	LSM6DS0_ACC_GYRO_BDU_BLOCK_UPDATE 		 = 0x40,
} LSM6DS0_ACC_GYRO_BDU_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_BOOT_NORMAL_MODE 		 = 0x00,
	LSM6DS0_ACC_GYRO_BOOT_REBOOT_MODE 		 = 0x80,
} LSM6DS0_ACC_GYRO_BOOT_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: STOP_ON_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_STOP_ON_FTH_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_STOP_ON_FTH_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_STOP_ON_FTH_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: MODE3_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_MODE3_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_MODE3_EN_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_MODE3_EN_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_I2C_DISABLE_I2C_AND_SPI 		 = 0x00,
	LSM6DS0_ACC_GYRO_I2C_DISABLE_SPI_ONLY 		 = 0x04,
} LSM6DS0_ACC_GYRO_I2C_DISABLE_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: DRDY_MSK
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DRDY_MSK_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DRDY_MSK_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_DRDY_MSK_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: FIFO_TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: INT2_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_ON_INT1_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_ON_INT1_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_INT2_ON_INT1_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: SLEEP_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SLEEP_G_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_SLEEP_G_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_SLEEP_G_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: BW_SCAL_ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_BW_SCAL_ODR_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_BW_SCAL_ODR_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t;

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0x14
* Bit Group Name: ST_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_ST_XL_NORMAL_MODE 		 = 0x00,
	LSM6DS0_ACC_GYRO_ST_XL_POS_SIGN_TEST 		 = 0x01,
	LSM6DS0_ACC_GYRO_ST_XL_NEG_SIGN_TEST 		 = 0x02,
	LSM6DS0_ACC_GYRO_ST_XL_NA 		 = 0x03,
} LSM6DS0_ACC_GYRO_ST_XL_t;

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0x14
* Bit Group Name: ST_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_ST_G_NORMAL_MODE 		 = 0x00,
	LSM6DS0_ACC_GYRO_ST_G_POS_SIGN_TEST 		 = 0x04,
	LSM6DS0_ACC_GYRO_ST_G_NA 		 = 0x08,
	LSM6DS0_ACC_GYRO_ST_G_NEG_SIGN_TEST 		 = 0x0C,
} LSM6DS0_ACC_GYRO_ST_G_t;

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0x15
* Bit Group Name: LP_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_LP_XL_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_LP_XL_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_LP_XL_t;

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0x15
* Bit Group Name: DEN_LVL2_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DEN_LVL2_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DEN_LVL2_EN_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_DEN_LVL2_EN_t;

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0x15
* Bit Group Name: DEN_LVL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DEN_LVL_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DEN_LVL_EN_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_DEN_LVL_EN_t;

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0x15
* Bit Group Name: DEN_EDGE_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DEN_EDGE_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DEN_EDGE_EN_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_DEN_EDGE_EN_t;

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0x16
* Bit Group Name: HPM_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_HPM_G_NORMAL_MODE 		 = 0x00,
	LSM6DS0_ACC_GYRO_HPM_G_REF_SIGNAL 		 = 0x10,
	LSM6DS0_ACC_GYRO_HPM_G_NORMAL_MODE_2 		 = 0x20,
	LSM6DS0_ACC_GYRO_HPM_G_AUTO_RESET_ON_INT 		 = 0x30,
} LSM6DS0_ACC_GYRO_HPM_G_t;

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0x16
* Bit Group Name: HP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_HP_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_HP_EN_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_HP_EN_t;

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0x16
* Bit Group Name: LP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_LP_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_LP_EN_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_LP_EN_t;

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0x17
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FDS_FILTER_OFF 		 = 0x00,
	LSM6DS0_ACC_GYRO_FDS_FILTER_ON 		 = 0x04,
} LSM6DS0_ACC_GYRO_FDS_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0x18
* Bit Group Name: XEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_XEN_XL_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_XEN_XL_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_XEN_XL_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0x18
* Bit Group Name: YEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_YEN_XL_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_YEN_XL_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_YEN_XL_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0x18
* Bit Group Name: ZEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_ZEN_XL_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_ZEN_XL_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_ZEN_XL_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: SIGN_MOTION_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIGN_MOTION_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIGN_MOTION_EN_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_SIGN_MOTION_EN_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: PEDO_RST_STEP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PEDO_RST_STEP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_PEDO_RST_STEP_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_PEDO_RST_STEP_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: XEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_XEN_G_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_XEN_G_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_XEN_G_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: YEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_YEN_G_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_YEN_G_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_YEN_G_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: ZEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_ZEN_G_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_ZEN_G_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_ZEN_G_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: FUNC_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FUNC_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_FUNC_EN_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_FUNC_EN_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: MASTER_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_MASTER_ON_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_MASTER_ON_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_MASTER_ON_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: IRON_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_IRON_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_IRON_EN_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_IRON_EN_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: PASS_THRU_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PASS_THRU_MODE_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_PASS_THRU_MODE_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_PASS_THRU_MODE_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: PULL_UP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PULL_UP_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_PULL_UP_EN_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_PULL_UP_EN_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: START_CONFIG
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_START_CONFIG_XL_G_DRDY 		 = 0x00,
	LSM6DS0_ACC_GYRO_START_CONFIG_EXT_INT2 		 = 0x10,
} LSM6DS0_ACC_GYRO_START_CONFIG_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: DATA_VAL_SEL_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DATA_VAL_SEL_FIFO_XL_G_DRDY 		 = 0x00,
	LSM6DS0_ACC_GYRO_DATA_VAL_SEL_FIFO_SHUB_DRDY 		 = 0x40,
} LSM6DS0_ACC_GYRO_DATA_VAL_SEL_FIFO_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: DRDY_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DRDY_ON_INT1_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DRDY_ON_INT1_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_DRDY_ON_INT1_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: Z_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_Z_WU_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_Z_WU_DETECTED 		 = 0x01,
} LSM6DS0_ACC_GYRO_Z_WU_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: Y_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_Y_WU_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_Y_WU_DETECTED 		 = 0x02,
} LSM6DS0_ACC_GYRO_Y_WU_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: X_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_X_WU_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_X_WU_DETECTED 		 = 0x04,
} LSM6DS0_ACC_GYRO_X_WU_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: WU_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_WU_EV_STATUS_DETECTED 		 = 0x08,
} LSM6DS0_ACC_GYRO_WU_EV_STATUS_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: SLEEP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SLEEP_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_SLEEP_EV_STATUS_DETECTED 		 = 0x10,
} LSM6DS0_ACC_GYRO_SLEEP_EV_STATUS_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: FF_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FF_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_FF_EV_STATUS_DETECTED 		 = 0x20,
} LSM6DS0_ACC_GYRO_FF_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: Z_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_Z_TAP_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_Z_TAP_DETECTED 		 = 0x01,
} LSM6DS0_ACC_GYRO_Z_TAP_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: Y_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_Y_TAP_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_Y_TAP_DETECTED 		 = 0x02,
} LSM6DS0_ACC_GYRO_Y_TAP_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: X_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_X_TAP_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_X_TAP_DETECTED 		 = 0x04,
} LSM6DS0_ACC_GYRO_X_TAP_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: TAP_SIGN
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TAP_SIGN_POS_SIGN 		 = 0x00,
	LSM6DS0_ACC_GYRO_TAP_SIGN_NEG_SIGN 		 = 0x08,
} LSM6DS0_ACC_GYRO_TAP_SIGN_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: DOUBLE_TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DOUBLE_TAP_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DOUBLE_TAP_EV_STATUS_DETECTED 		 = 0x10,
} LSM6DS0_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: SINGLE_TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SINGLE_TAP_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_SINGLE_TAP_EV_STATUS_DETECTED 		 = 0x20,
} LSM6DS0_ACC_GYRO_SINGLE_TAP_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TAP_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TAP_EV_STATUS_DETECTED 		 = 0x40,
} LSM6DS0_ACC_GYRO_TAP_EV_STATUS_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_XL
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DSD_XL_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DSD_XL_DETECTED 		 = 0x01,
} LSM6DS0_ACC_GYRO_DSD_XL_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_XH
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DSD_XH_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DSD_XH_DETECTED 		 = 0x02,
} LSM6DS0_ACC_GYRO_DSD_XH_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_YL
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DSD_YL_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DSD_YL_DETECTED 		 = 0x04,
} LSM6DS0_ACC_GYRO_DSD_YL_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_YH
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DSD_YH_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DSD_YH_DETECTED 		 = 0x08,
} LSM6DS0_ACC_GYRO_DSD_YH_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DSD_ZL_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DSD_ZL_DETECTED 		 = 0x10,
} LSM6DS0_ACC_GYRO_DSD_ZL_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_DSD_ZH_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_DSD_ZH_DETECTED 		 = 0x20,
} LSM6DS0_ACC_GYRO_DSD_ZH_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: D6D_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_D6D_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_D6D_EV_STATUS_DETECTED 		 = 0x40,
} LSM6DS0_ACC_GYRO_D6D_EV_STATUS_t;

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0x1E
* Bit Group Name: XLDA
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_XLDA_NO_DATA_AVAIL 		 = 0x00,
	LSM6DS0_ACC_GYRO_XLDA_DATA_AVAIL 		 = 0x01,
} LSM6DS0_ACC_GYRO_XLDA_t;

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0x1E
* Bit Group Name: GDA
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_GDA_NO_DATA_AVAIL 		 = 0x00,
	LSM6DS0_ACC_GYRO_GDA_DATA_AVAIL 		 = 0x02,
} LSM6DS0_ACC_GYRO_GDA_t;

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0x1E
* Bit Group Name: EV_BOOT
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_EV_BOOT_NO_BOOT_RUNNING 		 = 0x00,
	LSM6DS0_ACC_GYRO_EV_BOOT_BOOT_IS_RUNNING 		 = 0x08,
} LSM6DS0_ACC_GYRO_EV_BOOT_t;

/*******************************************************************************
* Register      : FIFO_STATUS1
* Address       : 0x3A
* Bit Group Name: DIFF_FIFO
* Permission    : RO
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_DIFF_FIFO_STATUS1_MASK  	0xFF
#define  	LSM6DS0_ACC_GYRO_DIFF_FIFO_STATUS1_POSITION  	0
#define  	LSM6DS0_ACC_GYRO_DIFF_FIFO_STATUS2_MASK  0xF
#define  	LSM6DS0_ACC_GYRO_DIFF_FIFO_STATUS2_POSITION  	0

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: FIFO_EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FIFO_EMPTY_FIFO_NOT_EMPTY 		 = 0x00,
	LSM6DS0_ACC_GYRO_FIFO_EMPTY_FIFO_EMPTY 		 = 0x10,
} LSM6DS0_ACC_GYRO_FIFO_EMPTY_t;

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: FIFO_FULL
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FIFO_FULL_FIFO_NOT_FULL 		 = 0x00,
	LSM6DS0_ACC_GYRO_FIFO_FULL_FIFO_FULL 		 = 0x20,
} LSM6DS0_ACC_GYRO_FIFO_FULL_t;

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: OVERRUN
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_OVERRUN_NO_OVERRUN 		 = 0x00,
	LSM6DS0_ACC_GYRO_OVERRUN_OVERRUN 		 = 0x40,
} LSM6DS0_ACC_GYRO_OVERRUN_t;

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_WTM_BELOW_WTM 		 = 0x00,
	LSM6DS0_ACC_GYRO_WTM_ABOVE_OR_EQUAL_WTM 		 = 0x80,
} LSM6DS0_ACC_GYRO_WTM_t;

/*******************************************************************************
* Register      : FIFO_STATUS3
* Address       : 0x3C
* Bit Group Name: FIFO_PATTERN
* Permission    : RO
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_FIFO_STATUS3_PATTERN_MASK  	0xFF
#define  	LSM6DS0_ACC_GYRO_FIFO_STATUS3_PATTERN_POSITION  	0
#define  	LSM6DS0_ACC_GYRO_FIFO_STATUS4_PATTERN_MASK  	0x03
#define  	LSM6DS0_ACC_GYRO_FIFO_STATUS4_PATTERN_POSITION  	0

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: SENS_HUB_END
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SENS_HUB_END_STILL_ONGOING 		 = 0x00,
	LSM6DS0_ACC_GYRO_SENS_HUB_END_OP_COMPLETED 		 = 0x01,
} LSM6DS0_ACC_GYRO_SENS_HUB_END_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: SOFT_IRON_END
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SOFT_IRON_END_NOT_COMPLETED 		 = 0x00,
	LSM6DS0_ACC_GYRO_SOFT_IRON_END_COMPLETED 		 = 0x02,
} LSM6DS0_ACC_GYRO_SOFT_IRON_END_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: PEDO_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PEDO_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_PEDO_EV_STATUS_DETECTED 		 = 0x10,
} LSM6DS0_ACC_GYRO_PEDO_EV_STATUS_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: TILT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TILT_EV_STATUS_DETECTED 		 = 0x20,
} LSM6DS0_ACC_GYRO_TILT_EV_STATUS_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: SIGN_MOT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIGN_MOT_EV_STATUS_NOT_DETECTED 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIGN_MOT_EV_STATUS_DETECTED 		 = 0x40,
} LSM6DS0_ACC_GYRO_SIGN_MOT_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_LIR_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_LIR_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_LIR_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: TAP_Z_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TAP_Z_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TAP_Z_EN_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_TAP_Z_EN_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: TAP_Y_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TAP_Y_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TAP_Y_EN_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_TAP_Y_EN_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TAP_X_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TAP_X_EN_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_TAP_X_EN_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: TILT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TILT_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TILT_EN_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_TILT_EN_t;

/*******************************************************************************
* Register      : EMB_FUNC_EN_A
* Address       : 0x04
* Bit Group Name: PEDO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_PEDO_EN_DISABLED 	 = 0x00,
	LSM6DS0_ACC_GYRO_PEDO_EN_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_PEDO_EN_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: TIMER_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TIMER_EN_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_TIMER_EN_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_TIMER_EN_t;

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0x59
* Bit Group Name: TAP_THS
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_TAP_THS_MASK  	0x1F
#define  	LSM6DS0_ACC_GYRO_TAP_THS_POSITION  	0

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0x59
* Bit Group Name: SIXD_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SIXD_THS_80_degree 		 = 0x00,
	LSM6DS0_ACC_GYRO_SIXD_THS_70_degree 		 = 0x20,
	LSM6DS0_ACC_GYRO_SIXD_THS_60_degree 		 = 0x40,
	LSM6DS0_ACC_GYRO_SIXD_THS_50_degree 		 = 0x60,
} LSM6DS0_ACC_GYRO_SIXD_THS_t;

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0x5A
* Bit Group Name: SHOCK
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_SHOCK_MASK  	0x03
#define  	LSM6DS0_ACC_GYRO_SHOCK_POSITION  	0

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0x5A
* Bit Group Name: QUIET
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_QUIET_MASK  	0x0C
#define  	LSM6DS0_ACC_GYRO_QUIET_POSITION  	2

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0x5A
* Bit Group Name: DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_DUR_MASK  	0xF0
#define  	LSM6DS0_ACC_GYRO_DUR_POSITION  	4

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x5B
* Bit Group Name: WK_THS
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_WK_THS_MASK  	0x3F
#define  	LSM6DS0_ACC_GYRO_WK_THS_POSITION  	0

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x5B
* Bit Group Name: INACTIVITY_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INACTIVITY_ON_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INACTIVITY_ON_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_INACTIVITY_ON_t;

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x5B
* Bit Group Name: SINGLE_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP 		 = 0x00,
	LSM6DS0_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP 		 = 0x80,
} LSM6DS0_ACC_GYRO_SINGLE_DOUBLE_TAP_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x5C
* Bit Group Name: SLEEP_DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_SLEEP_DUR_MASK  	0x0F
#define  	LSM6DS0_ACC_GYRO_SLEEP_DUR_POSITION  	0

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x5C
* Bit Group Name: TIMER_HR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_TIMER_HR_6_4ms 		 = 0x00,
	LSM6DS0_ACC_GYRO_TIMER_HR_25us 		 = 0x10,
} LSM6DS0_ACC_GYRO_TIMER_HR_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x5C
* Bit Group Name: WAKE_DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_WAKE_DUR_MASK  	0x60
#define  	LSM6DS0_ACC_GYRO_WAKE_DUR_POSITION  	5

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0x5D
* Bit Group Name: FF_DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS0_ACC_GYRO_FF_FREE_FALL_DUR_MASK  	0xF8
#define  	LSM6DS0_ACC_GYRO_FF_FREE_FALL_DUR_POSITION  	3
#define  	LSM6DS0_ACC_GYRO_FF_WAKE_UP_DUR_MASK  	0x80
#define  	LSM6DS0_ACC_GYRO_FF_WAKE_UP_DUR_POSITION  	7


/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0x5D
* Bit Group Name: FF_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_FF_THS_5 		 = 0x00,
	LSM6DS0_ACC_GYRO_FF_THS_7 		 = 0x01,
	LSM6DS0_ACC_GYRO_FF_THS_8 		 = 0x02,
	LSM6DS0_ACC_GYRO_FF_THS_10 		 = 0x03,
	LSM6DS0_ACC_GYRO_FF_THS_11 		 = 0x04,
	LSM6DS0_ACC_GYRO_FF_THS_13 		 = 0x05,
	LSM6DS0_ACC_GYRO_FF_THS_15 		 = 0x06,
	LSM6DS0_ACC_GYRO_FF_THS_16 		 = 0x07,
} LSM6DS0_ACC_GYRO_FF_THS_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_TIMER
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_TIMER_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_TIMER_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_INT1_TIMER_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_TILT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_TILT_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_TILT_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_INT1_TILT_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_6D_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_6D_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_INT1_6D_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_TAP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_TAP_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_INT1_TAP_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_FF_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_FF_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_INT1_FF_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_WU_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_WU_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_INT1_WU_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_SINGLE_TAP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_SINGLE_TAP_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_INT1_SINGLE_TAP_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_SLEEP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT1_SLEEP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT1_SLEEP_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_INT1_SLEEP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_TIMER
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_TIMER_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_TIMER_ENABLED 		 = 0x01,
} LSM6DS0_ACC_GYRO_INT2_TIMER_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_TILT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_TILT_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_TILT_ENABLED 		 = 0x02,
} LSM6DS0_ACC_GYRO_INT2_TILT_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_6D_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_6D_ENABLED 		 = 0x04,
} LSM6DS0_ACC_GYRO_INT2_6D_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_TAP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_TAP_ENABLED 		 = 0x08,
} LSM6DS0_ACC_GYRO_INT2_TAP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_FF_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_FF_ENABLED 		 = 0x10,
} LSM6DS0_ACC_GYRO_INT2_FF_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_WU_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_WU_ENABLED 		 = 0x20,
} LSM6DS0_ACC_GYRO_INT2_WU_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_SINGLE_TAP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_SINGLE_TAP_ENABLED 		 = 0x40,
} LSM6DS0_ACC_GYRO_INT2_SINGLE_TAP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_SLEEP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS0_ACC_GYRO_INT2_SLEEP_DISABLED 		 = 0x00,
	LSM6DS0_ACC_GYRO_INT2_SLEEP_ENABLED 		 = 0x80,
} LSM6DS0_ACC_GYRO_INT2_SLEEP_t;

#endif  // End of __LSM6DS0IMU_H__ definition check
