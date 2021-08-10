/******************************************************************************
SparkFunLSM6DSO.h
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
Arduino IDE 1.8.1
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LSM6DSOIMU_H__
#define __LSM6DSOIMU_H__

#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

#define I2C_MODE 0
#define SPI_MODE 1
#define SPI_READ_COMMAND 0x80
#define DEFAULT_ADDRESS 0x6B
#define ALT_ADDRESS 0x6A

// Return values 
typedef enum
{
  IMU_SUCCESS = 0x00,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	IMU_GENERIC_ERROR = 0xFF,
} status_t;

//  This is the core operational class of the driver.
//  LSM6DSOCore contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LSM6DSO which inherits
//  this class.

class LSM6DSOCore
{
public:

	LSM6DSOCore();
	status_t beginCore(uint8_t, TwoWire &i2cPort );
	status_t beginSPICore(uint8_t, uint32_t, SPIClass &spiPort );
	
	status_t readMultipleRegisters(uint8_t*, uint8_t, uint8_t );
	status_t readRegister(uint8_t*, uint8_t);
	status_t readRegisterInt16(int16_t*, uint8_t);
	status_t writeRegister(uint8_t, uint8_t);
	status_t writeMultipleRegisters(uint8_t*, uint8_t, uint8_t);
  status_t enableEmbeddedFunctions(bool = true);

  SPISettings mySpiSettings; 
	
private:

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
	bool gyroEnabled;
	uint16_t gyroRange;
	uint16_t gyroSampleRate;
	uint16_t gyroBandWidth;

	uint8_t gyroFifoEnabled;
	uint8_t gyroAccelDecimation;

	//Accelerometer settings
	bool accelEnabled;
	uint8_t accelODROff;
	uint16_t accelRange;
	uint16_t accelSampleRate;
	uint16_t accelBandWidth;
	
	uint8_t accelFifoEnabled;
	
	
	//Non-basic mode settings
	uint8_t commMode;
	
	//FIFO control data
  bool fifoEnabled;  
	uint16_t fifoThreshold;
	int16_t fifoSampleRate;
	uint8_t fifoModeWord;
	
};

struct fifoData{ 
public:
  uint8_t fifoTag;
  float xAccel; 
  float yAccel; 
  float zAccel; 

  float xGyro; 
  float yGyro; 
  float zGyro; 

  float temperatureC; 
  float temperatureF; 
};



//This is the highest level class of the driver.
//LSM6DSO inherits LSM6DSOcore and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

#define ACCEL_DATA_READY 0x01
#define GYRO_DATA_READY 0x02
#define TEMP_DATA_READY 0x04
#define ALL_DATA_READY 0x07

#define BASIC_SETTINGS 0x00
#define SOFT_INT_SETTINGS 0x01
#define HARD_INT_SETTINGS 0x02
#define FIFO_SETTINGS 0x03
#define PEDOMETER_SETTINGS 0x04
#define TAP_SETTINGS 0x05
#define FREE_FALL_SETTINGS 0x06

class LSM6DSO : public LSM6DSOCore
{
  public:
    //IMU settings
    SensorSettings imuSettings;

    //Error checking
    uint16_t allOnesCounter;
    uint16_t nonSuccessCounter;

    LSM6DSO();
    bool begin(uint8_t deviceAddress = DEFAULT_ADDRESS, TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 10000000, SPIClass &spiPort = SPI );
    bool initialize(uint8_t settings = BASIC_SETTINGS);
    status_t beginSettings();


    bool setAccelRange(uint8_t) ;
    bool setAccelDataRate(uint16_t) ;
    bool setGyroDataRate(uint16_t);
    bool setGyroRange(uint16_t) ;
    bool setBlockDataUpdate(bool);
    bool setHighPerfAccel(bool);
    bool setHighPerfGyro(bool);

    uint8_t  getAccelRange();
    float    getAccelDataRate();
    float    getGyroDataRate();
    uint16_t getGyroRange();
    uint8_t  listenDataReady();
    uint8_t  getAccelFullScale();
    uint8_t  getAccelHighPerf();

    int16_t readRawAccelX();
    int16_t readRawAccelY();
    int16_t readRawAccelZ();
    int16_t readRawGyroX();
    int16_t readRawGyroY();
    int16_t readRawGyroZ();

    float readFloatAccelX();
    float readFloatAccelY();
    float readFloatAccelZ();
    float readFloatGyroX();
    float readFloatGyroY();
    float readFloatGyroZ();

    bool setInterruptOne(uint8_t);
    uint8_t getInterruptOne(); 
    bool configHardOutInt(uint8_t, uint8_t pushOrDrain = 0x00) ;
    bool setInterruptTwo(uint8_t);
    int16_t readRawTemp();
    float readTempC();
    float readTempF();

    void fifoBeginSettings();
    bool setFifoMode(uint8_t);
    uint8_t getFifoMode();
    bool setFifoDepth(uint16_t);
    uint16_t getFifoDepth();
    bool setAccelBatchDataRate(uint16_t);
    float getAccelBatchDataRate();
    bool setGyroBatchDataRate(uint16_t);
    float getGyroBatchDataRate();
    void fifoClear();
    fifoData fifoRead();
    uint16_t getFifoStatus();
    void fifoEnd();
    
    float calcGyro( int16_t );
    float calcAccel( int16_t );

    bool enablePedometer(bool enable = true);
    uint8_t getPedometer();
    uint8_t getSteps();
    bool resetSteps();
    bool enableTap(bool enable = true, bool xEnable = true, bool yEnable = false, bool zEnable = false) ;
    bool setTapDirPrior(uint8_t);
    uint8_t getTapDirPrior();
    bool setTapClearOnRead(bool = true);
    uint8_t getTapClearOnRead();
    uint8_t clearTapInt();
    bool setXThreshold(uint8_t);
    bool listenStep();
    bool configureTap(uint8_t);

    bool routeHardInterOne(uint8_t) ;
    bool routeHardInterTwo(uint8_t);
    bool setIncrement(bool enable = true) ;
    bool softwareReset();
    uint8_t clearAllInt();

  private:

};

enum LSM6DSO_REGISTERS {

  FUNC_CFG_ACCESS        = 0x01,  
  LSM6DO_PIN_CTRL        = 0x02, 

  FIFO_CTRL1             = 0x07,
  FIFO_CTRL2             = 0x08,
  FIFO_CTRL3             = 0x09,
  FIFO_CTRL4             = 0x0A,

  COUNTER_BDR_REG1       = 0x0B,   
  COUNTER_BDR_REG2       = 0x0C,  

  INT1_CTRL              = 0x0D,
  INT2_CTRL              = 0x0E,
  WHO_AM_I_REG           = 0x0F,
  CTRL1_XL               = 0x10,
  CTRL2_G                = 0x11,
  CTRL3_C                = 0x12,
  CTRL4_C                = 0x13,
  CTRL5_C                = 0x14,
  CTRL6_C                = 0x15,
  CTRL7_G                = 0x16,
  CTRL8_XL               = 0x17,
  CTRL9_XL               = 0x18,
  CTRL10_C               = 0x19,
  ALL_INT_SRC            = 0x1A,
  WAKE_UP_SRC            = 0x1B,
  TAP_SRC                = 0x1C,
  D6D_SRC                = 0x1D,
  STATUS_REG             = 0x1E,
  OUT_TEMP_L             = 0x20,
  OUT_TEMP_H             = 0x21,
  OUTX_L_G               = 0x22,
  OUTX_H_G               = 0x23,
  OUTY_L_G               = 0x24,
  OUTY_H_G               = 0x25,
  OUTZ_L_G               = 0x26,
  OUTZ_H_G               = 0x27,

  OUTX_L_A               = 0x28,
  OUTX_H_A               = 0x29,
  OUTY_L_A               = 0x2A,
  OUTY_H_A               = 0x2B,
  OUTZ_L_A               = 0x2C,
  OUTZ_H_A               = 0x2D,

  EMB_FUNC_STATUS_MP     = 0x35,
  FSM_FUNC_STATUS_A_MP   = 0x36,
  FSM_FUNC_STATUS_B_MP   = 0x37,
  STATUS_MASTER_MAINPAGE = 0x39,

  FIFO_STATUS1           = 0x3A,
  FIFO_STATUS2           = 0x3B,

  TIMESTAMP0_REG         = 0x40,
  TIMESTAMP1_REG         = 0x41,
  TIMESTAMP2_REG         = 0x42,
  TIMESTAMP3_REG         = 0x43,

  TAP_CFG0               = 0x56,  
  TAP_CFG1               = 0x57,   
  TAP_CFG2               = 0x58, 
  TAP_THS_6D             = 0x59,
  INT_DUR2               = 0x5A,
  WAKE_UP_THS            = 0x5B,
  WAKE_UP_DUR            = 0x5C,
  FREE_FALL              = 0x5D,
  MD1_CFG                = 0x5E,
  MD2_CFG                = 0x5F,

  I3C_BUS_AVB            = 0x62,   
  INTERNAL_FREQ_FINE     = 0x63,  


  INT_OIS                = 0x6F,  
  CTRL1_OIS              = 0x70,  
  CTRL2_OIS              = 0x71,  
  CTRL3_OIS              = 0x72,  
  X_OFS_USR              = 0x73,  
  Y_OFS_USR              = 0x74,  
  Z_OFS_USR              = 0x75,  

  FIFO_DATA_OUT_TAG      = 0x78,  
  FIFO_DATA_OUT_X_L      = 0x79,  
  FIFO_DATA_OUT_X_H      = 0x7A,  
  FIFO_DATA_OUT_Y_L      = 0x7B,  
  FIFO_DATA_OUT_Y_H      = 0x7C,  
  FIFO_DATA_OUT_Z_L      = 0x7D,  
  FIFO_DATA_OUT_Z_H      = 0x7E,  

};

#define GYRO_RAM_SIZE 4096

enum EMBEDDED_REGISTERS {

  PAGE_SEL               = 0x02,
  EMB_FUNC_EN_A          = 0x04,
  EMB_FUNC_EN_B          = 0x05,
  PAGE_ADDRESS           = 0x08,
  PAGE_VALUE             = 0x09,
  EMB_FUNC_INT1          = 0x0A,
  FSM_INT1_A             = 0x0B,
  FSM_INT1_B             = 0x0C,
  EMB_FUNC_INT2          = 0x0E,
  FSM_INT2_A             = 0x0F,
  FSM_INT2_B             = 0x10,
  EMB_FUNC_STATUS        = 0x12,
  FSM_STATUS_A           = 0x13,
  FSM_STATUS_B           = 0x14,
  PAGE_RW                = 0x17,
  // RESERVED            = 0x18-0x43,
  EMB_FUNC_FIFO_CFG      = 0x44,
  FSM_ENABLE_A           = 0x46,
  FSM_ENABLE_B           = 0x47,
  FSM_LONG_COUNTER_L     = 0x48,
  FSM_LONG_COUNTER_H     = 0x49,
  FSM_LONG_COUNTER_CLEAR = 0x4A,
  FSM_OUTS1              = 0x4C,
  FSM_OUTS2              = 0x4D,
  FSM_OUTS3              = 0x4E,
  FSM_OUTS4              = 0x4F,
  FSM_OUTS5              = 0x50,
  FSM_OUTS6              = 0x51,
  FSM_OUTS7              = 0x52,
  FSM_OUTS8              = 0x53,
  FSM_OUTS9              = 0x54,
  FSM_OUTS10             = 0x55,
  FSM_OUTS11             = 0x56,
  FSM_OUTS12             = 0x57,
  FSM_OUTS13             = 0x58,
  FSM_OUTS14             = 0x59,
  FSM_OUTS15             = 0x5A,
  FSM_OUTS16             = 0x5B,
  //RESERVED             = 0x5E
  EMB_FUNC_ODR_CFG_B     = 0x5F,
  STEP_COUNTER_L         = 0x62,
  STEP_COUNTER_H         = 0x63,
  EMB_FUNC_SRC           = 0x64,
  EMB_FUNC_INIT_A        = 0x66,
  EMB_FUNC_INIT_B        = 0x67

};

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

} LSM6DSO_FIFO_TAGS_t; 

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0x08
* Bit Group Name: STOP_ON_WTM
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_STOP_ON_WTM_DISABLED = 0x00,
	FIFO_STOP_ON_WTM_ENABLED = 0x01,
	FIFO_STOP_ON_WTM_MASK     = 0x7F
} LSM6DSO_STOP_ON_WTM_t;

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0x08
* Bit Group Name: FIFO_COMPR_RT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_COMPR_RT_DISABLED = 0x00,
	FIFO_COMPR_RT_ENABLE   = 0x01,
	FIFO_COMPR_RT_MASK     = 0xBF
} LSM6DSO_FIFO_COMPR_RT_t;


/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0x08
* Bit Group Name: ODRCHG_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_ODRCHG_DISABLED = 0x00,
	FIFO_ODRCHG_ENABLE   = 0x01,
	FIFO_ODRCHG_MASK     = 0xEF
} LSM6DSO_FIFO_ODRCHG_t;


/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0x08
* Bit Group Name: UNCOPTR_RATE
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_UNCOPTR_RATE_DISABLED = 0x00,
	FIFO_UNCOPTR_RATE_8    = 0x02,
	FIFO_UNCOPTR_RATE_16   = 0x04,
	FIFO_UNCOPTR_RATE_32   = 0x06,
	FIFO_UNCOPTR_RATE_MASK   = 0xF9
} LSM6DSO_FIFO_UNCOPTR_RATE_t;

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0x09
* Bit Group Name: BDR_GY
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_BDR_GYRO_NOT_BATCHED   = 0x00,
	FIFO_BDR_GYRO_12_5Hz        = 0x10,
	FIFO_BDR_GYRO_26Hz          = 0x20,
	FIFO_BDR_GYRO_52Hz          = 0x30,
	FIFO_BDR_GYRO_104Hz         = 0x40,
	FIFO_BDR_GYRO_208Hz         = 0x50,
	FIFO_BDR_GYRO_417Hz         = 0x60,
	FIFO_BDR_GYRO_833Hz         = 0x70,
	FIFO_BDR_GYRO_1667Hz        = 0x80,
	FIFO_BDR_GYRO_3333Hz        = 0x90,
	FIFO_BDR_GYRO_6667Hz        = 0xA0,
	FIFO_BDR_GYRO_6_5Hz         = 0xB0,
	FIFO_BDR_GYRO_MASK          = 0x0F
} LSM6DSO_BDR_GY_FIFO_t;

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0x09
* Bit Group Name: BDR_Xl
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_BDR_ACC_NOT_BATCHED   = 0x00,
	FIFO_BDR_ACC_12_5Hz        = 0x01,
	FIFO_BDR_ACC_26Hz          = 0x02,
	FIFO_BDR_ACC_52Hz          = 0x03,
	FIFO_BDR_ACC_104Hz         = 0x04,
	FIFO_BDR_ACC_208Hz         = 0x05,
	FIFO_BDR_ACC_417Hz         = 0x06,
	FIFO_BDR_ACC_833Hz         = 0x07,
	FIFO_BDR_ACC_1667Hz        = 0x08,
	FIFO_BDR_ACC_3333Hz        = 0x09,
	FIFO_BDR_ACC_6667Hz        = 0x0A,
	FIFO_BDR_ACC_1_6Hz         = 0x0B,
  FIFO_BDR_ACC_MASK          = 0xF0
} LSM6DSO_BDR_XL_FIFO_t;


/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0x0A
* Bit Group Name: DEC_TS_BATCH
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_TS_DEC_DISABLED = 0x00,
	FIFO_TS_DEC_BY_1 		 = 0x04,
	FIFO_TS_DEC_BY_8 		 = 0x08,
	FIFO_TS_DEC_BY_32 	 = 0x0C
} LSM6DSO_FIFO_TS_DEC_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0x0A
* Bit Group Name: ODR_T_BATCH_
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_TEMP_ODR_DISABLE = 0x00,
	FIFO_TEMP_ODR_1_6    = 0x10,
	FIFO_TEMP_ODR_12_5   = 0x20,
	FIFO_TEMP_ORD_52     = 0x30
} LSM6DSO_TEMPERATURE_ODR_BATCH_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0x0A
* Bit Group Name: FIFO_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_MODE_DISABLED       = 0x00,
	FIFO_MODE_STOP_WHEN_FULL = 0x01,
	FIFO_MODE_CONT_TO_FIFO   = 0x03,
	FIFO_MODE_BYPASS_TO_CONT = 0x04,
	FIFO_MODE_CONTINUOUS     = 0x06,
	FIFO_MODE_BYPASS_TO_FIFO = 0x07,
  FIFO_MODE_MASK           = 0xF0 
} LSM6DSO_FIFO_MODE_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: ORIENT
* Permission    : RW
*******************************************************************************/
typedef enum {
	ORIENT_XYZ 		 = 0x00,
	ORIENT_XZY 		 = 0x01,
	ORIENT_YXZ 		 = 0x02,
	ORIENT_YZX 		 = 0x03,
	ORIENT_ZXY 		 = 0x04,
	ORIENT_ZYX 		 = 0x05
} LSM6DSO_ORIENT_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: SIGN_Z_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	SIGN_Z_G_POSITIVE 		 = 0x00,
	SIGN_Z_G_NEGATIVE 		 = 0x08,
} LSM6DSO_SIGN_Z_G_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: SIGN_Y_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	SIGN_Y_G_POSITIVE 		 = 0x00,
	SIGN_Y_G_NEGATIVE 		 = 0x10,
} LSM6DSO_SIGN_Y_G_t;

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0x0B
* Bit Group Name: SIGN_X_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	SIGN_X_G_POSITIVE 		 = 0x00,
	SIGN_X_G_NEGATIVE 		 = 0x20,
} LSM6DSO_SIGN_X_G_t;

/*******************************************************************************
* Register      : REFERENCE_G
* Address       : 0x0C
* Bit Group Name: REF_G
* Permission    : RW
*******************************************************************************/
#define  	REF_G_MASK  	0xFF
#define  	REF_G_POSITION  	0

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_DRDY_XL_DISABLED 		 = 0x00,
	INT1_DRDY_XL_ENABLED 		 = 0x01,
} LSM6DSO_INT1_DRDY_XL_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_DRDY_G_DISABLED 		 = 0x00,
	INT1_DRDY_G_ENABLED 		 = 0x02,
} LSM6DSO_INT1_DRDY_G_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_BOOT_DISABLED 		 = 0x00,
	INT1_BOOT_ENABLED 		 = 0x04,
} LSM6DSO_INT1_BOOT_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_FULL_TH
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_FIFO_TH_DISABLED    = 0x00,
	INT1_FIFO_TH_ENABLED 		 = 0x08,
} LSM6DSO_INT1_TH_FULL_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_FIFO_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_FIFO_OVR_DISABLED 		 = 0x00,
	INT1_FIFO_OVR_ENABLED 		 = 0x10,
} LSM6DSO_INT1_FIFO_OVR_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_FIFO_FULL
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_FIFO_FULL_DISABLED 		 = 0x00,
	INT1_FIFO_FULL_ENABLED 		   = 0x20,
} LSM6DSO_INT1_FIFO_FULL_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0x0D
* Bit Group Name: INT1_CNT_BDR
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_CNT_BDR_DISABLED 		 = 0x00,
	INT1_CNT_BDR_ENABLED 		 = 0x40,
} LSM6DSO_INT1_CNT_BDR_t;


/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_DRDY_XL_DISABLED 		 = 0x00,
	INT2_DRDY_XL_ENABLED 		 = 0x01,
} LSM6DSO_INT2_DRDY_XL_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_DRDY_G_DISABLED 		 = 0x00,
	INT2_DRDY_G_ENABLED 		 = 0x02,
} LSM6DSO_INT2_DRDY_G_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_DRDY_TEMP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_DRDY_TEMP_DISABLED 		 = 0x00,
	INT2_DRDY_TEMP_ENABLED 		   = 0x04,
} LSM6DSO_INT2_DRDY_TEMP_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_FIFO_TH
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_FIFO_TH_DISABLED 		 = 0x00,
	INT2_FIFO_TH_ENABLED 		   = 0x08,
} LSM6DSO_INT2_FIFO_TH_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_FIFO_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_FIFO_OVR_DISABLED 		 = 0x00,
	INT2_FIFO_OVR_ENABLED 		 = 0x10,
} LSM6DSO_INT2_FIFO_OVR_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_FIFO_FULL
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_FIFO_FULL_DISABLED 		 = 0x00,
	INT2_FIFO_FULL_ENABLED 		 = 0x20,
} LSM6DSO_INT2_FIFO_FULL_t;

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0x0E
* Bit Group Name: INT2_CNT_BDR
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_CNT_BDR_DISABLED 		 = 0x00,
	INT2_CNT_BDR_ENABLE   		 = 0x40,
} LSM6DSO_INT2_CNT_BDR_t;

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0x0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	WHO_AM_I_BIT_MASK  	0xFF
#define  	WHO_AM_I_BIT_POSITION  	0


/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0x10
* Bits          : [1]
* Bit Group Name: LPF2_XL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LPF2_XL_DISABLE = 0x00,
	LPF2_XL_EN      = 0x02
} LSM6DSO_LPF2_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0x10
* Bits          : [3:2]
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	FS_XL_2g 		 = 0x00,
  FS_XL_16g  	 = 0x04,
	FS_XL_4g 		 = 0x08,
	FS_XL_8g 		 = 0x0C,
  FS_XL_MASK   = 0xF3
} LSM6DSO_FS_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0x10
* Bits          : [7:4]
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	ODR_XL_DISABLE   = 0x00, 
	ODR_XL_1_6Hz     = 0xB0, // Low Power only
	ODR_XL_12_5Hz    = 0x10, // Low Power only
	ODR_XL_26Hz      = 0x20, // Low Power only
	ODR_XL_52Hz      = 0x30, // Low Power only 
	ODR_XL_104Hz     = 0x40, // Normal Mode
	ODR_XL_208Hz     = 0x50, // Normal Mode
	ODR_XL_416Hz     = 0x60, // High performance
	ODR_XL_833Hz     = 0x70, // High Performance 
	ODR_XL_1660Hz    = 0x80, // High Performance
	ODR_XL_3330Hz    = 0x90, // High Performance
	ODR_XL_6660Hz    = 0xA0, // High Performance
  ODR_XL_MASK      = 0x0F
} LSM6DSO_ODR_XL_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0x11
* Bit           : [3:2]
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	FS_G_125dps   	= 0x02,
	FS_G_250dps 		= 0x00,
	FS_G_500dps 		= 0x04,
	FS_G_1000dps 		= 0x08,
	FS_G_2000dps 		= 0x0C,
  FS_G_MASK       = 0xF0
} LSM6DSO_FS_G_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0x11
* Bit           : [7:4]
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	ODR_GYRO_DISABLE = 0x00,
	ODR_GYRO_12_5Hz  = 0x10, // Low Power only
	ODR_GYRO_26Hz    = 0x20, // Low Power only
	ODR_GYRO_52Hz    = 0x30, // Low Power only
	ODR_GYRO_104Hz   = 0x40, // Normal Mode
	ODR_GYRO_208Hz   = 0x50, // Normal Mode
	ODR_GYRO_416Hz   = 0x60, // High performance
	ODR_GYRO_833Hz   = 0x70, // High Performance
	ODR_GYRO_1660Hz  = 0x80, // High Performance
	ODR_GYRO_3330Hz  = 0x90, // High Performance
	ODR_GYRO_6660Hz  = 0xA0, // High Performance
  ODR_GYRO_MASK    = 0x0F
} LSM6DSO_ODR_GYRO_G_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: SW_RESET
* Permission    : RW
*******************************************************************************/
typedef enum {
	SW_RESET_NORMAL_MODE 		 = 0x00,
	SW_RESET_DEVICE 		 = 0x01,
} LSM6DSO_SW_RESET_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: IF_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
	IF_INC_DISABLED 		 = 0x00,
	IF_INC_ENABLED 		 = 0x04,
} LSM6DSO_IF_INC_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
	SIM_4_WIRE 		 = 0x00,
	SIM_3_WIRE 		 = 0x08,
} LSM6DSO_SIM_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
	PP_OD_PUSH_PULL 		 = 0x00,
	PP_OD_OPEN_DRAIN 		 = 0x10,
} LSM6DSO_PP_OD_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT_ACTIVE_HIGH   = 0x00,
	INT_ACTIVE_LOW  = 0x20,
} LSM6DSO_H_LACTIVE_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
	BDU_CONTINUOS 	 = 0x00,
	BDU_BLOCK_UPDATE = 0x40,
  BDU_MASK         = 0xBF 
} LSM6DSO_BDU_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0x12
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	BOOT_NORMAL_MODE 		 = 0x00,
	BOOT_REBOOT_MODE 		 = 0x80,
} LSM6DSO_BOOT_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: STOP_ON_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
	STOP_ON_FTH_DISABLED 		 = 0x00,
	STOP_ON_FTH_ENABLED 		 = 0x01,
} LSM6DSO_STOP_ON_FTH_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: MODE3_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	MODE3_EN_DISABLED 		 = 0x00,
	MODE3_EN_ENABLED 		 = 0x02,
} LSM6DSO_MODE3_EN_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
	I2C_DISABLE_I2C_AND_SPI 		 = 0x00,
	I2C_DISABLE_SPI_ONLY 		 = 0x04,
} LSM6DSO_I2C_DISABLE_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: DRDY_MSK
* Permission    : RW
*******************************************************************************/
typedef enum {
	DRDY_MSK_DISABLED 		 = 0x00,
	DRDY_MSK_ENABLED 		 = 0x08,
} LSM6DSO_DRDY_MSK_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: FIFO_TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	FIFO_TEMP_EN_DISABLED 		 = 0x00,
	FIFO_TEMP_EN_ENABLED 		 = 0x10,
} LSM6DSO_FIFO_TEMP_EN_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: INT2_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_ON_INT1_DISABLED 		 = 0x00,
	INT2_ON_INT1_ENABLED 		 = 0x20,
} LSM6DSO_INT2_ON_INT1_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: SLEEP_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	SLEEP_G_DISABLED 		 = 0x00,
	SLEEP_G_ENABLED 		 = 0x40,
} LSM6DSO_SLEEP_G_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0x13
* Bit Group Name: BW_SCAL_ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
	BW_SCAL_ODR_DISABLED 		 = 0x00,
	BW_SCAL_ODR_ENABLED 		 = 0x80,
} LSM6DSO_BW_SCAL_ODR_t;

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0x14
* Bit Group Name: ST_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	ST_XL_NORMAL_MODE 		 = 0x00,
	ST_XL_POS_SIGN_TEST 		 = 0x01,
	ST_XL_NEG_SIGN_TEST 		 = 0x02,
	ST_XL_NA 		 = 0x03,
} LSM6DSO_ST_XL_t;

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0x14
* Bit Group Name: ST_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	ST_G_NORMAL_MODE 		 = 0x00,
	ST_G_POS_SIGN_TEST 		 = 0x04,
	ST_G_NA 		 = 0x08,
	ST_G_NEG_SIGN_TEST 		 = 0x0C,
} LSM6DSO_ST_G_t;

/*******************************************************************************
* Register      : CTRL6_C
* Address       : 0x15
* Bits          : [2:0]
* Bit Group Name: FTYPE
* Permission    : RW
*******************************************************************************/
typedef enum {
	FTYPE_ONE   = 0x00,
	FTYPE_TWO   = 0x01,
	FTYPE_THREE = 0x02,
	FTYPE_FOUR  = 0x03,
	FTYPE_FIVE  = 0x04,
	FTYPE_SIX   = 0x05,
	FTYPE_SEVEN = 0x06,
	FTYPE_EIGHT = 0x07,
	FTYPE_NINE  = 0x08,
	FTYPE_TEN   = 0x09
} LSM6DSO_FTYPE_t;

/*******************************************************************************
* Register      : CTRL6_C
* Address       : 0x15
* Bit Group Name: USR_OFF_W
* Permission    : RW
*******************************************************************************/
typedef enum {
	USER_OFFSET_2_10 = 0x00,
	USER_OFFSET_2_6  = 0x08,
} LSM6DSO_USER_OFFSET_t;

/*******************************************************************************
* Register      : CTRL6_C
* Address       : 0x15
* Bit Group Name: XL_HM_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	HIGH_PERF_ACC_ENABLE 		 = 0x00, //Default
	HIGH_PERF_ACC_DISABLE 		 = 0x10,
} LSM6DSO_HIGH_PERF_ACC_t;

/*******************************************************************************
* Register      : CTRL6_C
* Address       : 0x15
* Bits          : [7:5]
* Bit Group Names: TRIG_EN, LVL1_EN, LVL2_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	TRIG_MODE_EDGE    = 0x80,
	TRIG_MODE_TRIGGER = 0x40,
	TRIG_MODE_LATCH   = 0x30,
	TRIG_MODE_FIFO    = 0x60,
} LSM6DSO_TRIG_MODE_t;

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0x16
* Bit Group Name: G_HM_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	HIGH_PERF_GYRO_ENABLE 		 = 0x00,
	HIGH_PERF_GYRO_DISABLE 		 = 0x80
} LSM6DSO_HIGH_PERF_GYRO_t;

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0x16
* Bit Group Name: HP_EN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	HP_EN_DISABLED 	 = 0x00,
	HP_EN_ENABLED 		 = 0x40,
} LSM6DSO_HP_EN_t;

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0x16
* Bit Group Name: HPM_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	HPM_G_16mHz  = 0x00,
	HPM_G_65mHz  = 0x20,
	HPM_G_260mHz = 0x30,
	HPM_G_1_04Hz = 0x40,
} LSM6DSO_HPM_G_t;

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0x17
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum {
	FDS_FILTER_OFF 		 = 0x00,
	FDS_FILTER_ON 		 = 0x04,
} LSM6DSO_FDS_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0x18
* Bit Group Name: XEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	XEN_XL_DISABLED 		 = 0x00,
	XEN_XL_ENABLED 		 = 0x08,
} LSM6DSO_XEN_XL_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0x18
* Bit Group Name: YEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	YEN_XL_DISABLED 		 = 0x00,
	YEN_XL_ENABLED 		 = 0x10,
} LSM6DSO_YEN_XL_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0x18
* Bit Group Name: ZEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	ZEN_XL_DISABLED 		 = 0x00,
	ZEN_XL_ENABLED 		 = 0x20,
} LSM6DSO_ZEN_XL_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: XEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	XEN_G_DISABLED 		 = 0x00,
	XEN_G_ENABLED 		 = 0x08,
} LSM6DSO_XEN_G_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: YEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	YEN_G_DISABLED 		 = 0x00,
	YEN_G_ENABLED 		 = 0x10,
} LSM6DSO_YEN_G_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: ZEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	ZEN_G_DISABLED 		 = 0x00,
	ZEN_G_ENABLED 		 = 0x20,
} LSM6DSO_ZEN_G_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0x19
* Bit Group Name: FUNC_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	FUNC_EN_DISABLED 		 = 0x00,
	FUNC_EN_ENABLED 		 = 0x04,
} LSM6DSO_FUNC_EN_t;

/*******************************************************************************
* Register      : ALL_INT_SRC
* Address       : 0x1A
* Permission    : R
*******************************************************************************/
typedef enum {
  INT_FREE_FALL          = 0x01,
  INT_WAKE_UP            = 0x02,
  INT_SINGLE_TAP         = 0x04,
  INT_DOUBLE_TAP         = 0x08,
  INT_D6D                = 0x10,
  INT_SLEEP_CHANGEN      = 0x20,
  INT_TIMESTAMP_ENDCOUNT = 0x80,
} LSM6DSO_ALL_INT_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: MASTER_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
	MASTER_ON_DISABLED 		 = 0x00,
	MASTER_ON_ENABLED 		 = 0x01,
} LSM6DSO_MASTER_ON_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: IRON_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	IRON_EN_DISABLED 		 = 0x00,
	IRON_EN_ENABLED 		 = 0x02,
} LSM6DSO_IRON_EN_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: PASS_THRU_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	PASS_THRU_MODE_DISABLED 		 = 0x00,
	PASS_THRU_MODE_ENABLED 		 = 0x04,
} LSM6DSO_PASS_THRU_MODE_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: PULL_UP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	PULL_UP_EN_DISABLED 		 = 0x00,
	PULL_UP_EN_ENABLED 		 = 0x08,
} LSM6DSO_PULL_UP_EN_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: START_CONFIG
* Permission    : RW
*******************************************************************************/
typedef enum {
	START_CONFIG_XL_G_DRDY 		 = 0x00,
	START_CONFIG_EXT_INT2 		 = 0x10,
} LSM6DSO_START_CONFIG_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: DATA_VAL_SEL_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
	DATA_VAL_SEL_FIFO_XL_G_DRDY 		 = 0x00,
	DATA_VAL_SEL_FIFO_SHUB_DRDY 		 = 0x40,
} LSM6DSO_DATA_VAL_SEL_FIFO_t;

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0x1A
* Bit Group Name: DRDY_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
	DRDY_ON_INT1_DISABLED 		 = 0x00,
	DRDY_ON_INT1_ENABLED 		 = 0x80,
} LSM6DSO_DRDY_ON_INT1_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: Z_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
	Z_WU_NOT_DETECTED 		 = 0x00,
	Z_WU_DETECTED 		 = 0x01,
} LSM6DSO_Z_WU_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: Y_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
	Y_WU_NOT_DETECTED 		 = 0x00,
	Y_WU_DETECTED 		 = 0x02,
} LSM6DSO_Y_WU_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: X_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
	X_WU_NOT_DETECTED 		 = 0x00,
	X_WU_DETECTED 		 = 0x04,
} LSM6DSO_X_WU_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: WU_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	WU_EV_STATUS_NOT_DETECTED 		 = 0x00,
	WU_EV_STATUS_DETECTED 		 = 0x08,
} LSM6DSO_WU_EV_STATUS_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: SLEEP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	SLEEP_EV_STATUS_NOT_DETECTED 		 = 0x00,
	SLEEP_EV_STATUS_DETECTED 		 = 0x10,
} LSM6DSO_SLEEP_EV_STATUS_t;

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0x1B
* Bit Group Name: FF_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	FF_EV_STATUS_NOT_DETECTED 		 = 0x00,
	FF_EV_STATUS_DETECTED 		 = 0x20,
} LSM6DSO_FF_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: Z_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	Z_TAP_NOT_DETECTED 		 = 0x00,
	Z_TAP_DETECTED 		 = 0x01,
} LSM6DSO_Z_TAP_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: Y_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	Y_TAP_NOT_DETECTED 		 = 0x00,
	Y_TAP_DETECTED 		 = 0x02,
} LSM6DSO_Y_TAP_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: X_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	X_TAP_NOT_DETECTED 		 = 0x00,
	X_TAP_DETECTED 		 = 0x04,
} LSM6DSO_X_TAP_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: TAP_SIGN
* Permission    : RO
*******************************************************************************/
typedef enum {
	TAP_SIGN_POS_SIGN 		 = 0x00,
	TAP_SIGN_NEG_SIGN 		 = 0x08,
} LSM6DSO_TAP_SIGN_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: DOUBLE_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	DOUBLE_TAP_NOT_DETECTED 		 = 0x00,
	DOUBLE_TAP_DETECTED 		 = 0x10,
} LSM6DSO_DOUBLE_TAP_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: SINGLE_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
	SINGLE_TAP_NOT_DETECTED 		 = 0x00,
	SINGLE_TAP_DETECTED 		 = 0x20,
} LSM6DSO_SINGLE_TAP_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0x1C
* Bit Group Name: TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	TAP_EV_NOT_DETECTED 		 = 0x00,
	TAP_EV_DETECTED 		 = 0x40,
} LSM6DSO_TAP_EV_STATUS_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_XL
* Permission    : RO
*******************************************************************************/
typedef enum {
	DSD_XL_NOT_DETECTED 		 = 0x00,
	DSD_XL_DETECTED 		 = 0x01,
} LSM6DSO_DSD_XL_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_XH
* Permission    : RO
*******************************************************************************/
typedef enum {
	DSD_XH_NOT_DETECTED 		 = 0x00,
	DSD_XH_DETECTED 		 = 0x02,
} LSM6DSO_DSD_XH_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_YL
* Permission    : RO
*******************************************************************************/
typedef enum {
	DSD_YL_NOT_DETECTED 		 = 0x00,
	DSD_YL_DETECTED 		 = 0x04,
} LSM6DSO_DSD_YL_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_YH
* Permission    : RO
*******************************************************************************/
typedef enum {
	DSD_YH_NOT_DETECTED 		 = 0x00,
	DSD_YH_DETECTED 		 = 0x08,
} LSM6DSO_DSD_YH_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
	DSD_ZL_NOT_DETECTED 		 = 0x00,
	DSD_ZL_DETECTED 		 = 0x10,
} LSM6DSO_DSD_ZL_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: DSD_ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
	DSD_ZH_NOT_DETECTED 		 = 0x00,
	DSD_ZH_DETECTED 		 = 0x20,
} LSM6DSO_DSD_ZH_t;

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0x1D
* Bit Group Name: D6D_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	D6D_EV_STATUS_NOT_DETECTED 		 = 0x00,
	D6D_EV_STATUS_DETECTED 		 = 0x40,
} LSM6DSO_D6D_EV_STATUS_t;

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0x1E
* Bit Group Name: XLDA
* Permission    : RO
*******************************************************************************/
typedef enum {
	XLDA_NO_DATA_AVAIL 		 = 0x00,
	XLDA_DATA_AVAIL 		 = 0x01,
} LSM6DSO_XLDA_t;

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0x1E
* Bit Group Name: GDA
* Permission    : RO
*******************************************************************************/
typedef enum {
	GDA_NO_DATA_AVAIL 		 = 0x00,
	GDA_DATA_AVAIL 		 = 0x02,
} LSM6DSO_GDA_t;

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0x1E
* Bit Group Name: EV_BOOT
* Permission    : RO
*******************************************************************************/
typedef enum {
	EV_BOOT_NO_BOOT_RUNNING 		 = 0x00,
	EV_BOOT_BOOT_IS_RUNNING 		 = 0x08,
} LSM6DSO_EV_BOOT_t;

/*******************************************************************************
* Register      : FIFO_STATUS1
* Address       : 0x3A
* Bit Group Name: DIFF_FIFO
* Permission    : RO
*******************************************************************************/
#define  	DIFF_FIFO_STATUS1_MASK  	0xFF
#define  	DIFF_FIFO_STATUS1_POSITION  	0
#define  	DIFF_FIFO_STATUS2_MASK  0xF
#define  	DIFF_FIFO_STATUS2_POSITION  	0

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: FIFO_EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
	FIFO_EMPTY_FIFO_NOT_EMPTY 		 = 0x00,
	FIFO_EMPTY_FIFO_EMPTY 		 = 0x10,
} LSM6DSO_FIFO_EMPTY_t;

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: FIFO_FULL
* Permission    : RO
*******************************************************************************/
typedef enum {
	FIFO_FULL_FIFO_NOT_FULL 		 = 0x00,
	FIFO_FULL_FIFO_FULL 		 = 0x20,
} LSM6DSO_FIFO_FULL_t;

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: OVERRUN
* Permission    : RO
*******************************************************************************/
typedef enum {
	OVERRUN_NO_OVERRUN 		 = 0x00,
	OVERRUN_OVERRUN 		 = 0x40,
} LSM6DSO_OVERRUN_t;

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0x3B
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
	WTM_BELOW_WTM 		 = 0x00,
	WTM_ABOVE_OR_EQUAL_WTM 		 = 0x80,
} LSM6DSO_WTM_t;


/*******************************************************************************
* Register      : FIFO_DATA_OUT_TAG
* Address       : 0x78
* Bit Group Name: TAG_SENSOR
* Permission    : R
*******************************************************************************/
typedef enum {
	TAG_GYRO_NC      = 0x01,
	TAG_ACCEL_NC     = 0x02,
	TAG_TEMPERATURE  = 0x03,
	TAG_TIME_STAMP   = 0x04,
	TAG_CFG_CHANGE   = 0x05,
	TAG_ACCEL_NC_T_2 = 0x06,
	TAG_ACCEL_NC_T_1 = 0x07,
	TAG_ACCEL_2xC    = 0x08,
	TAG_ACCEL_3xC    = 0x09,
	TAG_GYRO_NC_T_2  = 0x0A,
	TAG_GYRO_NC_T_1  = 0x0B,
	TAG_GYRO_2xC     = 0x0C,
	TAG_GYRO_3xC     = 0x0D,
	TAG_SENSOR_HUB_0 = 0x0E,
	TAG_SENSOR_HUB_1 = 0x0F,
	TAG_SENSOR_HUB_2 = 0x10,
	TAG_SENSOR_HUB_3 = 0x11,
	STEP_COUNTER     = 0x12,
	SENSOR_HUB_NACK  = 0x19,
} LSM6DSO_FIFO_TAG_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: SENS_HUB_END
* Permission    : RO
*******************************************************************************/
typedef enum {
	SENS_HUB_END_STILL_ONGOING 		 = 0x00,
	SENS_HUB_END_OP_COMPLETED 		 = 0x01,
} LSM6DSO_SENS_HUB_END_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: SOFT_IRON_END
* Permission    : RO
*******************************************************************************/
typedef enum {
	SOFT_IRON_END_NOT_COMPLETED 		 = 0x00,
	SOFT_IRON_END_COMPLETED 		 = 0x02,
} LSM6DSO_SOFT_IRON_END_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: PEDO_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	PEDO_EV_STATUS_NOT_DETECTED 		 = 0x00,
	PEDO_EV_STATUS_DETECTED 		 = 0x10,
} LSM6DSO_PEDO_EV_STATUS_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: TILT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	TILT_EV_STATUS_NOT_DETECTED 		 = 0x00,
	TILT_EV_STATUS_DETECTED 		 = 0x20,
} LSM6DSO_TILT_EV_STATUS_t;

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0x53
* Bit Group Name: SIGN_MOT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
	SIGN_MOT_EV_STATUS_NOT_DETECTED 		 = 0x00,
	SIGN_MOT_EV_STATUS_DETECTED 		 = 0x40,
} LSM6DSO_SIGN_MOT_EV_STATUS_t;

/*******************************************************************************
* Register      : TAP_CFG0
* Address       : 0x56
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIR_DISABLED 	 = 0x00,
	LIR_ENABLED 	 = 0x01,
	LIR_MASK 		   = 0xFE
} LSM6DSO_LIR_t;

#define TAP_INTERRUPT_MASK 0xF1 
/*******************************************************************************
* Register      : TAP_CFG0
* Address       : 0x56
* Bit Group Name: TAP_Z_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	TAP_Z_EN_DISABLED 	 = 0x00,
	TAP_Z_EN_ENABLED 		 = 0x02,
} LSM6DSO_TAP_Z_EN_t;

/*******************************************************************************
* Register      : TAP_CFG0
* Address       : 0x56
* Bit Group Name: TAP_Y_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	TAP_Y_EN_DISABLED    = 0x00,
	TAP_Y_EN_ENABLED 		 = 0x04,
} LSM6DSO_TAP_Y_EN_t;

/*******************************************************************************
* Register      : TAP_CFG0
* Address       : 0x56
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	TAP_X_EN_DISABLED		 = 0x00,
	TAP_X_EN_ENABLED 		 = 0x08,
} LSM6DSO_TAP_X_EN_t;

/*******************************************************************************
* Register      : TAP_CFG0
* Address       : 0x56
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT_CLR_ON_READ_AT_ODR 		 = 0x00,
	INT_CLR_ON_READ_IMMEDIATE  = 0x80
} LSM6DSO_INT_CLEAR_ON_READ_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0x58
* Bit Group Name: TAP_PRIORITY
* Permission    : RW
*******************************************************************************/
typedef enum {
	TAP_PRIORITY_XYZ 		 = 0x00,
	TAP_PRIORITY_YXZ 		 = 0x01,
	TAP_PRIORITY_XZY 		 = 0x02,
	TAP_PRIORITY_ZYX 		 = 0x03,
	//TAP_PRIORITY_XYZ 	 = 0x04, repeated
	TAP_PRIORITY_YZX 		 = 0x05,
	TAP_PRIORITY_ZXY   	 = 0x06,
	//TAP_PRIORITY_ZYX   = 0x07, repeated
  TAP_PRIORITY_MASK = 0x1F
} LSM6DSO_TAP_PRIORITY_t;


/*******************************************************************************
* Register      : TAP_CFG2
* Address       : 0x58
* Bit Group Name: INTERRUPTS_ENABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  INTERRUPTS_DISABLED = 0x00,
  INTERRUPTS_ENABLED = 0x80,
  INTERRUPTS_MASK = 0x7F,
} LSM6DSO_INTERRUPTS_t;

/*******************************************************************************
* Register      : EMB_FUNC_EN_A
* Address       : 0x04
* Bit Group Name: TILT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	TILT_DISABLED 	 = 0x00,
	TILT_ENABLED 		 = 0x10,
} LSM6DSO_TILT_EN_t;

/*******************************************************************************
* Register      : EMB_FUNC_EN_A
* Address       : 0x04
* Bit Group Name: PEDO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	PEDO_DISABLED 	 = 0x00,
	PEDO_ENABLED 		 = 0x08,
	PEDO_MASK 		 = 0xF7,
} LSM6DSO_PEDO_EN_t;

/*******************************************************************************
* Register      : EMB_FUNC_EN_A
* Address       : 0x04
* Bit Group Name: SIGN_MOTION_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	SIGN_MOTION_DISABLED 	 = 0x00,
	SIGN_MOTION_ENABLED 	 = 0x20,
} LSM6DSO_SIGN_MOTION_EN_t;


/*******************************************************************************
* Register      : EMB_FUNC_SRC
* Address       : 0x64
* Bit Group Name: PEDO_RST_STEP
* Permission    : RW
*******************************************************************************/
typedef enum {
	PEDO_RST_STEP_ENABLED 	 = 0x80,
	PEDO_RST_STEP_DISABLED 	 = 0x00,
  PEDO_RST_STEP_MASK 	     = 0x7F,
} PEDO_RST_STEP_t;

/*******************************************************************************
* Register      : EMB_FUNC_SRC
* Address       : 0x64
* Bit Group Name: STEP_DETECTED
* Permission    : RW
*******************************************************************************/
typedef enum {
	STEP_NOT_DETECED  = 0x00,
	STEP_DETECED 	    = 0x40,
	STEP_DETECED_MASK = 0xBF
} PEDO_STEP_DETECT_t;

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0x59
* Bit Group Name: TAP_THS
* Permission    : RW
*******************************************************************************/
#define  	TAP_THS_MASK  	0x1F
#define  	TAP_THS_POSITION  	0

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0x59
* Bit Group Name: SIXD_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
	SIXD_THS_80_degree 		 = 0x00,
	SIXD_THS_70_degree 		 = 0x20,
	SIXD_THS_60_degree 		 = 0x40,
	SIXD_THS_50_degree 		 = 0x60,
} LSM6DSO_SIXD_THS_t;

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0x5A
* Bit Group Name: SHOCK
* Permission    : RW
*******************************************************************************/
#define  	SHOCK_MASK  	0x03
#define  	SHOCK_POSITION  	0

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0x5A
* Bit Group Name: QUIET
* Permission    : RW
*******************************************************************************/
#define  	QUIET_MASK  	0x0C
#define  	QUIET_POSITION  	2

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0x5A
* Bit Group Name: DUR
* Permission    : RW
*******************************************************************************/
#define  	DUR_MASK  	0xF0
#define  	DUR_POSITION  	4

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x5B
* Bit Group Name: WK_THS
* Permission    : RW
*******************************************************************************/
#define  	WK_THS_MASK  	0x3F
#define  	WK_THS_POSITION  	0

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x5B
* Bit Group Name: INACTIVITY_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
	INACTIVITY_ON_DISABLED 		 = 0x00,
	INACTIVITY_ON_ENABLED 		 = 0x40,
} LSM6DSO_INACTIVITY_ON_t;

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x5B
* Bit Group Name: SINGLE_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	SINGLE_DOUBLE_TAP_DOUBLE_TAP 		 = 0x00,
	SINGLE_DOUBLE_TAP_SINGLE_TAP 		 = 0x80,
} LSM6DSO_SINGLE_DOUBLE_TAP_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x5C
* Bit Group Name: SLEEP_DUR
* Permission    : RW
*******************************************************************************/
#define  	SLEEP_DUR_MASK  	0x0F
#define  	SLEEP_DUR_POSITION  	0

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x5C
* Bit Group Name: TIMER_HR
* Permission    : RW
*******************************************************************************/
typedef enum {
	TIMER_HR_6_4ms 		 = 0x00,
	TIMER_HR_25us 		 = 0x10,
} LSM6DSO_TIMER_HR_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x5C
* Bit Group Name: WAKE_DUR
* Permission    : RW
*******************************************************************************/
#define  	WAKE_DUR_MASK  	0x60
#define  	WAKE_DUR_POSITION  	5

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0x5D
* Bit Group Name: FF_DUR
* Permission    : RW
*******************************************************************************/
#define  	FF_FREE_FALL_DUR_MASK  	0xF8
#define  	FF_FREE_FALL_DUR_POSITION  	3
#define  	FF_WAKE_UP_DUR_MASK  	0x80
#define  	FF_WAKE_UP_DUR_POSITION  	7


/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0x5D
* Bit Group Name: FF_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
	FF_THS_5 		 = 0x00,
	FF_THS_7 		 = 0x01,
	FF_THS_8 		 = 0x02,
	FF_THS_10 		 = 0x03,
	FF_THS_11 		 = 0x04,
	FF_THS_13 		 = 0x05,
	FF_THS_15 		 = 0x06,
	FF_THS_16 		 = 0x07,
} LSM6DSO_FF_THS_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_SHUB
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_SHUB_DISABLED 		 = 0x00,
	INT1_SHUB_ENABLED 		 = 0x01,
} LSM6DSO_INT1_SHUB_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_EMB_FUNC
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_EMB_FUNC_DISABLED 		 = 0x00,
	INT1_EMB_FUNC_ENABLED 		 = 0x02,
} LSM6DSO_INT1_TILT_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_6D_DISABLED 		 = 0x00,
	INT1_6D_ENABLED 		 = 0x04,
} LSM6DSO_INT1_6D_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_DOUBLE_TAP_DISABLED 		 = 0x00,
	INT1_DOUBLE_TAP_ENABLED 		 = 0x08,
} LSM6DSO_INT1_TAP_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_FF_DISABLED 		 = 0x00,
	INT1_FF_ENABLED 		 = 0x10,
} LSM6DSO_INT1_FF_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_WU_DISABLED 		 = 0x00,
	INT1_WU_ENABLED 		 = 0x20,
} LSM6DSO_INT1_WU_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_SINGLE_TAP_DISABLED 		 = 0x00,
	INT1_SINGLE_TAP_ENABLED 		 = 0x40,
  INT1_SINGLE_TAP_MASK = 0xBF
} LSM6DSO_INT1_SINGLE_TAP_t;

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0x5E
* Bit Group Name: INT1_SLEEP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT1_SLEEP_DISABLED 		 = 0x00,
	INT1_SLEEP_ENABLED 		 = 0x80,
} LSM6DSO_INT1_SLEEP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_TIMESTAMP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_TIMESTAMP_DISABLED 		 = 0x00,
	INT2_TIMESTAMP_ENABLED 		 = 0x01,
} LSM6DSO_INT2_TIMESTAMP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_EMB_FUNC
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_EMB_FUNC_DISABLED 		 = 0x00,
	INT2_EMB_FUNC_ENABLED 		 = 0x02,
} LSM6DSO_INT2_EMB_FUNC_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_6D_DISABLED 		 = 0x00,
	INT2_6D_ENABLED 		 = 0x04,
} LSM6DSO_INT2_6D_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_DOUBLE_TAP_DISABLED 		 = 0x00,
	INT2_DOUBLE_TAP_ENABLED 		 = 0x08,
} LSM6DSO_INT2_DOUBLE_TAP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_FF_DISABLED 		 = 0x00,
	INT2_FF_ENABLED 		 = 0x10,
} LSM6DSO_INT2_FF_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_WU_DISABLED 		 = 0x00,
	INT2_WU_ENABLED 		 = 0x20,
} LSM6DSO_INT2_WU_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_SINGLE_TAP_DISABLED 		 = 0x00,
	INT2_SINGLE_TAP_ENABLED 		 = 0x40,
} LSM6DSO_INT2_SINGLE_TAP_t;

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0x5F
* Bit Group Name: INT2_SLEEP
* Permission    : RW
*******************************************************************************/
typedef enum {
	INT2_SLEEP_DISABLED 		 = 0x00,
	INT2_SLEEP_ENABLED 		 = 0x80,
} LSM6DSO_INT2_SLEEP_t;

#endif  // End of __LSM6DSOIMU_H__ definition check
