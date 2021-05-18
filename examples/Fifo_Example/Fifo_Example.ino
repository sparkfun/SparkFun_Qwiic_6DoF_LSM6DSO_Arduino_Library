/******************************************************************************
Tap_Interrupts.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This example enables the use of the FIFO for data collection. The FIFO is set
to collect 500 bytes of temperature and gyroscopic data before it stops
collecting data (FIFO_MODE). Upon reading the data (or if you continuously read the FIFO)
the FIFO will being to refill. 

There are other available modes (see datasheet found in github repository for
more information) but the entire list of fifo functionality predefines can be found below. 

myIMU.setFifoMode( -arguments below- )

FIFO_MODE_DISABLED       
FIFO_MODE_STOP_WHEN_FULL 
FIFO_MODE_CONT_TO_FIFO   
FIFO_MODE_BYPASS_TO_CONT 
FIFO_MODE_CONTINUOUS     
FIFO_MODE_BYPASS_TO_FIFO 
FIFO_MODE_MASK           

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfugn.com.
Distributed as-is; no warranty is given.
******************************************************************************/
Please review the LICENSE.md file included with this example. If you have any questions #include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU( SPI_MODE, 10 );

void setup( void ) {
  SPI.begin();
  //Over-ride default settings if desired
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 833;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroAccelDecimation = 1;  //Decimation can be set to: 1, 2, or 3. These settings correspond to 1, 8, or 32 respectively.

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 833;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  myIMU.settings.tempEnabled = 1;
  
    //Non-basic mode settings
  myIMU.settings.commMode = 1;

  //FIFO control settings
  myIMU.settings.fifoThreshold = 100;  //Can be 0 to 256 (8 bits)
  //myIMU.settings.fifoSampleRate = 50;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
  myIMU.settings.fifoModeWord = 6;  //FIFO mode.
  //FIFO mode.  Can be:
  //  0 (Bypass mode, FIFO off)
  //  1 (Stop when full)
  //  3 (Continuous during trigger)
  //  4 (Bypass until trigger)
  //  6 (Continous mode)
  

  // Initialize the struct that holds our x and y data
  fifoData = mydata; 

  Serial.begin(57600);  // start serial for output
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMUs
  if( myIMU.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor with CS @ Pin 10.");
  }
  else
  {
	  Serial.println("Sensor with CS @ Pin 10 started.");
  }
  
  Serial.print("Configuring FIFO with no error checking...");
  myIMU.fifoBegin();
  Serial.print("Done!\n");
  
  Serial.print("Clearing out the FIFO...");
  myIMU.fifoClear();
  Serial.print("Done!\n");
  
}


void loop()
{
  uint16_t tempUnsigned;
  
  while( ( myIMU.fifoGetStatus() & 0x8000 ) == 0 ) {};  //Wait for watermark
 
  //Now loop until FIFO is empty.  NOTE:  As the FIFO is only 8 bits wide,
  //the channels must be synchronized to a known position for the data to align
  //properly.  Emptying the fifo is one way of doing this (this example)
  while( ( myIMU.fifoGetStatus() & 0x1000 ) == 0 ) {

  myData = myIMU.fifoRead();
  if( myData.fifoTag = GYROSCOPE_DATA ){
    myData.xData = myIMU.calcGyro(myData.xData);
    myData.yData = myIMU.calcGyro(myData.yData);
    myData.zData = myIMU.calcGyro(myData.zData);
    Serial.print("Gyro: ");
  }

  else if (myData.fifoTag = ACCELEROMETER_DATA){
    myData.xData = myIMU.calcAccel(myData.xData);
    myData.yData = myIMU.calcAccel(myData.yData);
    myData.zData = myIMU.calcAccel(myData.zData);
    Serial.print("Accel: ");
  }
  

  Serial.print(myData.xData);
  Serial.print(",");


  Serial.print(myData.yData);
  Serial.print(",");

  Serial.print(myData.zData);
  Serial.println();

  delay(10); //Wait for the serial buffer to clear (~50 bytes worth of time @ 57600baud)
  
  }

  tempUnsigned = myIMU.fifoGetStatus();
  Serial.print("\nFifo Status 1 and 2 (16 bits): 0x");
  Serial.println(tempUnsigned, HEX);
  Serial.print("\n");  

}
