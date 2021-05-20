/******************************************************************************
Fifo_Example.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This example enables the use of the FIFO for data collection. The FIFO is set
to collect 500 bytes of temperature and gyroscopic data before it stops
collecting data (FIFO_MODE). Upon reading the data (or if you continuously read the FIFO)
the FIFO will begine to refill. 

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

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DSO myIMU;
fifoData myFifo; //This will hold our FIFO data
int availableBytes = 0;

void setup()
{

  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(FIFO_SETTINGS) )
    Serial.println("Loaded Settings.");

	
  
}


void loop()
{
  availableBytes = myIMU.getFifoStatus();  //Check for data in FIFO

  if( availableBytes > 0 ){
    Serial.print("Number of bytes in FIFO: ");
    Serial.println(availableBytes);

    myFifo = myIMU.fifoRead(); // Get the data

    if( myFifo.fifoTag == ACCELEROMETER_DATA ){
      Serial.println("Accelerometer:");
      Serial.print(" X = ");
      Serial.println(myFifo.xAccel, 3);
      Serial.print(" Y = ");
      Serial.println(myFifo.yAccel, 3);
      Serial.print(" Z = ");
      Serial.println(myFifo.zAccel, 3);
    }

    if( myFifo.fifoTag == GYROSCOPE_DATA ){
      Serial.println("Gyroscope: ");
      Serial.print(" X = ");
      Serial.println(myFifo.xGyro, 3);
      Serial.print(" Y = ");
      Serial.println(myFifo.yGyro, 3);
      Serial.print(" Z = ");
      Serial.println(myFifo.zGyro, 3);
    }
  }

  delay(500);
 
}
