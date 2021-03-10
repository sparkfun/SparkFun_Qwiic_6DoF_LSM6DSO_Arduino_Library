/******************************************************************************
MultiSPI.ino

Original Library written for the LSM6DS3 by Marshall Taylor @ SparkFun Electronics
Updated to modern SparkFun practices for the LSM6DS0 by Elias Santistevan @ SparkFun Electronics
March, 2021
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DS0
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DS0_Arduino_Library

Description:
Example using up to two LSM6DS0's on the same SPI channel, with different CS pins.
If only one sensor is attached, this sketch reports failure on that channel and
runs with the single sensor instead.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

(Multiple SPI devices share pins except for the Chip Select lines which
are unique for each device on the bus.)

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS0.h"
#include "Wire.h"
#include "SPI.h"

//Create two instances of the driver class
LSM6DS0 SensorOne( SPI_MODE, 10 );
LSM6DS0 SensorTwo( SPI_MODE, 9 );


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  Wire.begin();
  //Call .begin() to configure the IMUs
  if( SensorOne.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor with CS @ Pin 10.");
  }
  else
  {
	  Serial.println("Sensor with CS @ Pin 10 started.");
  }
  if( SensorTwo.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor with CS @ Pin 9.");
  }
  else
  {
	  Serial.println("Sensor with CS @ Pin 9 started.");
  }
  
}


void loop()
{
  //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X1 = ");
  Serial.println(SensorOne.readFloatAccelX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(SensorOne.readFloatAccelY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(SensorOne.readFloatAccelZ(), 4);
  Serial.print(" X2 = ");
  Serial.println(SensorTwo.readFloatAccelX(), 4);
  Serial.print(" Y2 = ");
  Serial.println(SensorTwo.readFloatAccelY(), 4);
  Serial.print(" Z2 = ");
  Serial.println(SensorTwo.readFloatAccelZ(), 4);
  
  Serial.print("\nGyroscope:\n");
  Serial.print(" X1 = ");
  Serial.println(SensorOne.readFloatGyroX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(SensorOne.readFloatGyroY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(SensorOne.readFloatGyroZ(), 4);
  Serial.print(" X2 = ");
  Serial.println(SensorTwo.readFloatGyroX(), 4);
  Serial.print(" Y2 = ");
  Serial.println(SensorTwo.readFloatGyroY(), 4);
  Serial.print(" Z2 = ");
  Serial.println(SensorTwo.readFloatGyroZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(SensorTwo.readTempC(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(SensorTwo.readTempF(), 4);
  
  Serial.print("\nSensorOne Bus Errors Reported:\n");
  Serial.print(" All '1's = ");
  Serial.println(SensorOne.allOnesCounter);
  Serial.print(" Non-success = ");
  Serial.println(SensorOne.nonSuccessCounter);
  Serial.print("SensorTwo Bus Errors Reported:\n");
  Serial.print(" All '1's = ");
  Serial.println(SensorTwo.allOnesCounter);
  Serial.print(" Non-success = ");
  Serial.println(SensorTwo.nonSuccessCounter);
  delay(1000);
}
