/******************************************************************************
Software_Interrupts.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This examples polls the "data ready" bit before checking for new acceleromter,
gyroscopic, and temperature data. The data is then printed out to the serial
monitor. 

For the sake of demonstration all the data points by have been turned on by default and
so you can expect the IMU to send "all the data" when data is ready:
"myIMU.listenDataReady()". 

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
// #include "SPI.h"

LSM6DSO myIMU; 

int data; 

void setup() {

  Serial.begin(115200);
  delay(500); 

  Wire.begin();
  if( myIMU.begin() )// Load software interrupt related settings
    Serial.println("Ready.");
  else {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(SOFT_INT_SETTINGS)) 
    Serial.println("Loaded Settings.");

}


void loop()
{

  data = myIMU.listenDataReady();

  if( data == ALL_DATA_READY ){
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatAccelX(), 3);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatAccelY(), 3);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatAccelZ(), 3);
    Serial.print("\nGyroscope:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatGyroX(), 3);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatGyroY(), 3);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatGyroZ(), 3);
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees F = ");
    Serial.println(myIMU.readTempF(), 3);
  }

  delay(1000);
/*
  if( data == ACCEL_DATA_READY ){
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatAccelX(), 3);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatAccelY(), 3);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatAccelZ(), 3);
  }

  if( data == GYRO_DATA_READY ){
    Serial.print("\nGyroscope:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatGyroX(), 3);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatGyroY(), 3);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatGyroZ(), 3);
  }

  if( data == TEMP_DATA_READY ){
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees F = ");
    Serial.println(myIMU.readTempF(), 3);
  }
  */
}


