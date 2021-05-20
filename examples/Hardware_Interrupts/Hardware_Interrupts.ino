/******************************************************************************
Hardware_Interrupts.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This examples gets acclerometer data when an interrupt is deteced from "INT1"
and gyroscopic data when an interrupt is detected from "INT2". These two
interrupts can be put anywhere on your dev board, but for this example pins
two and three were used for "INT1" and INT2" respectively.

Interrupts can be changed to be active low: 
myIMU.configHardOutInt(INT_ACTIVE_LOW);
Interrupts are being configured automatically like so: 
myIMU.setInterruptOne(INT1_DRDY_XL_ENABLED);
myIMU.setInterruptTwo(INT2_DRDY_G_ENABLED);

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

int accelInt = 2; 
int gyroInt = 3; 

void setup()
{

  Serial.begin(115200);
  delay(500); 

  pinMode(accelInt, INPUT);
  pinMode(gyroInt,INPUT);

  Wire.begin();
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
    while(1);
  }

  if( myIMU.initialize(HARD_INT_SETTINGS) )
    Serial.println("Settings Loaded.");
	
}


void loop()
{

  if( digitalRead(accelInt) == HIGH ){
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatAccelX(), 3);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatAccelY(), 3);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatAccelZ(), 3);
  }

  if( digitalRead(gyroInt) == HIGH ){
    Serial.print("\nGyroscope:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatGyroX(), 3);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatGyroY(), 3);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatGyroZ(), 3);
  }

  delay(1000);
}


