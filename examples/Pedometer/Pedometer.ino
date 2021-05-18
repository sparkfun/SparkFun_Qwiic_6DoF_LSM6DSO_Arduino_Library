/******************************************************************************
Basic_Readings.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This examples enable step detection. 

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DSO myIMU(I2C_MODE, 0x6B);
//LSM6DSO myIMU(SPI_MODE, 10);

void setup() {

  Serial.begin(115200);
  delay(500); 
  Serial.println("Ready.");
  
  myIMU.begin(PEDOMETER_SETTINGS); // Loads settings for pedometer.
  
}


void loop()
{
  if ( myIMU.listenStep() ) {
    Serial.println("Step taken!");
    Serial.print("Total Steps: ");
    Serial.println(myIMU.getSteps());
  }
  
  // Want to reset the counter? Uncomment: 
  // myIMU.resetSteps();
  delay(50);
}
