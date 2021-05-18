/******************************************************************************
Tap_Interrupts.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This example enables tap detection in the x-axis direction. The tap is detected
from "INT1" on the product and is routed to hardware pin 2 on an SparkFun
Redboard. This can be configured to listen to taps in other directions, with different priorities
using the following functions: 

The arguements for the following function are as follows: 
enableTap( enable-tap, x-direction, y-direction, z-direction) 

e.g. for y-direction only
myIMU.enableTap(true, false, true, false);
e.g. for all directions 
myIMU.enableTap(true, true, true, true);

If you enable them all, you will then need to enable priority: 
Tap direction priority x, then y, then z.
myIMU.setTapDirPrior(TAP_PRIORITY_XYZ);
Tap direction priority y, then x, then z.
myIMU.setTapDirPrior(TAP_PRIORITY_YXZ);
etc.

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfugn.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#include "SparkFunLSM6DSO.h"
#include <Wire.h>
//#include <SPI.h>

LSM6DSO myIMU( I2C_MODE, 0x6B );
//LSM6DSO myIMU( SPI_MODE, 10 );

int tapInterrupt = 2; 

void setup() {

  Serial.begin(115200);
  delay(500); 
  Serial.println("Ready.");

  pinMode(tapInterrupt, INPUT_PULLUP);

  myIMU.begin(TAP_SETTINGS); // Load tap interrupt related settings
	
}

void loop()
{
  
  if( digitalRead( tapInterrupt ) == LOW ){
    Serial.println("Tap Detected.");
  }

  delay(100);
}