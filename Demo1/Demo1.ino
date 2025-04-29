#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Initial 2-second pause

  // Run forward at speed 200 for 5 seconds (5000 milliseconds)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(5000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop after 5 seconds

  // End of setup sequence
}

void loop() {
  // Keep loop empty
}
