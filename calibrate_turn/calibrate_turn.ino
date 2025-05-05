#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); 
  
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(160); // Initial guess for 45 degrees right
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); 
}

void loop() {
  
} 
