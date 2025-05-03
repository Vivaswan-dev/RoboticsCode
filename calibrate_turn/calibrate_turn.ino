#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  // Initialize serial communication for reporting (optional)
  Serial.begin(9600);
  Serial.println("Right Turn Calibration Test - Target: 45 Degrees");
  
  // Initialize motors
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Initial 2-second pause
  
  // --- TEST: Turn RIGHT --- 
  Serial.println("Starting RIGHT turn...");
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(160); // Initial guess for 45 degrees right
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop
  Serial.println("RIGHT turn complete - measure the angle turned.");
  
}

void loop() {
  // Nothing here - test runs once in setup()
} 