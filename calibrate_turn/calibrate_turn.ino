#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  // Initialize serial communication for reporting (optional)
  // Serial.begin(9600);
  // Serial.println("Turn Calibration Test - 1 Second Left Turn");
  
  // Initialize motors
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Initial 2-second pause
  
  // TEST: Turn LEFT for exactly 1 second (1000ms)
  // Serial.println("Starting LEFT turn for 1000ms...");
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(178); // Turn for exactly 1 second         //160 is 45 degrees //275 is 90 degrees //142 is 40 degrees //178 50 degrees
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop
  // Serial.println("LEFT turn complete - measure the angle turned");
  
  // Serial.println("Test complete. Please measure the angle and calculate:");
  // Serial.println("ms_per_degree = 1000 / measured_angle_in_degrees");
}

void loop() {
  // Nothing here - test runs once in setup()
} 