#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

// Define the motion control enum values if they are not automatically available
// (Remove this if ApplicationFunctionSet_xxx0.cpp defines them globally)
/*
enum SmartRobotCarMotionControl {
  Forward = 0, Backward = 1, Left = 2, Right = 3,
  LeftForward = 4, LeftBackward = 5, RightForward = 6, RightBackward = 7,
  stop_it = 8
};
*/

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Wait 2 seconds before starting

  // Calibration: Move Forward for 1 second (1000ms) at speed 200
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop
}

void loop() {
  // Nothing needed
} 