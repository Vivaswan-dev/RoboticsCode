#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

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

// Calculated from 1000ms -> 35.25 inches (89.535 cm)
#define FORWARD_MS_PER_CM 11.17

void setup() {
  Serial.begin(9600); // Initialize Serial communication
  while (!Serial); // Wait for Serial port to connect (needed for some boards like Leonardo)
  Serial.println("Starting Forward Calibration Test...");

  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Wait 2 seconds before starting

  Serial.println("Moving forward for 1000ms...");
  // Calibration: Move Forward for 1 second (1000ms) at speed 200
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop
  Serial.println("Movement complete. Measure distance.");
  Serial.println("Calculate: ms_per_cm = 1000 / distance_in_cm");
}

void loop() {
  // Nothing needed
} 