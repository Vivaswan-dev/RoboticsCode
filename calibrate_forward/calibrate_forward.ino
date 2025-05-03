#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

#define FORWARD_MS_PER_CM 11.17

const float TARGET_DISTANCE_CM = 41.0;
const int TEST_DELAY_MS = 540;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("---------------------------------");
  Serial.print("Starting Forward Calibration for ");
  Serial.print(TARGET_DISTANCE_CM);
  Serial.println(" cm");
  Serial.print("Testing Delay: ");
  Serial.print(TEST_DELAY_MS);
  Serial.println(" ms");
  Serial.println("---------------------------------");


  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);

  Serial.println("Moving forward...");
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(TEST_DELAY_MS);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  Serial.println("Movement complete.");
  Serial.print("Measure actual distance. Target was: ");
  Serial.print(TARGET_DISTANCE_CM);
  Serial.println(" cm");
  Serial.println("Adjust TEST_DELAY_MS and re-run if needed.");
}

void loop() {
}