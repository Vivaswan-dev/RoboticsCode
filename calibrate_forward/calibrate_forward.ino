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
  
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(TEST_DELAY_MS);
  
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
}

void loop() {
}
