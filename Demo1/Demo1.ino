#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(727);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(89);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(401);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(79);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(333);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(79);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(545);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(89);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(515);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(89);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(288);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(177);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(621);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(99);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(333);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
}

void loop() {
}
