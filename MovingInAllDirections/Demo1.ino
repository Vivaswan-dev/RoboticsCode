#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(2000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1200);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(450);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(450);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1600);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(900);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(550);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
}

void loop() {
}
