#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Initial pause

  // Sequence from info.txt with calculated delays

  // 1. Move 48cm (727ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(727);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250); // Brief pause between steps

  // 2. turn 45 degrees left (500ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 3. move 26.5cm (401ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(401);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 4. turn 40 degrees right (444ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(444);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 5. move 22 cm forward (333ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(333);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 6. turn 40 degrees right (444ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(444);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 7. move 36cm forward (545ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(545);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 8. turn 45 degrees left (500ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 9. move 34 cm forward (515ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(515);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 10. turn 45 degrees right (500ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 11. move forward 19cm (288ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(288);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 12. turn 90 degrees left (1000ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 13. turn right 50 degrees (556ms estimated)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(556);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(250);

  // 14. go 22cm forward (333ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(333);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Final stop
}

void loop() {
  // Keep loop empty
}
