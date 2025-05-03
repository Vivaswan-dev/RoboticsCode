#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

// --- Calibration Values ---
// Calculated from 1000ms -> 35.25 inches (89.535 cm)
#define FORWARD_MS_PER_CM 11.17 

// Turn Delays (ms at speed 200)
const int DELAY_LEFT_45 = 160;
const int DELAY_RIGHT_45 = 160; // Assume Right 45 is symmetrical to Left 45
const int DELAY_LEFT_90 = 275;

// Short delay between actions
const int STOP_DELAY = 250;

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0; // Consider if this global instance is used/needed

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000); // Initial pause

  // 1. Forward 48cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(48 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 2. Turn Left 45 degrees
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 3. Forward 26.5cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(26.5 * FORWARD_MS_PER_CM) ); // Cast to int for delay
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 4. Turn Right 45 degrees (Changed from 40)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 5. Forward 22cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(22 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 6. Turn Right 45 degrees (Changed from 40)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 7. Forward 36cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(36 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 8. Turn Left 45 degrees
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 9. Forward 34cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(34 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 10. Turn Right 45 degrees
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 11. Forward 19cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(19 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 12. Turn Left 90 degrees
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_90);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 13. Forward 41cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(41 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 14. Turn Right 45 degrees (Changed from 50)
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 15. Forward 22cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay( (int)(22 * FORWARD_MS_PER_CM) );
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  // No final delay needed unless something comes after
}

void loop() {
  // Nothing further needed in loop for this demo
}
