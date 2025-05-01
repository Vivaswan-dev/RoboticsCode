#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  Serial.begin(9600);
  Serial.println("Turn Calibration Test - 1 Second");
  
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);
  
  Serial.println("Starting LEFT turn for 1000ms...");
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(1000); 
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); 
  Serial.println("LEFT turn complete - measure the angle turned");
  
  delay(5000);
  
  Serial.println("Starting RIGHT turn for 1000ms...");
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(1000); 
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  Serial.println("RIGHT turn complete - measure the angle turned");
  
  Serial.println("Tests complete. Please measure both angles and calculate:");
  Serial.println("ms_per_degree = 1000 / measured_angle_in_degrees");
}

void loop() {
} 