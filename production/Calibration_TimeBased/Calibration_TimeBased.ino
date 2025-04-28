#include <arduino.h>

#include "Motor_Driver.h"
#include "LineTracker_Driver.h"

const unsigned long TEST_DURATION_MS = 5000; // How long each test runs (milliseconds)
const uint8_t FORWARD_SPEED = 130;     // Speed for straight forward test
const uint8_t LINE_FOLLOW_SPEED = 120; // Base speed for line follow test

const float KP = 0.6;
const float KI = 0.01;
const float KD = 0.2;
const float SENSOR_THRESHOLD = 500;

DeviceDriverSet_Motor Motors;
DeviceDriverSet_ITR20001 LineSensors;

float integral = 0;
float lastError = 0;

void stopMotors() {
  Motors.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
  delay(100);
}

void resetPID() {
  integral = 0;
  lastError = 0;
}

void followLine() {
  float sensorL = LineSensors.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
  float sensorM = LineSensors.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
  float sensorR = LineSensors.DeviceDriverSet_ITR20001_getAnaloguexxx_R();

  float error = 0;
  bool onLineL = sensorL > SENSOR_THRESHOLD;
  bool onLineM = sensorM > SENSOR_THRESHOLD;
  bool onLineR = sensorR > SENSOR_THRESHOLD;

  if (onLineL && !onLineM && !onLineR) error = -2.0;
  else if (onLineL && onLineM && !onLineR) error = -1.0;
  else if (!onLineL && onLineM && !onLineR) error = 0.0;
  else if (!onLineL && onLineM && onLineR) error = 1.0;
  else if (!onLineL && !onLineM && onLineR) error = 2.0;
  else if (!onLineL && !onLineM && !onLineR) error = lastError;
  else error = 0.0;

  integral += error;
  float derivative = error - lastError;
  float correction = KP * error + KI * integral + KD * derivative;
  lastError = error;

  int speedA = LINE_FOLLOW_SPEED - correction;
  int speedB = LINE_FOLLOW_SPEED + correction;

  speedA = constrain(speedA, -255, 255);
  speedB = constrain(speedB, -255, 255);

  boolean dirA = (speedA >= 0) ? direction_just : direction_back;
  boolean dirB = (speedB >= 0) ? direction_just : direction_back;

  uint8_t absSpeedA = abs(speedA);
  uint8_t absSpeedB = abs(speedB);

  absSpeedA = constrain(absSpeedA, 0, 255);
  absSpeedB = constrain(absSpeedB, 0, 255);

  Motors.DeviceDriverSet_Motor_control(dirA, absSpeedA, dirB, absSpeedB, control_enable);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing Motors...");
  Motors.DeviceDriverSet_Motor_Init();

  Serial.println("Initializing Line Sensors...");
  if (!LineSensors.DeviceDriverSet_ITR20001_Init()) {
    Serial.println("ERROR: Line Sensor Init Failed!");
    while (1);
  }

  Serial.println("\n--- Calibration Sketch Ready ---");
  Serial.print("Test duration: ");
  Serial.print(TEST_DURATION_MS);
  Serial.println(" ms");
  Serial.println("Enter command via Serial Monitor:");
  Serial.println("  'F' - Run Forward Test");
  Serial.println("  'L' - Run Line Follow Test (Place on a straight line!)");
  Serial.println("---------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    Serial.println();

    float distance_cm = 0;
    float calculated_ms_per_cm = 0;

    if (command == 'F' || command == 'f') {
      Serial.println("Starting Forward Test...");
      Serial.print("Running motors forward at speed ");
      Serial.print(FORWARD_SPEED);
      Serial.print(" for ");
      Serial.print(TEST_DURATION_MS);
      Serial.println(" ms.");
      delay(1000);

      unsigned long startTime = millis();
      Motors.DeviceDriverSet_Motor_control(direction_just, FORWARD_SPEED, direction_just, FORWARD_SPEED, control_enable);

      while (millis() - startTime < TEST_DURATION_MS) {
        delay(10); 
      }

      stopMotors();
      Serial.println(">>> Forward Test Complete <<<");
      Serial.println("Measure the distance traveled straight forward (in cm).");
      Serial.print("Enter distance (cm) and press Enter: ");

      while (Serial.available() <= 0) {
         delay(50);
      }
      distance_cm = Serial.parseFloat();
      while(Serial.available() > 0) { Serial.read(); }

      if (distance_cm > 0) {
          calculated_ms_per_cm = (float)TEST_DURATION_MS / distance_cm;
          Serial.print("===> Calculated Value: MS_PER_CM_FORWARD = ");
          Serial.println(calculated_ms_per_cm);
          Serial.println("Copy this value and paste it into line_follow_turn.ino");
      } else {
          Serial.println("Invalid distance entered.");
      }
      Serial.println("---------------------------------");
      Serial.println("Enter 'F' or 'L' for next test...");
      Serial.println("---------------------------------");


    } else if (command == 'L' || command == 'l') {
      Serial.println("Starting Line Follow Test...");
      Serial.println("Place robot centered on a STRAIGHT black line.");
      Serial.print("Running line following at base speed ");
      Serial.print(LINE_FOLLOW_SPEED);
      Serial.print(" for ");
      Serial.print(TEST_DURATION_MS);
      Serial.println(" ms.");
      delay(1000);

      resetPID(); 
      unsigned long startTime = millis();

      while (millis() - startTime < TEST_DURATION_MS) {
        followLine(); 
        delay(5);    
      }

      stopMotors();
      Serial.println(">>> Line Follow Test Complete <<<");
      Serial.println("Measure the distance traveled ALONG THE LINE (in cm).");
      Serial.print("Enter distance (cm) and press Enter: ");

      while (Serial.available() <= 0) {
         delay(50);
      }
      distance_cm = Serial.parseFloat();
      while(Serial.available() > 0) { Serial.read(); }
      
      if (distance_cm > 0) {
          calculated_ms_per_cm = (float)TEST_DURATION_MS / distance_cm;
          Serial.print("===> Calculated Value: MS_PER_CM_LINEFOLLOW = ");
          Serial.println(calculated_ms_per_cm);
          Serial.println("Copy this value and paste it into line_follow_turn.ino");
      } else {
          Serial.println("Invalid distance entered.");
      }
      Serial.println("---------------------------------");
      Serial.println("Enter 'F' or 'L' for next test...");
      Serial.println("---------------------------------");


    } else if (command != '\n' && command != '\r') {
      Serial.println("Invalid command.");
      Serial.println("Enter 'F' for Forward Test or 'L' for Line Follow Test.");
      Serial.println("---------------------------------");
    }
  }
}
