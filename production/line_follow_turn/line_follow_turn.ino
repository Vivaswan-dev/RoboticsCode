#include <arduino.h>
#include <Wire.h>

#include "LineTracker_Driver.h" 
#include "Motor_Driver.h"
#include "MPU6050_getdata.h"

const byte ENCODER_LEFT_PIN = 2;
const byte ENCODER_RIGHT_PIN = 3;
const int ENCODER_PULSES_PER_ROTATION = 192;
const float WHEEL_DIAMETER_CM = 6.7;

// --- Time-Based Movement Calibration (Tune These!) ---
const int MS_PER_CM_FORWARD = 80;     // Approx. milliseconds to move 1cm straight forward
const int MS_PER_CM_LINEFOLLOW = 100; // Approx. milliseconds to move 1cm while line following

const unsigned long DURATION_LF_48CM    = 48UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_FWD_13CM   = 13UL * MS_PER_CM_FORWARD;
const unsigned long DURATION_LF_13_5CM  = (unsigned long)(13.5 * MS_PER_CM_LINEFOLLOW);
const unsigned long DURATION_LF_22CM    = 22UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_LF_10CM    = 10UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_FWD_6CM    =  6UL * MS_PER_CM_FORWARD;
const unsigned long DURATION_LF_20CM    = 20UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_LF_34CM    = 34UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_LF_19CM    = 19UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_LF_41CM    = 41UL * MS_PER_CM_LINEFOLLOW;
const unsigned long DURATION_LF_22CM_B  = 22UL * MS_PER_CM_LINEFOLLOW;

const uint8_t LINE_FOLLOW_SPEED = 120;
const uint8_t TURN_SPEED = 100;
const uint8_t FORWARD_SPEED = 130;

const float KP = 0.6;
const float KI = 0.01;
const float KD = 0.2;
const float SENSOR_THRESHOLD = 500;

const float TURN_ANGLE_TOLERANCE = 2.0;

enum RobotState {
  INITIALIZING,
  LINE_FOLLOW_48CM,
  ROTATE_LEFT_40DEG,
  FORWARD_13CM,
  LINE_FOLLOW_13_5CM,
  ROTATE_RIGHT_30DEG,
  LINE_FOLLOW_22CM,
  ROTATE_RIGHT_40DEG,
  LINE_FOLLOW_10CM,
  FORWARD_6CM,
  LINE_FOLLOW_20CM,
  ROTATE_LEFT_20DEG,
  LINE_FOLLOW_34CM,
  ROTATE_RIGHT_30DEG_B,
  LINE_FOLLOW_19CM,
  ROTATE_LEFT_90DEG,
  LINE_FOLLOW_41CM,
  ROTATE_RIGHT_20DEG,
  LINE_FOLLOW_22CM_B,
  STOPPED,
  ERROR_STATE
};
RobotState currentState = INITIALIZING;
RobotState nextState = INITIALIZING;

DeviceDriverSet_ITR20001 LineSensors;
DeviceDriverSet_Motor Motors;
MPU6050_getdata Mpu6050;

unsigned long stateStartTime = 0;

float integral = 0;
float lastError = 0;

float currentYaw = 0.0;
float targetYaw = 0.0;
float startYaw = 0.0;

void stopMotors() {
  Motors.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
  delay(50);
}

float angleDifference(float angle1, float angle2) {
  float diff = angle1 - angle2;
  while (diff <= -180.0) diff += 360.0;
  while (diff > 180.0) diff -= 360.0;
  return diff;
}

bool updateCurrentYaw() {
    return Mpu6050.MPU6050_dveGetEulerAngles(&currentYaw); 
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

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  Motors.DeviceDriverSet_Motor_control(direction_just, speedA, direction_just, speedB, control_enable);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Robot Initializing...");
  currentState = INITIALIZING;

  Wire.begin();

  if (!LineSensors.DeviceDriverSet_ITR20001_Init()) {
    Serial.println("ERROR: Line Sensor Init Failed!");
    currentState = ERROR_STATE;
  }
  Motors.DeviceDriverSet_Motor_Init();

  Serial.println("Initializing MPU6050...");
  if (!Mpu6050.MPU6050_dveInit()) {
     Serial.println("ERROR: MPU6050 Init Failed!");
     currentState = ERROR_STATE;
  } else {
     Serial.println("Calibrating MPU6050...");
     if (!Mpu6050.MPU6050_calibration()) {
         Serial.println("Warning: MPU6050 Calibration Failed/Not Supported?");

     } else {
        Serial.println("MPU6050 Calibration Complete.");
     }
  }
  
  if (currentState == INITIALIZING) {
    Serial.println("Initialization Complete. Starting Sequence.");
    if (!updateCurrentYaw()){
        Serial.println("ERROR: Failed to get initial Yaw!");
        currentState = ERROR_STATE;
    } else {
        startYaw = currentYaw;
        targetYaw = currentYaw;
        Serial.print("Initial Yaw: "); Serial.println(startYaw);
        currentState = LINE_FOLLOW_48CM;
    }
  } else {
     Serial.println("Entering Error State due to initialization failure.");
     stopMotors(); 
  }
}

void loop() {
  float currentDistance = 0;
  bool angleReached = false;

  if (currentState != ERROR_STATE && currentState != INITIALIZING) {
      if (!updateCurrentYaw()){
          Serial.println("ERROR: Failed to read Yaw mid-sequence!");
          currentState = ERROR_STATE;
          stopMotors();
      }
  }

  switch (currentState) {

    case INITIALIZING:

      delay(100);
      break;

    case LINE_FOLLOW_48CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_48CM) {
        Serial.println("Step: LINE_FOLLOW_48CM complete (Time-based).");
        nextState = ROTATE_LEFT_40DEG; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_LEFT_40DEG:

      if (targetYaw == startYaw) { 
          targetYaw = startYaw - 40.0; 
          while (targetYaw <= -180.0) targetYaw += 360.0;
          Serial.print("Turning Left. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_back, TURN_SPEED, direction_just, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
      if (angleReached) {
          Serial.println("Step: ROTATE_LEFT_40DEG complete.");
          nextState = FORWARD_13CM;
          currentState = STOPPED; 
      }
      break;

    case FORWARD_13CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      Motors.DeviceDriverSet_Motor_control(direction_just, FORWARD_SPEED, direction_just, FORWARD_SPEED, control_enable);
      if (millis() - stateStartTime >= DURATION_FWD_13CM) {
        Serial.println("Step: FORWARD_13CM complete (Time-based).");
        nextState = LINE_FOLLOW_13_5CM;
        currentState = STOPPED;
      }
      break;
      
    case LINE_FOLLOW_13_5CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_13_5CM) {
        Serial.println("Step: LINE_FOLLOW_13_5CM complete (Time-based).");
        nextState = ROTATE_RIGHT_30DEG; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_RIGHT_30DEG:
       if (targetYaw == startYaw) { 
          targetYaw = startYaw + 30.0;
          while (targetYaw > 180.0) targetYaw -= 360.0;
          Serial.print("Turning Right. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_just, TURN_SPEED, direction_back, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
       if (angleReached) {
          Serial.println("Step: ROTATE_RIGHT_30DEG complete.");
          nextState = LINE_FOLLOW_22CM;
          currentState = STOPPED; 
      }
      break;

    case LINE_FOLLOW_22CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_22CM) {
        Serial.println("Step: LINE_FOLLOW_22CM complete (Time-based).");
        nextState = ROTATE_RIGHT_40DEG; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_RIGHT_40DEG:
       if (targetYaw == startYaw) { 
          targetYaw = startYaw + 40.0;
          while (targetYaw > 180.0) targetYaw -= 360.0;
           Serial.print("Turning Right. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_just, TURN_SPEED, direction_back, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
       if (angleReached) {
          Serial.println("Step: ROTATE_RIGHT_40DEG complete.");
          nextState = LINE_FOLLOW_10CM;
          currentState = STOPPED; 
      }
      break;

    case LINE_FOLLOW_10CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_10CM) {
        Serial.println("Step: LINE_FOLLOW_10CM complete (Time-based).");
        nextState = FORWARD_6CM; 
        currentState = STOPPED; 
      }
      break;

    case FORWARD_6CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      Motors.DeviceDriverSet_Motor_control(direction_just, FORWARD_SPEED, direction_just, FORWARD_SPEED, control_enable);
      if (millis() - stateStartTime >= DURATION_FWD_6CM) {
        Serial.println("Step: FORWARD_6CM complete (Time-based).");
        nextState = LINE_FOLLOW_20CM;
        currentState = STOPPED;
      }
      break;
      
    case LINE_FOLLOW_20CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_20CM) {
        Serial.println("Step: LINE_FOLLOW_20CM complete (Time-based).");
        nextState = ROTATE_LEFT_20DEG; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_LEFT_20DEG:
       if (targetYaw == startYaw) { 
          targetYaw = startYaw - 20.0; 
          while (targetYaw <= -180.0) targetYaw += 360.0;
          Serial.print("Turning Left. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_back, TURN_SPEED, direction_just, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
       if (angleReached) {
          Serial.println("Step: ROTATE_LEFT_20DEG complete.");
          nextState = LINE_FOLLOW_34CM;
          currentState = STOPPED; 
      }
      break;

    case LINE_FOLLOW_34CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_34CM) {
        Serial.println("Step: LINE_FOLLOW_34CM complete (Time-based).");
        nextState = ROTATE_RIGHT_30DEG_B; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_RIGHT_30DEG_B:
       if (targetYaw == startYaw) { 
          targetYaw = startYaw + 30.0;
          while (targetYaw > 180.0) targetYaw -= 360.0;
           Serial.print("Turning Right. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_just, TURN_SPEED, direction_back, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
       if (angleReached) {
          Serial.println("Step: ROTATE_RIGHT_30DEG_B complete.");
          nextState = LINE_FOLLOW_19CM;
          currentState = STOPPED; 
      }
      break;

    case LINE_FOLLOW_19CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_19CM) {
        Serial.println("Step: LINE_FOLLOW_19CM complete (Time-based).");
        nextState = ROTATE_LEFT_90DEG; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_LEFT_90DEG:
       if (targetYaw == startYaw) { 
          targetYaw = startYaw - 90.0; 
          while (targetYaw <= -180.0) targetYaw += 360.0;
           Serial.print("Turning Left. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_back, TURN_SPEED, direction_just, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
       if (angleReached) {
          Serial.println("Step: ROTATE_LEFT_90DEG complete.");
          nextState = LINE_FOLLOW_41CM;
          currentState = STOPPED; 
      }
      break;

    case LINE_FOLLOW_41CM:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_41CM) {
        Serial.println("Step: LINE_FOLLOW_41CM complete (Time-based).");
        nextState = ROTATE_RIGHT_20DEG; 
        currentState = STOPPED; 
      }
      break;

    case ROTATE_RIGHT_20DEG:
       if (targetYaw == startYaw) { 
          targetYaw = startYaw + 20.0;
          while (targetYaw > 180.0) targetYaw -= 360.0;
           Serial.print("Turning Right. Start Yaw: "); Serial.print(startYaw); 
          Serial.print(", Target Yaw: "); Serial.println(targetYaw);
      }
      Motors.DeviceDriverSet_Motor_control(direction_just, TURN_SPEED, direction_back, TURN_SPEED, control_enable); 
      if (abs(angleDifference(currentYaw, targetYaw)) <= TURN_ANGLE_TOLERANCE) {
          angleReached = true;
      }
       if (angleReached) {
          Serial.println("Step: ROTATE_RIGHT_20DEG complete.");
          nextState = LINE_FOLLOW_22CM_B;
          currentState = STOPPED; 
      }
      break;

    case LINE_FOLLOW_22CM_B:
      if (stateStartTime == 0) { stateStartTime = millis(); }
      followLine();
      if (millis() - stateStartTime >= DURATION_LF_22CM_B) {
        Serial.println("Step: LINE_FOLLOW_22CM_B complete (Time-based).");
        Serial.println("--- SEQUENCE COMPLETE ---");
        nextState = STOPPED; 
        currentState = STOPPED; 
      }
      break;

    case STOPPED:
      stopMotors();

      integral = 0; 
      lastError = 0;
      startYaw = currentYaw; 
      targetYaw = startYaw; 
      stateStartTime = 0; 
      currentState = nextState; 
      delay(100); 
      break;

    case ERROR_STATE:

      stopMotors();

      Serial.println("ERROR STATE - EXECUTION HALTED");
      delay(1000);
      break;
  } 

  delay(10); 
} 
