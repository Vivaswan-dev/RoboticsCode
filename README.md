###  1. Requirements: 
- Navigate the Lunar Highway: Drive along a predetermined path (the "Lunar Highway") without veering off course.
- Obstacle Avoidance: Detect and avoid moon rocks of various sizes  scattered along the path.
- Alert System (Challenge Goal): Implement a warning system to alert mission control of any obstacles or challenges encountered
### 2. Ideas:
- Ultrasonic Sensor: Used as an obstacle avoidance tool. The ultrasonic sensor should detect the object within a foot from it to ensure that there are no errors.
- Infrared sensor: Use the black lines in to move
    - If only the left infrared sensor detects black lines, move the robot towards the left side.
    - If only the right infrared sensor detects black lines, move the robot towards the right side.
    - If their is no more black lines when it move in a direction back track and find a part of the line where there is a black line and follow. 
    - If there is an obstacle that is in the way of the black path, the ultrasonic sensor will work with the obstacle avoidance and the robot will move backwards, using the color sensor to find another part of the black line.
    - If the robot goes back where it came from,
- LED: Use the LED for the challenge problem, if the ultrasonic sensor  detects an object in front the robot then it will flash the LED red for until the robot does not detect the object anymore. 
- When the Robot reaches the end make the LED flash green to show that it made it.
### 3. Research:
## Pseudo Code: 
- Instructions: 
Where to start it

Line following measure distance 

if line following reaches the distance or time stop it 

manual code for angle correction and small forward 

after continue line following.

Start time after detecting black.





### Important snippets: 
Demo1.ino

```
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);
  for (Application_SmartRobotCarxxx0.Motion_Control = 0; Application_SmartRobotCarxxx0.Motion_Control < 9; Application_SmartRobotCarxxx0.Motion_Control = Application_SmartRobotCarxxx0.Motion_Control + 1)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, 200 /*speed*/);
    delay(1000);
  }
}

void loop() {
  
}
```
ApplicationFunctionSet_xxx0.cpp

```
#include "DeviceDriverSet_xxx0.h"

extern DeviceDriverSet_Motor AppMotor;

/*运动方向控制序列*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
};

extern Application_xxx Application_SmartRobotCarxxx0;

static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Backward:
    /* code */

      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Left:
   
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    break;
  }
}
```


