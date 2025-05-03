#ifndef _APPLICATIONFUNCTIONSET_XXX0_H_
#define _APPLICATIONFUNCTIONSET_XXX0_H_

#include <stdint.h> // Required for uint8_t

/* Motion control direction enumeration */
enum SmartRobotCarMotionControl {
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};

/* Structure for application state (if needed by other files) */
struct Application_xxx {
  SmartRobotCarMotionControl Motion_Control;
};

/* Function prototype for motion control */
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

#endif // _APPLICATIONFUNCTIONSET_XXX0_H_ 