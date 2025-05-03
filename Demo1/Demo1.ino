#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include "FastLED.h"

#define PIN_RBGLED 4
#define NUM_LEDS 1

CRGB leds[NUM_LEDS];

const int DELAY_LEFT_45 = 160;
const int DELAY_RIGHT_45 = 160;
const int DELAY_LEFT_90 = 275;
const int STOP_DELAY = 250;
const int OBSTACLE_THRESHOLD_CM = 5;
const int BLINK_INTERVAL = 100;

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;
DeviceDriverSet_ULTRASONIC myUltrasonic;

bool checkObstacleAndStop() {
  uint16_t distance_cm = 0;
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&distance_cm);

  if (distance_cm > 0 && distance_cm <= OBSTACLE_THRESHOLD_CM) {
    Serial.println("*** OBSTACLE DETECTED DURING SETUP! Halting. ***");
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    while(1) {
        leds[0] = CRGB::Red;
        FastLED.show();
        delay(BLINK_INTERVAL);

        leds[0] = CRGB::Black;
        FastLED.show();
        delay(BLINK_INTERVAL);
    }
  }
  return false;
}

void setup() {
  Serial.begin(9600);

  AppMotor.DeviceDriverSet_Motor_Init();
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Init();
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(40);
  leds[0] = CRGB::Black;
  FastLED.show();

  Serial.println("Testing LED Blue...");
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(1000);
  leds[0] = CRGB::Black;
  FastLED.show();
  Serial.println("LED Test Complete.");

  delay(2000);

  Serial.println("Starting mission sequence...");

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(643);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(378);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(316);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(495);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(465);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(278);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_90);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(540);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  if (checkObstacleAndStop()) return;
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  delay(500);
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(2000);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

  Serial.println("Mission sequence complete normally. Entering continuous check loop.");
}

void loop() {
  uint16_t distance_cm = 0;
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&distance_cm);

  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  if (distance_cm > 0 && distance_cm <= OBSTACLE_THRESHOLD_CM) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

    leds[0] = CRGB::Red;
    FastLED.show();
    delay(BLINK_INTERVAL);

    leds[0] = CRGB::Black;
    FastLED.show();
    delay(BLINK_INTERVAL);

  } else {
    if (leds[0] != CRGB::Black) {
       leds[0] = CRGB::Black;
       FastLED.show();
    }
    delay(50);
  }
}
