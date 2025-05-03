#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include "FastLED.h"

// --- LED Definitions ---
#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// --- Calibration Values ---
// Calculated from 1000ms -> 35.25 inches (89.535 cm)
// #define FORWARD_MS_PER_CM 11.17 // No longer needed as delays are hardcoded

// Turn Delays (ms at speed 200)
const int DELAY_LEFT_45 = 160;
const int DELAY_RIGHT_45 = 160; // Assume Right 45 is symmetrical to Left 45
const int DELAY_LEFT_90 = 275;

// Short delay between actions
const int STOP_DELAY = 250;

// --- Object Instances ---
DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;
DeviceDriverSet_ULTRASONIC myUltrasonic;

// --- Obstacle Detection State ---
const int OBSTACLE_THRESHOLD_CM = 5; // Distance to trigger stop (~2 inches), changed from 3
// unsigned long lastBlinkTime = 0; // No longer needed
const int BLINK_INTERVAL = 100; // Blink speed (ms) - On/Off duration
// bool ledState = false; // No longer needed

// --- Helper Function for Obstacle Check in Setup ---
// Returns true if stopped due to obstacle, false otherwise.
bool checkObstacleAndStop() {
  uint16_t distance_cm = 0;
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&distance_cm);

  // We check for > 0 because a 0 reading is usually an error/timeout
  if (distance_cm > 0 && distance_cm <= OBSTACLE_THRESHOLD_CM) { 
    Serial.println("*** OBSTACLE DETECTED DURING SETUP! Halting. ***");
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop motors
    // Halt execution by entering an infinite loop that blinks the LED red.
    while(1) { 
        // Simple Blocking Blink Logic inside the halt loop
        leds[0] = CRGB::Red;   // Red
        FastLED.show();
        delay(BLINK_INTERVAL); 
        
        leds[0] = CRGB::Black; // Off
        FastLED.show();
        delay(BLINK_INTERVAL); 
    }
    // return true; // Technically unreachable
  }
  return false; // No obstacle, safe to continue
}

void setup() {
  Serial.begin(9600); // Initialize Serial for debugging (optional)
  
  // Initialize hardware
  AppMotor.DeviceDriverSet_Motor_Init();
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Init();
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(40); // Set LED brightness (adjust as needed)
  leds[0] = CRGB::Black; // Start with LED off
  FastLED.show();

  // --- LED HARDWARE TEST ---
  Serial.println("Testing LED Blue...");
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(1000); // Keep it blue for 1 second
  leds[0] = CRGB::Black; // Turn it off again
  FastLED.show();
  Serial.println("LED Test Complete.");
  // --- END LED HARDWARE TEST ---

  
  delay(2000); // Initial pause

  // --- Mission Sequence ---
  Serial.println("Starting mission sequence...");

  // 1. Forward 48cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(643); // Calibrated delay for 48cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 2. Turn Left 45 degrees
  // Optional: Check before turn if sensor might see something sideways
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 3. Forward 26.5cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(378); // Calibrated delay for 26.5cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 4. Turn Right 45 degrees
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 5. Forward 22cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(316); // Calibrated delay for 22cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 6. Turn Right 45 degrees
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 7. Forward 36cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(495); // Calibrated delay for 36cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 8. Turn Left 45 degrees
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 9. Forward 34cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(465); // Calibrated delay for 34cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 10. Turn Right 45 degrees
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 11. Forward 19cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(278); // Calibrated delay for 19cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 12. Turn Left 90 degrees
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200);
  delay(DELAY_LEFT_90);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 13. Forward 41cm
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(540); // Calibrated delay for 41cm
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 14. Turn Right 45 degrees
  // if (checkObstacleAndStop()) return; 
  ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 200);
  delay(DELAY_RIGHT_45);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay(STOP_DELAY);

  // 15. Forward 22cm (but continue off board)
  if (checkObstacleAndStop()) return; // Check before moving
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 200);
  delay(1000); // Drive forward for 1 second (adjust as needed)
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Stop after driving off
  
  // --- Celebration Spin ---
  delay(500); // Short pause before celebrating
  ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 200); // Spin Left
  delay(2000); // Spin for 2 seconds (User changed from 1000ms)
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Final stop
  
  Serial.println("Mission sequence complete normally. Entering continuous check loop.");
}

void loop() {
  // If setup() completed normally, this loop will run.
  // We still check for obstacles here in case one appears *after* the mission.

  uint16_t distance_cm = 0;
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&distance_cm);

  // Optional: Print distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // Trigger if distance is within threshold (ignore 0 here, setup handled it)
  if (distance_cm > 0 && distance_cm <= OBSTACLE_THRESHOLD_CM) {
    // Obstacle detected post-mission!
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // Ensure stopped

    // Simple Blocking Blink Logic
    leds[0] = CRGB::Red;   // Red
    FastLED.show();
    delay(BLINK_INTERVAL); 
    
    leds[0] = CRGB::Black; // Off
    FastLED.show();
    delay(BLINK_INTERVAL); 

  } else {
    // No obstacle detected
    // Turn LED off if it was on
    if (leds[0] != CRGB::Black) { 
       leds[0] = CRGB::Black;
       FastLED.show();
    }
    // Small delay only when *not* blinking to prevent busy-looping if needed
    delay(50); 
  }
  // Note: No delay needed *inside* the if block because the blink delays provide pauses.
}

