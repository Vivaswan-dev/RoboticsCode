# RoboticsCode
This is the code that we use for our final project in APL Robotics Spring 2025

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




