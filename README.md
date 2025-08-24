# TinkeringProject_Autonomous_Guided_Vehicle
An AGV that can perform human-following, obstacle avoidance and logistics transport

### GOAL: Merge everything into a file, when we run the file, the car runs.
emit ir not integrate with car
test if new code can run then integrate
readIR code, how many ir receivers we need
# TODO:
### SOFTWARE:
##### 1. Code Integration:
         Break your code into functions
            readIR() – detects or decodes IR beacon direction (instead of just sending continuously).
            **getUltrasonicDistance() – already done.**
            followHuman() – uses IR results to decide motor direction.
            avoidObstacle() – uses ultrasonic to avoid.
            driveMotors(leftSpeed, rightSpeed) – low-level motor control.

##### 2. Create an enum state machine (FOLLOW, AVOID, STOP) to manage navigation logic.

##### 3. Write a loop that checks sensors continuously.
        FLOW: 
        Continuously read ultrasonic distance.
        If distance < threshold → set state = AVOID.
        Else → set state = FOLLOW.
        Run behavior based on current state. (Refer to the state machine) 
        
### HARDWARE:
##### 1. ESP32 → Motor Driver:
       Use PWM output pins to control speed and direction via motor driver (L298N, L293D, or similar).
       Alt: 

##### 2. Power Supply:
        Battery → ESP32 & motor driver (share common GND).


### TESTING:
   Test motors alone with driveMotors() + stopMotors().
   Test ultrasonic detection.
   Test IR following separately.
   Run full navigation loop.
   Debug using Serial.print() statements.






        
