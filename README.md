# TinkeringProject_Autonomous_Guided_Vehicle
An AGV that can perform human-following, obstacle avoidance and logistics transport

### GOAL: Merge everything into main.ino, when we run the file, the car runs.

### Done by today :)
1. connect esp32 to battery and motor driver
2. merge IR receiver code into the .ino file
---
3. test motor control function
4. test follow human function
5. test avoid obstacle function (depends on our progress)
---
6. ppt 

# TODO:
### SOFTWARE:
##### 1. Code Integration:
Break your code into functions: <br> 
<br>
readIR() – detects or decodes IR beacon direction (instead of just sending continuously). <br>
**getUltrasonicDistance() – already done.** <br>
followHuman() – uses IR results to decide motor direction. **(need motor driver)** <br>
avoidObstacle() – uses ultrasonic to avoid. **(need motor driver)** <br> 
driveMotors(leftSpeed, rightSpeed) – low-level motor control. **(need motor driver)** <br> 

##### 2. Create an enum state machine (FOLLOW, AVOID, STOP) to manage navigation logic.

##### 3. Write a loop that checks sensors continuously.
FLOW: <br>
1. Continuously read ultrasonic distance. <br>
2. If distance < threshold value → set state = AVOID. <br>
3. Else → set state = FOLLOW. <br>
4. Run behavior based on current state. (Refer to the state machine)  <br>
        
### HARDWARE:
##### 1. Buy a Motor Driver:
Use PWM output pins to control speed and direction via motor driver (L298N, L293D, or similar). <br>
       
The ESP32 cannot directly drive motors (DC or stepper), because: <br>
ESP32 pins only supply 3.3V at ~12–20mA. <br>
Motors usually need 5–12V and hundreds of mA (or even amps). <br>

If you connect a motor directly → you’ll burn the ESP32 pin instantly.

##### 2. Power Supply:
Battery → ESP32 & motor driver (share common GND).

##### 3. Connection:
Motor driver on the same breadboard as the battery pack
ESP32 on another breadboard connected to another battery pack
Connect two breadboard together 

### TESTING:
Test motors alone with driveMotors() + stopMotors(). <br>
Test ultrasonic detection. <br>
Test IR following separately. <br>
Run full navigation loop. <br>
Debug using Serial.print() statements. <br>






        
