# R6
Code for ENES 100 Section 1101 Team 2

## Notes on Code Organization
1. All functions should be called from the Main.ino file
2. Declare new functions by name in Functions.h
3. Create the function in Functions.cpp

## Current Functions
1. stop();  Stops robot movement.
2. tankDrive(int percent, bool directioninv);  Drives the robot forward, provide percentage and a boolean, the boolean inverts the direction of movement.
3. tankTurn(int percent, bool directioninv);  Turns the robot, provide percentage and a boolean, true=Right false=Left.
4. turnToAngle(float angle);  Turns the robot to a given angle, -Pi -> Pi
5. ALPHA drivetoPoint(double x, double y, double theta);  Drives to a given point, provide x, y, and theta.
