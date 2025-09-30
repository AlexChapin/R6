# R6
Code for ENES 100 Section 1101 Team 2

## Notes on Code Organization
1. All functions should be called from the Main.ino file
2. Declare new functions by name in Functions.h
3. Create the function in Functions.cpp

## Current Functions
1. stop();  Stops robot movement.
1. initalizeProgram(bool connecttoENES);    Connects to the ENES100 system if True otherwise configures Serial Monitor.
1. portConfiguration();  Configures ports as defined in the pinmap. 
1. tankDrive(int percent, bool directioninv);  Drives the robot forward, provide percentage and a boolean, the boolean inverts the direction of movement.
1. tankTurn(int percent, bool directioninv);  Turns the robot, provide percentage and a boolean, true=Right false=Left.
1. squareWaveRead();    Reads squarewave read across pin 2 and ground, returns a true if the function was sucessful and false if it failed for any reason.
1. serialCommunication();  Checks the Serial Monitor Queue and executes functions based on user input
    1. Motor Test -> Tests Motor Functionality.
    1. Sensor Test -> Tests Sensor Functionality and prints values to Serial Monitor.
    1. Square Wave -> Reads Current Squarewave from PIN 2.
1. ALPHA    turnToAngle(float angle);  Turns the robot to a given angle, -Pi -> Pi.
1. ALPHA    drivetoPoint(double x, double y, double theta);  Drives to a given point, provide x, y, and theta. 
