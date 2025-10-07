# R6
Code for ENES 100 Section 1101 Team 2

## Notes on Code Organization
1. All functions should be called from the Main.ino file
2. Declare new functions by name in Functions.h
3. Create the function in Functions.cpp

## Public Functions
1. runProgram(bool value);  Configures state machine based on input, if bool is false the state machine will not be configured.
1. getState();  Returns the current state number from 0-??.
1. initalizeProgram(bool connecttoENES);    Connects to the ENES100 system if True otherwise configures Serial Monitor.
1. portConfiguration();  Configures ports as defined in the pinmap. 
1. serialCommunication();  Checks the Serial Monitor Queue and executes functions based on user input.
    1. Motor Test -> Tests motor functionality.
    1. Sensor Test -> Tests sensor functionality and prints values to Serial Monitor.
    1. Square Wave -> Reads current squarewave from PIN 2.
    1. Wifi Data -> Reads ENES100 data and prints values to Serial Monitor.
1. pathfindToObjective();   Moves robot from the respective starting areas to the mission area.
1. deployArm(); Prototype(Unimplemented) 
1. deployPinion();  Prototype(Unimplemented)
1. deployClaw();    Prototype(Unimplemented)
1. openClawWide();  Prototype(Unimplemented)
1. pickUpPuck();    Prototype(Unimplemented)
1. retractPinion(); Prototype(Unimplemented)
1. retractArm();    Prototype(Unimplemented)
1. detectBField();  Prototype(Unimplemented)

## Private Functions
1. stop();  Stops robot movement.
1. incrementState();    Move to next state in the state machine.
1. tankDrive(int percent, bool directioninv);  Drives the robot forward, provide percentage and a boolean, the boolean inverts the direction of movement.
1. tankTurn(int percent, bool directioninv);  Turns the robot, provide percentage and a boolean, true=Right false=Left.
1. squareWaveRead();    Reads squarewave read across pin 2 and ground, returns a true if the function was sucessful and false if it failed for any reason.
1. validateXYTheta(float x, float y, float theta); Returns true if the values are close to the last values and false if the values should be discarded. 
1. validateTheta(float theta);  Returns true if the values are close to the last value and false if the values should be discarded.
1. ALPHA    turnToAngle(float angle);  Turns the robot to a given angle, -Pi -> Pi.
1. ALPHA    drivetoPoint(double x, double y, double theta);  Drives to a given point, provide x, y, and theta. 
