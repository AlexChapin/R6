#ifndef Functions_h
#define Functions_h

int getState();
void pathfindToObjective();
void runProgram(bool value);
void initalizeProgram(bool connecttoENES);
void serialCommunication();
void portConfiguration();
void stop();
void deployArm();
void deployPinion();
void deployClaw();
void openClawWide();
void pickUpPuck();
void retractPinion();
void retractArm();
void detectBField();
void tankDrive(int percent, bool directioninv);
void tankTurn(int percent, bool directioninv);
void turnToAngle(float angle);
void driveToPoint(double x, double y, double theta);


#endif
