#ifndef Functions_h
#define Functions_h

void incrementState();
int getState();
void pathfindToObjective();
void runProgram(bool value);
void initalizeProgram(bool connecttoENES);
void serialCommunication();
bool squareWaveRead();
void portConfiguration();
void stop();
void tankDrive(int percent, bool directioninv);
void tankTurn(int percent, bool directioninv);
void turnToAngle(float angle);
void driveToPoint(double x, double y, double theta);
void deployArm();
void deployPinion();
void deployClaw();
void openClawWide();
void pickUpPuck();
void retractPinion();
void retractArm();
void detectBField();

#endif
