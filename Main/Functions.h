#ifndef Functions_h
#define Functions_h

int getState();
void incrementState();
void pathfindToObjective();
void runProgram(bool value);
float ultrasonicDistance1();
float ultrasonicDistance2();
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
bool detectBField();
void tankDrive(int percent);
void tankTurn(int percent);
void turnToAngle(float angle);
void driveToPoint(double x, double y, double theta);


#endif
