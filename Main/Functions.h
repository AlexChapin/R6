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
void newPathfindToObjective();
void tankDriveWallRun(int pwm);
void fullScore();
bool obstacleNavigate();
bool detectBField();
void pathfindToLog();
void driveOverLog();
void tankDrive(int pwm);
void tankTurn(int pwm);
void pinionDrive(int pwm);
void turnToAngle(float angle);
bool driveToPoint(double x, double y, double theta);
bool driveToPointObstructed(double x, double y, double theta);
bool squareWaveRead();



#endif
