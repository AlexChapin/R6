#ifndef Functions_h
#define Functions_h

bool detectBField();
bool driveToPoint(double x, double y, double theta);
bool squareWaveRead();

float ultrasonicDistance1();
float ultrasonicDistance2();
float unifiedUltrasonicClosest();

int getState();

void connectCheck();
void driveOverLog();
void fullScore();
void initalizeProgram(bool connectToENES);
void incrementState();
void obstacleNavigate();
void pathfindToObjective();
void pinionDrive(int pwm);
void portConfiguration();
void runProgram(bool value);
void serialCommunication();
void stop();
void tankDrive(int pwm);
void tankDriveWallRun(int pwm);
void tankTurn(int pwm);
void turnToAngle(float angle);
void victory();


#endif
