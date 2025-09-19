#ifndef Functions_h
#define Functions_h

void initalizeProgram();
void stop();
void tankDrive(int percent, bool directioninv);
void tankTurn(int percent, bool directioninv);
void drivetoPoint(double x, double y, double theta);
void turnToAngle(float angle);

#endif
