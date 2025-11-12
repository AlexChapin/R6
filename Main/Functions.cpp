/*
    Functions.cpp (Primary)
*/
#include "src\NewPing\NewPing.h"
#include "src\Enes100\Enes100.h"
#include "src\PID\PID_v1.h"
#include "Servo.h"
#include "Functions.h"
int state; 

//Rotational PID
double Setpoint, Input, Output;
double Kp=15, Ki=10, Kd=3; //TUNE
PID angleController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Ultrasonic Configs
#define TRIGGER_PIN1 8
#define ECHO_PIN1 9
#define MAX_DISTANCE 50
#define TRIGGER_PIN2 10
#define ECHO_PIN2 11
NewPing ultrasonic1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing ultrasonic2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

//Servo Configs
Servo armServo;
Servo clawServo;

//Motor Configs
#define leftIN1 22
#define leftIN2 23
#define leftEN 2
#define rightIN1 24
#define rightIN2 25
#define rightEN 3
#define pinionIN1 26
#define pinionIN2 27
#define pinionEN 4

double percentToPWM = 255/100;


void runProgram(bool runAuton){
    if(runAuton){
        state = 1;
    }
    else{
        state = -1;
    }
}

void incrementState(){
    state++;
}

int getState(){
    return state;
}

void initalizeProgram(bool connectToENES){ 
    Serial.begin(9600);
    Serial.println("");
    Serial.println("");
    Serial.println("Serial Monitor Connected!");
    if (connectToENES){
        int wifiModuleTX = 53;
        int wifiModuleRX = 51;
        int roomNumber = 1116;
        int markerId = 200;
        Enes100.begin("R6", DATA, markerId, roomNumber, wifiModuleTX, wifiModuleRX);
        if (Enes100.state() == 0x01){
            Serial.println("WiFi Connected!");
        }
        else{
            Serial.println("Wifi Failure!!!!!");
        }
        Enes100.println("R6 Connected to Vision!");
    }
    else{
        Serial.println("WARNING! It is not Recommended to Run Autonomously Without WiFi! Disabling Autonomous Mode");
        state = -1;
    }
}

float ultrasonicDistance1(){
    unsigned int uS = ultrasonic1.ping(MAX_DISTANCE);
    if (uS != 0) {
    return ultrasonic1.convert_in(uS);
  }
  else return -1;
}

float ultrasonicDistance2(){
    unsigned int uS = ultrasonic2.ping(MAX_DISTANCE);
    if (uS != 0) {
    return ultrasonic2.convert_in(uS);
  }
  else return -1;
}

float unifiedUltrasonicClosest(){
    if(ultrasonicDistance1() > ultrasonicDistance2()){
        return ultrasonicDistance2();
    }
    else{
        return ultrasonicDistance1();
    }
}

void pathfindToObjective(){
    Serial.println("Path Finding To Objective");
    int x = Enes100.getX();
    int y = Enes100.getY();
    if((x < .4 || x > .9) || (y < 0.25 || y > 1.75)){
        Serial.println("FAILED to Find Valid Field Location, Ensure Wifi is Connected and Robot is on the Field!");
        stop();
        delay(1000);
        return;
    }
    bool top = y > 1;

    switch (top) {
        case true:
            turnToAngle(-PI / 2);
            driveToPoint(0.55, 0.55, -PI / 2);
            break;
    
        case false:
            turnToAngle(PI / 2);
            driveToPoint(.55, 1.45, PI / 2);
            break;
    }
    incrementState();
    return; 
}

void deployArm(){
    armServo.write(56);
    delay(500);
    clawServo.write(0);
    incrementState();
}

void deployPinion(){

}

void deployClaw(){
    clawServo.write(100);
    incrementState();
}

void openClawWide(){
    clawServo.write(100);
    incrementState();
}

void pickUpPuck(){
    clawServo.write(100);
    incrementState();
}

void retractPinion(){

}

void retractArm(){
    armServo.write(90);
    delay(500);
    incrementState();
}

bool detectBField(){
    int numReadings = 250;
    float total = 0;
    for(int i = 0; i < numReadings; i++){
        total += analogRead(A0);
        delay(2);
    }
    total = ((total / numReadings) / 2.0351) -100;
    Serial.print("Measured B-Field: ");
    Serial.print(total);
    Serial.println("%");
    if(total < -10 || total > 10){
        Enes100.mission(MAGNETISM, MAGNETIC);
        Serial.println("B-Field True");
        return true;
    } 
    else{
        Enes100.mission(MAGNETISM, NOT_MAGNETIC);
        Serial.println("B-Field False");
        return false;
    }
}

void pathfindToLog(){
    float targetX = .5;
    float targetY = 4;
    float targetTheta = 0;
    if(!driveToPointObstructed(targetX, targetY, targetTheta)){
        tankDrive(-25);
        delay(250);
        stop();
        pathfindToLog();
    }
    incrementState();
}

void driveOverLog(){
    turnToAngle(0);
    float y = Enes100.getY();
    int cyclesElapsed = 0;
    while(y < 4.2){
        tankDrive(255);
        delay(10);
        cyclesElapsed++;
        if(cyclesElapsed > 75){
            driveToPoint(.5, 4, 0);
            break;
        }    
    }
    if(y < 4.2){
        driveOverLog();
    }
}

void portConfiguration(){
    pinMode(2, INPUT_PULLUP); 
    pinMode(A0, INPUT);
    pinMode(leftIN1, OUTPUT);
    pinMode(leftIN2, OUTPUT);
    pinMode(leftEN, OUTPUT);
    pinMode(rightIN1, OUTPUT);
    pinMode(rightIN2, OUTPUT);
    pinMode(rightEN, OUTPUT);
    armServo.attach(5);
    clawServo.attach(6);
    armServo.write(25);
    clawServo.write(90);
}

bool squareWaveRead() {
    int numReadings = 50;
    int numFailures = 0;
    double total = 0;
    unsigned long timeHigh;
    unsigned long timeLow;
    float dutyCycle;
    for(int i = 0; i < numReadings; i++){
        timeHigh = pulseIn(2, HIGH);
        timeLow = pulseIn(2, LOW);
        unsigned long totalPeriod = timeHigh + timeLow;
        if (totalPeriod > 0) {
            dutyCycle = ((float)timeHigh / totalPeriod) * 100.0;
        } else {
            Serial.println("Square Wave Reading FAILED!");
            dutyCycle = 0.0;
            if(++numFailures > 3){
                Serial.println("4 Failed Squarewave Readings: Data Discarded and NOT Reported to ENES Service!");
                return false;
            }
        }
        total = total + dutyCycle;
    }
    Serial.print("Measured Duty Cycle is ");
    total = total / numReadings;
    Serial.print(total);
    total = 10 * round(total / 10);
    if(total == 10 || total == 30 || total == 50 || total == 70 || total == 90){
        Enes100.mission(CYCLE, total);
        Serial.print("  Reported Duty Cycle is ");
        Serial.print(total);
        Serial.println("");
        return true;
    }
    return false;
    Serial.println("Reading Was NOT WITHIN Reportable Range!");
}

void serialCommunication(){
    if (Serial.available() > 0){
        String receivedMessage = Serial.readStringUntil('\n');
        receivedMessage.trim();
        if(receivedMessage == "Claw"){
            clawServo.write(90+30);
            delay(5000);
            clawServo.write(90+50);
        }

        if(receivedMessage == "B-Field"){
           detectBField();
        }

        if(receivedMessage == "pid"){
            turnToAngle(0);
        }

        if(receivedMessage == "Motor Test"){
            tankDrive(100 * percentToPWM);
            Serial.println("Forward at 25%");
            delay(5000);
            tankDrive(-25 * percentToPWM);
            Serial.println("Backward at 25%");
            delay(5000);
            tankDrive(75 * percentToPWM);
            Serial.println("Forward at 75%");
            delay(5000);
            tankDrive(-75 * percentToPWM);
            Serial.println("Backward at 75%");
            delay(5000);
            tankTurn(25 * percentToPWM);
            Serial.println("Turn Right at 25%");
            delay(5000);
            tankTurn(-25 * percentToPWM);
            Serial.println("Turn Left at 25%");
            delay(5000);
            tankTurn(75 * percentToPWM);
            Serial.println("Turn Right at 75%");
            delay(5000);
            tankTurn(-75 * percentToPWM);
            Serial.println("Turn Left at 75%");
            delay(5000);
            stop();
        }

        if(receivedMessage == "Sensor Test"){
            Serial.println("Sensor Testing!");
            Serial.println(ultrasonicDistance1());
            Serial.println(ultrasonicDistance2());
            Serial.println(unifiedUltrasonicClosest());
        }

        if(receivedMessage == "Square Wave"){
            squareWaveRead();
        }
        if(receivedMessage == "Wifi Data"){
            Serial.println("Fetching Data from ENES Service...");
            Serial.print("x = ");
            Serial.print(Enes100.getX());
            Serial.print(", y = ");
            Serial.print(Enes100.getY());
            Serial.print(", Theta = ");
            Serial.println(Enes100.getTheta());
        }
    }
}

void stop(){
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, LOW);
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, LOW);
}

void tankDrive(int pwm){
    if(pwm<0){
        digitalWrite(leftIN1, HIGH);
        digitalWrite(leftIN2, LOW);
        digitalWrite(rightIN1, HIGH);
        digitalWrite(rightIN2, LOW);
        analogWrite(leftEN, pwm);
        analogWrite(rightEN, pwm);
    }
    else{
        Serial.print("Motors Forward");
        digitalWrite(leftIN1, LOW);
        digitalWrite(leftIN2, HIGH);
        digitalWrite(rightIN1, LOW);
        digitalWrite(rightIN2, HIGH);
        analogWrite(leftEN, pwm);
        analogWrite(rightEN, pwm);
    }
}

void tankTurn(int pwm){// Positive Number to the right, Negative to the left
    if(pwm<0){
        pwm = abs(pwm);
        digitalWrite(leftIN1, LOW);
        digitalWrite(leftIN2, HIGH);
        digitalWrite(rightIN1, HIGH);
        digitalWrite(rightIN2, LOW);
        analogWrite(leftEN, pwm);
        analogWrite(rightEN, pwm);
     }
     else{
        digitalWrite(leftIN1, HIGH);
        digitalWrite(leftIN2, LOW);
        digitalWrite(rightIN1, LOW);
        digitalWrite(rightIN2, HIGH);
        analogWrite(leftEN, pwm);
        analogWrite(rightEN, pwm);
     }
}

void turnToAngle(float angle){//-PI -> PI
    Setpoint = angle;
    float theta = Enes100.getTheta();
    float error = theta-angle;
    float correctedError = 0;
    Input = theta;
    angleController.SetOutputLimits(0.0, 1.0);
    angleController.SetOutputLimits(-1.0, 0.0);
    angleController.SetOutputLimits(-255, 255);
    delay(10);
    angleController.SetMode(AUTOMATIC);
    while(abs(theta-angle)>(.05)){
        theta = Enes100.getTheta();
        if(theta != -1){
            Input = theta;
            error = theta-angle;
            if(error > PI){
                correctedError = error - (2 * PI);
            }
            if(error < -PI){
                correctedError = error + (2 * PI);
            }
            if(error>(2*PI)){
                delay(30);
                continue;
            }
            if(angleController.Compute()){
                tankTurn(Output);
            }
            Serial.print("Error: ");
            Serial.print(error);
            Serial.print("    Corrected Error: ");
            Serial.print(correctedError);
            Serial.print("    Field Reported Value");
            Serial.print(theta);
            Serial.print("    Output: ");
            Serial.println(Output);
        }
        delay(10);      
     Enes100.println("Angle Reached");
     stop();
    }
}

void driveToPoint(double x, double y, double theta){
    while(Enes100.getX() == -1){
        delay(5);
    }
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    float deltax = x - initx;
    float deltay = y - inity;
    float targetAngle = atan2(deltay, deltax);
    turnToAngle(targetAngle);
    angleController.SetTunings(200, 300, 5);  //TUNE
    while (abs(initx-x)>.05 || abs(inity-y)>.05 || initx == -1 || inity == -1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        deltax = x - initx;
        deltay = y - inity;
        targetAngle = atan2(deltay, deltax);
        while(initx == -1 || inity == -1){
            initx = Enes100.getX();
            inity = Enes100.getY();
            delay(10);
            Serial.print("Stuck");
            stop();
        }
        turnToAngle(targetAngle);
        Serial.print("Driving   deltax: ");
        Serial.print(abs(initx - x));
        Serial.print("deltay: ");
        Serial.println(abs(inity - y));
        tankDrive(50);
        delay(200);
    }
    angleController.SetTunings(200, 75, 0);
    turnToAngle(theta);
}

bool driveToPointObstructed(double x, double y, double theta){
    while(Enes100.getX() == -1){
        delay(5);
    }
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    float deltax = x - initx;
    float deltay = y - inity;
    float targetAngle = atan2(deltay, deltax);
    turnToAngle(targetAngle);
    angleController.SetTunings(200, 300, 5);   //TUNE
    float angleOffset = 0;
    bool obstacleFound = false;
    int cyclesSinceObstacleFound;
    while (abs(initx-x)>.04 || abs(inity-y)>.04 || initx == -1 || inity == -1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        deltax = x - initx;
        deltay = y - inity;
        targetAngle = atan2(deltay, deltax);
        while(initx == -1 || inity == -1){
            initx = Enes100.getX();
            inity = Enes100.getY();
            delay(10);
            Serial.print("Stuck");
            stop();
        }
        
        if(unifiedUltrasonicClosest() < 4){
            if(inity >= 1){
                angleOffset = -(PI/32);
            }
            while(unifiedUltrasonicClosest() < 6){
                turnToAngle(targetAngle + angleOffset);
                if (angleOffset >= 0){
                    angleOffset += (PI/32);
                }
                else{
                    angleOffset -= (PI/32);
                }
                if (angleOffset > (PI/3)){
                    angleOffset = -(PI/32);
                }
                else if (angleOffset < - (PI/3)){
                    Serial.print("Passable Angle Finding FAILED");
                    return false;
                }
            }
            obstacleFound = true;
            cyclesSinceObstacleFound = 0;
            Serial.print("Obstacle Found!!");
        }
        else if(obstacleFound){
            cyclesSinceObstacleFound++;
            if (cyclesSinceObstacleFound > 10){
                angleOffset = 0;
                cyclesSinceObstacleFound = 0;
                obstacleFound = false;
            }
            else{
                turnToAngle(targetAngle + angleOffset + ((angleOffset/angleOffset)*(PI/6)));
            }
        }
        else{
            turnToAngle(targetAngle);
        }
        Serial.print("Driving   deltax: ");
        Serial.print(abs(initx - x));
        Serial.print("deltay: ");
        Serial.println(abs(inity - y));
        tankDrive(50);
        delay(200);
    }
    angleController.SetTunings(Kp, Ki, Kd);
    turnToAngle(theta);
    return true;
}