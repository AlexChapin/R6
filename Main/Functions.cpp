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
double Kp=15, Ki=30, Kd=60; //TUNE
PID angleController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double prevX;
double prevY;
double prevTargetAngle;

//Ultrasonic Configs
#define TRIGGER_PIN1 8
#define ECHO_PIN1 9
#define MAX_DISTANCE 350
#define TRIGGER_PIN2 10
#define ECHO_PIN2 11
NewPing ultrasonic1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing ultrasonic2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

//Servo Configs
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

void connectCheck(){
    if(Enes100.state() != 0x01){
        Serial.print("Attempting To Reconnect!");
        stop();
        Enes100.begin("R6", DATA, 200, 1116, 53, 51);
    }
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
            Serial.println("Wifi Failure!!");
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
  else return 1000;
}

float ultrasonicDistance2(){
    unsigned int uS = ultrasonic2.ping(MAX_DISTANCE);
    if (uS != 0) {
    return ultrasonic2.convert_in(uS);
  }
  else return 1000;
}

float unifiedUltrasonicClosest(){
    float result = 0;
    if(ultrasonicDistance1() > ultrasonicDistance2()){
        return ultrasonicDistance2();
    }
    else{
        return ultrasonicDistance1();
    }
}

void pathfindToObjective(){
    Enes100.println("Path Finding To Objective");
    float x = Enes100.getX();
    float y = Enes100.getY();
    float theta = Enes100.getTheta();
    Serial.print(x);
    Serial.println(y);
    bool top = y > 1;

    switch (top) {
        case true:
            turnToAngle(-PI / 2);
            driveToPoint(0.26, 0.84, -PI / 2);
            delay(100);
            turnToAngle(-PI / 2);
            delay(100);
            turnToAngle(-PI / 2);
            delay(100);
            turnToAngle(-PI / 2);
            while(y > .73){
                y = Enes100.getY();
                tankDrive(80);
                delay(10);
                if( y > .74){
                    theta = Enes100.getTheta();
                    if(abs(theta + PI/2) > PI/16){
                        y = Enes100.getY();
                        tankTurn(90 * (theta + PI/2) / (theta + PI/2));
                        delay(25);
                        tankDrive(80);
                        delay(10);
                    }
                }
            }
            delay(500);
            if(unifiedUltrasonicClosest() > 2){
                tankDrive(-90);
                delay(1500);
                pathfindToObjective();
            }
            break;
    
        case false:
            turnToAngle(PI / 2);
            driveToPoint(0.33, 1.19, PI / 2);
            delay(100);
            turnToAngle(PI / 2);
            delay(100);
            turnToAngle(PI / 2);
            delay(100);
            turnToAngle(PI / 2);
            while(y < 1.28){
                y = Enes100.getY();
                tankDrive(80);
                delay(10);
            }
            if( y < 1.27){
                    theta = Enes100.getTheta();
                    if(abs(theta - PI/2) > PI/16){
                        y = Enes100.getY();
                        tankTurn(90 * (theta - PI/2) / (theta - PI/2));
                        delay(25);
                        tankDrive(80);
                        delay(10);
                    }
                }
            delay(500);
            if(unifiedUltrasonicClosest() > 2){
                tankDrive(-90);
                delay(1500);
                pathfindToObjective();
            }
            break;
    }
    return; 
}

void fullScore(){
    clawServo.write(24);
    pinionDrive(-255);
    delay(2500);
    clawServo.write(-20);
    pinionDrive(0);
    squareWaveRead();
    if(!squareWaveRead()){
        clawServo.write(24);
        pinionDrive(-255);
        delay(100);
        pinionDrive(0);
        clawServo.write(-20);
        squareWaveRead();
    }
    else{
        clawServo.write(15);
        pinionDrive(-255);
        delay(100); 
    }
    clawServo.write(4);
    pinionDrive(-255);
    delay(1000);
    pinionDrive(0);
    clawServo.write(35);
    detectBField();
    delay(500);
    pinionDrive(255);
    delay(4500);
    pinionDrive(0);
    clawServo.write(24);
}

bool detectBField(){
    int numReadings = 250;
    float total = 0;
    for(int i = 0; i < numReadings; i++){
        total += analogRead(A0);
        delay(2);
    }
    total = ((total / numReadings) / 2.0351) - 101.2;
    Serial.print("Measured B-Field: ");
    Serial.print(total);
    Serial.println("%");
    if(total < -30 || total > 30){
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

void driveOverLog(){
    float x = Enes100.getX();
    int cyclesElapsed = 0;
    while(x < 3.55){
        x = Enes100.getX();
        tankDrive(255);
        delay(200);
    }
    stop();
}

void portConfiguration(){
    pinMode(12, INPUT_PULLUP); 
    pinMode(A0, INPUT);
    pinMode(leftIN1, OUTPUT);
    pinMode(leftIN2, OUTPUT);
    pinMode(leftEN, OUTPUT);
    pinMode(rightIN1, OUTPUT);
    pinMode(rightIN2, OUTPUT);
    pinMode(rightEN, OUTPUT);
    clawServo.attach(6);
    clawServo.write(15);

}

bool squareWaveRead() {
    int numReadings = 50;
    int numFailures = 0;
    double total = 0;
    unsigned long timeHigh;
    unsigned long timeLow;
    float dutyCycle;
    for(int i = 0; i < numReadings; i++){
        timeHigh = pulseIn(12, HIGH);
        Serial.println(timeHigh);
        Serial.println(timeLow);
        timeLow = pulseIn(12, LOW);
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

        if(receivedMessage == "init"){
            state = 1;
        }

        if(receivedMessage == "ObstacleNav"){
            obstacleNavigate();
        }

        if(receivedMessage == "Test"){
            pathfindToObjective();
            stop();
            fullScore();
        }

        if(receivedMessage == "Claw"){
            clawServo.write(20);
            delay(5000);
            clawServo.write(9);
        }

        if(receivedMessage == "pinion"){
            Serial.print("Pinion");
            pinionDrive(-150);
            delay(3000);
            pinionDrive(0);
        }

        if(receivedMessage == "B-Field"){
           detectBField();
        }

        if(receivedMessage == "score"){
            fullScore();
        }

        if(receivedMessage == "pid"){
            turnToAngle(0);
        }

        if(receivedMessage == "log"){
            driveOverLog();
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

void pinionDrive(int pwm){
    if(pwm<0){
        pwm = abs(pwm);
        digitalWrite(pinionIN1, HIGH);
        digitalWrite(pinionIN2, LOW);
        analogWrite(pinionEN, pwm);
    }
    else{
        digitalWrite(pinionIN1, LOW);
        digitalWrite(pinionIN2, HIGH);
        analogWrite(pinionEN, pwm);
    }
}

void tankDriveWallRun(int pwm){
    if(pwm>0){
        digitalWrite(leftIN1, LOW);
        digitalWrite(leftIN2, HIGH);
        digitalWrite(rightIN1, LOW);
        digitalWrite(rightIN2, HIGH);
        analogWrite(leftEN, pwm);
        analogWrite(rightEN, pwm * 0.5);
    }
}

void tankDrive(int pwm){
    if(pwm<0){
        pwm = abs(pwm);
        digitalWrite(leftIN1, HIGH);
        digitalWrite(leftIN2, LOW);
        digitalWrite(rightIN1, HIGH);
        digitalWrite(rightIN2, LOW);
        analogWrite(leftEN, pwm);
        analogWrite(rightEN, pwm);
    }
    else{
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
    float preverror = theta-angle;
    int timeStuck = 0;
    Input = theta;
    angleController.SetMode(MANUAL);
    Output = 0;
    delay(10);
    angleController.SetOutputLimits(-255, 255);
    angleController.SetMode(AUTOMATIC);
    while(abs(theta-angle)>(.1)){
        // connectCheck();
        prevX = Enes100.getX();
        prevY = Enes100.getY();
        theta = Enes100.getTheta();
        if(theta != -1){
            Input = theta;
            error = theta-angle;
            if(abs(error-preverror) > .5){
                delay(50);
                Serial.println("Reading Inaccurate");
                Serial.print("Error Was:");
                Serial.println(error);
                timeStuck++;
                if(timeStuck > 20){
                    timeStuck = 0;
                    preverror = error;
                }
                continue;
            }
            if(angleController.Compute()){
                tankTurn(Output);
            }
            if((abs(Output) > 250 && error < (PI / 2)) || (error > 0 && preverror < 0) || (preverror < 0 && error > 0)){
                angleController.SetMode(MANUAL);
                Output = 0;
                delay(10);
                angleController.SetOutputLimits(-255, 255);
                angleController.SetMode(AUTOMATIC);
            }
            Serial.print("Error: ");
            Serial.print(error);
            Serial.print("    Field Reported Value");
            Serial.print(theta);
            Serial.print("    Output: ");
            Serial.println(Output);
            preverror = error;
        }
        delay(100);
     stop();
    }
}

bool driveToPoint(double x, double y, double theta){
    while(Enes100.getX() == -1){
        delay(5);
    }
    int timeStuck = 0;
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    float deltax = x - initx;
    float deltay = y - inity;
    float targetAngle = atan2(deltay, deltax);
    prevTargetAngle = targetAngle;
    turnToAngle(targetAngle);
    angleController.SetTunings(15, 60, 60);
    while (abs(initx-x)>.075 || abs(inity-y)>.075 || initx == -1 || inity == -1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        deltax = x - initx;
        deltay = y - inity;
        targetAngle = atan2(deltay, deltax);
        if(abs(targetAngle - prevTargetAngle) > (PI / 4)){
            stop();
            delay(50);
            timeStuck++;
            targetAngle = atan2(deltay, deltax);
            Serial.print("Atan Diff");
            if(timeStuck > 15){
                return false;
            }
            continue;
        }
        while(initx == -1 || inity == -1){
            initx = Enes100.getX();
            inity = Enes100.getY();
            delay(10);
            Serial.print("Stuck");
            stop();
        }
        prevTargetAngle = targetAngle;
        turnToAngle(targetAngle);
        Serial.print("Driving   deltax: ");
        Serial.print(abs(initx - x));
        Serial.print("   deltay: ");
        Serial.println(abs(inity - y));
        tankDrive(65);
        delay(100);
        prevX = initx;
        prevY = inity;
        timeStuck = 0;
    }
    turnToAngle(theta);
    return true;
}

bool obstacleNavigate(){
    float x = Enes100.getX();
    float y = Enes100.getY();
    float theta = Enes100.getTheta();
    while(x == -1){
        x = Enes100.getX();
        y = Enes100.getY();
        theta = Enes100.getTheta();
    }
    bool top = y > 1;
    tankDrive(-150);
    delay(250);
    if(top){
        delay(200);
    }
    turnToAngle(0);
    delay(100);
    turnToAngle(0);
    tankDrive(175);
    delay(100);
    stop();
    turnToAngle(-PI/2);
    delay(100);
    turnToAngle(-PI/2);
    delay(100);
    turnToAngle(-PI/2);
    while(y > .1 || y == -1){
        y = Enes100.getY();
        tankDrive(90);
        delay(50);
        y = Enes100.getY();
        turnToAngle(-PI/2);
    }
    turnToAngle(-0.3);
    delay(100);
    turnToAngle(-0.3);
    delay(100);
    turnToAngle(-0.3);
    while(x < 2.55){
        x = Enes100.getX();
        tankDriveWallRun(175);
    }
    stop();
    driveOverLog();
}

void victory(){
    for(int i = 0; i < 100; i++){
        Enes100.println("TEAM R6 HAS SUCESSFULLY NAVIGATED TO THE END!!!");
        clawServo.write(15);
        delay(500);
        clawServo.write(0);
        delay(500);
    }
}