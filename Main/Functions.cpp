/*
    Functions.cpp (Primary)
*/
#include "src\L298N\L298N.h"
#include "src\NewPing\NewPing.h"
#include "src\Enes100\Enes100.h"
#include "src\PID\PID_v1.h"
#include "Servo.h"
#include "Functions.h"
int state; 

//Rotational PID
double Setpoint, Input, Output;
double Kp=20, Ki=5, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Ultrasonic Configs
#define TRIGGER_PIN1 8
#define ECHO_PIN1 9
#define MAX_DISTANCE 200
#define TRIGGER_PIN2 10
#define ECHO_PIN2 11
NewPing ultrasonic1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing ultrasonic2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

//Servo Configs
Servo armServo;
Servo clawServo;

#define leftIN1 22
#define leftIN2 23
#define leftEN 2
#define rightIN1 24
#define rightIN2 25
#define rightEN 3
#define pinionIN1 26
#define pinionIN2 27
#define pinionEN 4
L298N leftMotor(leftIN1, leftIN2, leftEN);
L298N rightMotor(rightIN1, rightIN2, rightEN);
L298N pinionMotor(pinionIN1, pinionIN2, pinionEN);

double percentToPWM = 255/100;


void runProgram(bool value){
    if(value){
        state = 0;
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
        int wifiModuleTX = 52;
        int wifiModuleRX = 50;
        int roomNumber = 1116;
        int markerId = 0;
        Enes100.begin("R6", DATA, markerId, roomNumber, wifiModuleTX, wifiModuleRX);
        if (Enes100.state() == 0x01){
            Enes100.println("WiFi Connected!");
        }
        else{
            Enes100.println("Wifi Failure!!!!!");
        }
        Enes100.println("Connected to Vision? " + Enes100.isConnected());
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
            driveToPoint(0, 0, PI / 2);
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
    armServo.write(0);
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
        Serial.println("True");
        return true;
    } 
    else{
        Enes100.mission(MAGNETISM, NOT_MAGNETIC);
        Serial.println("False");
        return false;
    }
}

void portConfiguration(){
    pinMode(2, INPUT_PULLUP); 
    pinMode(A0, INPUT);
    armServo.attach(5);
    clawServo.attach(6);
    armServo.write(25);
    clawServo.write(25);
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

        if(receivedMessage == "B-Field"){
           detectBField();
        }

        if(receivedMessage == "Motor Test"){
            tankDrive(50 * percentToPWM);
            Serial.println("Forward at 50%");
            delay(5000);
            tankDrive(-50 * percentToPWM);
            Serial.println("Backward at 50%");
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
    rightMotor.stop();
    leftMotor.stop();
}

void tankDrive(int pwm){
    if(pwm<0){
        pwm = abs(pwm);
        rightMotor.backward();
        leftMotor.backward();
    }
    else{
        rightMotor.forward();
        leftMotor.forward();
    }
    leftMotor.setSpeed(pwm);
    rightMotor.setSpeed(pwm);
}

void tankTurn(int pwm){// Positive Number to the right, Negative to the left
    if(pwm<0){
        pwm = abs(pwm);
        rightMotor.forward();
        leftMotor.backward();
    }
    else{
        rightMotor.backward();
        leftMotor.forward();
    }
    leftMotor.setSpeed(pwm);
    rightMotor.setSpeed(pwm);
}

void turnToAngle(float angle){//-PI -> PI
    Setpoint = angle;
    float theta = Enes100.getTheta();
    float error = theta-angle;
    Input = theta;
    myPID.SetOutputLimits(0.0, 1.0);
    myPID.SetOutputLimits(-1.0, 0.0);
    myPID.SetOutputLimits(-255, 255);
    delay(10);
    myPID.SetMode(AUTOMATIC);
    while(abs(theta-angle)>(.05)){
        theta = Enes100.getTheta();
        if(theta != -1){
            Input = theta;
            error = theta-angle;
            if (error > PI){
                error -= 2 * PI;
            }
            else if (error < PI){
                error += 2 * PI;
            }
            Serial.print("Error:");
            Serial.print(error);
            Serial.print("  Setpoint:");
            Serial.print(Setpoint);
            Serial.print("  Output:");
            Serial.println(Output);
            myPID.Compute();
            tankTurn(Output);
        }
        delay(10);      
     Enes100.println("Angle Reached");
     stop();
    }
}

void driveToPoint(double x, double y, double theta){
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    
    while (abs(initx-x)>.1||abs(inity-y)>.1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        float deltax = x - initx;
        float deltay = y - inity;
        float targetangle = atan2(deltay, deltax);
        turnToAngle(targetangle);
        tankDrive(50);
        delay(50);
    }
    turnToAngle(theta);
}

void driveToPointObstructed(double x, double y, double theta){
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    
    while (abs(initx-x)>.1||abs(inity-y)>.1){
        if(ultrasonicDistance1() == -1 || ultrasonicDistance1() == -1){
            stop();
        }
        else if(ultrasonicDistance1() < 4 || ultrasonicDistance2() < 4){
            
        }
        initx = Enes100.getX();
        inity = Enes100.getY();
        float deltax = x - initx;
        float deltay = y - inity;
        float targetangle = atan2(deltay, deltax);
        turnToAngle(targetangle);
        tankDrive(50);
        delay(50);  
    }
    turnToAngle(theta);
}

