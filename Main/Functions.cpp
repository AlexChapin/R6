#include "src\NewPing\NewPing.h"
#include "src\Enes100\Enes100.h"
#include "src\PID\PID_v1.h"
#include "Functions.h"
int state; 
double lastx;
double lasty;
double lastTheta;

#define TRIGGER_PIN1 1
#define ECHO_PIN1 1
#define MAX_DISTANCE 200
#define TRIGGER_PIN2 1
#define ECHO_PIN2 1
NewPing ultrasonic1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing ultrasonic2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

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
        int wifiModuleTX = 50;
        int wifiModuleRX = 51;
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
    unsigned int uS = ultrasonic1.ping();
    if (uS) {
    return ultrasonic1.convert_cm(uS)
  }
  else return NULL
}

float ultrasonicDistance2(){
    unsigned int uS = ultrasonic2.ping();
    if (uS) {
    return ultrasonic2.convert_cm(uS)
  }
  else return NULL
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
    
        case false:
            turnToAngle(PI / 2);
            driveToPoint(0, 0, PI / 2);
    }
    state++;
    return; 
}

void deployArm(){

}

void deployPinion(){

}

void deployClaw(){

}

void openClawWide(){

}

void pickUpPuck(){

}

void retractPinion(){

}

void retractArm(){

}

void detectBField(){

}

void portConfiguration(){
    pinMode(2, INPUT_PULLUP); 
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

bool validateXYTheta(float x, float y, float theta){
    bool result = true;
    if ((abs(lastx - x) > .1) || (abs(lasty - y) > .1) || (abs(lastTheta - theta) > (PI/8))){
        result = false;
    }
    lastx = x;
    lasty = y;
    lastTheta = theta;
    return result;
}

bool validateTheta(float theta){
    bool result = true;
    if (abs(lastTheta - theta) > (PI/8)){
        result = false;
    }
    lastTheta = theta;
    return result;
}

void serialCommunication(){
    if (Serial.available() > 0){
        String receivedMessage = Serial.readStringUntil('\n');
        receivedMessage.trim();

        if(receivedMessage == "Motor Test"){
            tankDrive(25, true);
            Serial.println("Forward at 25%");
            delay(500);
            tankDrive(25, false);
            Serial.println("Backward at 25%");
            delay(500);
            tankDrive(75, true);
            Serial.println("Forward at 75%");
            delay(500);
            tankDrive(75, false);
            Serial.println("Backward at 75%");
            delay(500);
            tankTurn(25, true);
            Serial.println("Turn Right at 25%");
            delay(500);
            tankTurn(25, false);
            Serial.println("Turn Left at 25%");
            delay(500);
            tankTurn(75, true);
            Serial.println("Turn Right at 75%");
            delay(500);
            tankTurn(75, false);
            Serial.println("Turn Left at 75%");
            delay(500);
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
    
}

void tankDrive(int percent, bool directioninv){ //Forward is True //Backward is False
    double drivingSpeed = 255*percent/100;
    if (!directioninv){
        drivingSpeed = drivingSpeed * -1;
    }

    
}

void tankTurn(int percent, bool directioninv){ //Right is True //Left is False
    double drivingSpeed = 255*percent/100;
    if (!directioninv){
        drivingSpeed = drivingSpeed * -1;
    }

    
}

void turnToAngle(float angle){//-PI -> PI
    double Kp=2, Ki=0, Kd=0;
    double Input, Output, Setpoint = angle;
    PID angleController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    float theta = Enes100.getTheta();
    //float speedreduce = 1;
    while(abs(theta-angle)>(0.5*PI/180)){
        if(validateTheta(Enes100.getTheta())){
            Input = Enes100.getTheta();
            if (angleController.Compute()){
                tankTurn(Output, true);
            }
        }
        else{
            stop();
        }
        
        
        // if ((abs(theta - angle) < (7.5 * PI / 180))){
        //     speedreduce = .15;
        // }
        // else{
        //     speedreduce = 1;
        // }
        
        // if(theta > angle && abs(theta-angle)<=PI){
        //     tankTurn(25 * speedreduce,false);
        // }
        // else if(theta < angle && abs(theta-angle)<=PI){
        //     tankTurn(25 * speedreduce,true);
        // }
        // else if(theta > angle && abs(theta-angle)>PI){
            
        //     if(theta > (PI * 172.5 / 180) || (theta < (PI * -172.5 / 180))){
        //      speedreduce = .10;
        //      Enes100.print("SpeedRed");
        //     }
        //     tankTurn(25 * speedreduce,true);
        // }
        // else if(theta < angle && abs(theta-angle)>PI){
            
        //     if(theta > (PI * 172.5 / 180) || (theta < (PI * -172.5 / 180))){
        //      speedreduce = .10;
        //     }
        //      tankTurn(25 * speedreduce,false);
        // }
        // delay(1);
        // Enes100.println(abs(theta-angle));
    }
     Enes100.println("Angle Reached");
     stop();
}

void driveToPoint(double x, double y, double theta){
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    
    while (abs(initx-x)>.1||abs(inity-y)>.1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        if(validateXYTheta(Enes100.getX(), Enes100.getY(), Enes100.getTheta())){
            float deltax = x - initx;
            float deltay = y - inity;
            float targetangle = atan2(deltay, deltax);
            turnToAngle(targetangle);
            tankDrive(50,true);
            delay(50);
        }
        else{
            stop();
        }     
    }
    turnToAngle(theta);
}

void driveToPointObstructed(double x, double y, double theta){
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    
    while (abs(initx-x)>.1||abs(inity-y)>.1){

        initx = Enes100.getX();
        inity = Enes100.getY();
        if(validateXYTheta(Enes100.getX(), Enes100.getY(), Enes100.getTheta())){
            float deltax = x - initx;
            float deltay = y - inity;
            float targetangle = atan2(deltay, deltax);
            turnToAngle(targetangle);
            tankDrive(50,true);
            delay(50);
        }
        else{
            stop();
        }     
    }
    turnToAngle(theta);
}

