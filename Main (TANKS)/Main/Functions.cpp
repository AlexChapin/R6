/*
    Functions.cpp (Tanks)
*/
#include "src\Enes100\Enes100.h"
#include "src\PID\PID_v1.h"
#include "src\Tank\Tank.h"
#include "Functions.h"
int state; 
//Rotational PID Constants
double Setpoint, Input, Output;
double Kp=200, Ki=75, Kd=5;

PID angleController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


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
        int markerId = 16;
        Enes100.begin("R6", DATA, markerId, roomNumber, wifiModuleTX, wifiModuleRX);
        if (Enes100.state() == 0x01){
            Enes100.println("WiFi Connected!");
            Serial.println("WiFi Connected Condition == Good!");
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
    Tank.begin();
    //pinMode(2, INPUT_PULLUP); 
}

void driveTANK(){
    driveToPoint(1.55, 0.55, 0);
}

void turn90(){
    float targetAngle = Enes100.getTheta() - (PI / 2);
    if(targetAngle < -PI){
        targetAngle += 2 * PI;
    }
    turnToAngle(targetAngle);
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
            tankTurn(25);
            Serial.println("Turn Right at 25%");
            delay(500);
            tankTurn(25);
            Serial.println("Turn Left at 25%");
            delay(500);
            tankTurn(75);
            Serial.println("Turn Right at 75%");
            delay(500);
            tankTurn(75);
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
        if(receivedMessage == "Tank Drive"){
            driveTANK();
        }
        if(receivedMessage == "Turn"){
            turn90();
        }
        if(receivedMessage == "Example Run"){
            pathfindToObjective();
            driveToPoint(3,1,0);
        }
        
    }
}

void stop(){
    Tank.turnOffMotors();
}

void tankDrive(int percent, bool directioninv){ //Forward is True //Backward is False
    double drivingSpeed = 255*percent/100;
    if (!directioninv){
        drivingSpeed = drivingSpeed * -1;
    }
    Tank.setLeftMotorPWM(drivingSpeed);
    Tank.setRightMotorPWM(drivingSpeed);
}

void tankTurn(int PWM){ //Right is True //Left is False
    Tank.setLeftMotorPWM(-PWM);
    Tank.setRightMotorPWM(PWM);    
}

void turnToAngle(float angle){//-PI -> PI
    Setpoint = angle;
    float theta = Enes100.getTheta();
    float error = theta-angle;
    Input = theta;
    angleController.SetOutputLimits(0.0, 1.0);
    angleController.SetOutputLimits(-1.0, 0.0);
    angleController.SetOutputLimits(-200, 200);
    delay(10);
    angleController.SetMode(AUTOMATIC);
    while(abs(theta-angle)>(.05)){
        theta = Enes100.getTheta();
        if(theta != -1){
            Input = theta;
            error = theta-angle;
            if(angleController.Compute()){
                tankTurn(Output);
            }
            Serial.print("Error: ");
            Serial.print(error);
            Serial.print("   Output: ");
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
    angleController.SetTunings(200, 300, 5);
    while (abs(initx-x)>.05 || abs(inity-y)>.05 || initx == -1 || inity == -1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        while(initx == -1 || inity == -1){
            initx = Enes100.getX();
            inity = Enes100.getY();
            delay(10);
            Serial.print("Stuck");
            stop();
        }
        float deltax = x - initx;
        float deltay = y - inity;
        float targetangle = atan2(deltay, deltax);
        turnToAngle(targetangle);
        Serial.print("Driving   deltax: ");
        Serial.print(abs(initx - x));
        Serial.print("deltay: ");
        Serial.println(abs(inity - y));
        tankDrive(50,false);
        delay(200);
    }
    angleController.SetTunings(200, 75, 5);
    turnToAngle(theta);
}

bool driveToPointObstructed(double x, double y, double theta){
    while(Enes100.getX() == -1){
        delay(5);
    }
    float initx = Enes100.getX();
    float inity = Enes100.getY();
    float inittheta = Enes100.getTheta();
    angleController.SetTunings(200, 300, 5);
    float angleOffset = 0;
    bool obstacleFound = false;
    int cyclesSinceObstacleFound;
    while (abs(initx-x)>.04 || abs(inity-y)>.04 || initx == -1 || inity == -1){
        initx = Enes100.getX();
        inity = Enes100.getY();
        while(initx == -1 || inity == -1){
            initx = Enes100.getX();
            inity = Enes100.getY();
            delay(10);
            Serial.print("Stuck");
            stop();
        }
        float deltax = x - initx;
        float deltay = y - inity;
        float targetangle = atan2(deltay, deltax);
        if(Tank.readDistanceSensor(1) < .25){
            if(inity >= 1){
                angleOffset = -(PI/32);
            }
            while(Tank.readDistanceSensor(1) < .3){
                turnToAngle(targetAngle + angleOffset)
                if (angleOffset >= 0){
                    angleOffset += (PI/32);
                }
                else{
                    angleOffset -= (PI/32);
                }
                if angleOffset > (PI/3){
                    angleOffset = -(PI/32);
                }
                else if angleOffset < - (PI/3){
                    Serial.print("Passable Angle Finding FAILED");
                    return;
                }
            }
            
        }
        else if(obstacleFound){
            cyclesSinceObstacleFound++;
            if (cyclesSinceObstacleFound > 5){
                angleOffset = 0;
                cyclesSinceObstacleFound = 0;
                obstacleFound = false;
            }
            else{
                turnToAngle(targetAngle + angleOffset);
            }
        }
        else{
            turnToAngle(targetangle);
        }
        Serial.print("Driving   deltax: ");
        Serial.print(abs(initx - x));
        Serial.print("deltay: ");
        Serial.println(abs(inity - y));
        tankDrive(50,false);
        delay(200);
    }
    angleController.SetTunings(200, 75, 5);
    turnToAngle(theta);
}


