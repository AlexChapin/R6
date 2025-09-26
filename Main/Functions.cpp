#include "src\Enes100\Enes100.h"
#include "src\PID\PID_v1.h"
#include "Functions.h"


void initalizeProgram(bool connecttoENES){ 
    Serial.begin(9600);
    Serial.println("");
    if (connecttoENES){
        int wifiModuleTX = 0;
        int wifiModuleRX = 0;
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
    Serial.println("Serial Monitor Connected!");
    Serial.println("WARNING! Do not attempt to run turnToAngle or driveToPoint without vision system connected!");
    }
}

void portConfiguration(){
    pinMode(2, INPUT);
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
                return false;
            }
        }
        total = total + dutyCycle;
    }
    String print = "Measured Duty Cycle is ";
    total = total / numReadings;
    print += total;
    total = 10 * round(total / 10);
    Enes100.mission(CYCLE, total);
    print += "  Reported Duty Cycle is ";
    print += total;
    Serial.println(print);
    return true;
}

void serialCommunication(){
    if (Serial.available() > 0){
        String receivedMessage = Serial.readStringUntil('\n');
        receivedMessage.trim();

        if(receivedMessage == "Motor Test"){
            Serial.println("Motor Testing!");
        }

        if(receivedMessage == "Sensor Test"){
            Serial.println("Sensor Testing!");
        }

        if(receivedMessage == "Square Wave"){
            squareWaveRead();
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
    float speedreduce = 1;
    while(abs(theta-angle)>(0.5*PI/180)){
        Input = Enes100.getTheta();
        if (angleController.Compute()){
            tankTurn(Output, true);
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

void drivetoPoint(double x, double y, double theta){
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
        tankDrive(50,true);
        delay(50);
        
    }
    turnToAngle(theta);
}

