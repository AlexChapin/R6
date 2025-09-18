#include "Enes100.h"


void setup() {

bool connecttoENES = false;
  if (connecttoENES){
    int wifiModuleTX = 0;
    int wifiModuleRX = 0;
    int roomNumber = 1116;
    int markerId = 0;
    Enes100.begin("R6", DATA, markerId, roomNumber, wifiModuleTX, wifiModuleRX);
    if (Enes100.state()==0x01){
      Enes100.println("WiFi Connected!");
    }
    else{
      Enes100.println("Wifi Failure!!!!!");
    }
    Enes100.println("Connected to Vision? " + Enes100.isConnected());
}
else{
  Serial.begin(9600);
  Serial.println("Serial Monitor Connected!");
}







}

void loop() {
  // put your main code here, to run repeatedly:

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

void turnToAngle(int angle){ //180->-180
    float theta = Enes100.getTheta();
    float speedreduce = 1;
    while(abs(theta-angle)>(0.5*PI/180)){
        theta = Enes100.getTheta();
        if ((abs(theta-angle)<(7.5*PI/180))){
            speedreduce = .15;
        }
        else{
            speedreduce = 1;
        }
        
        if(theta > angle){
            tankTurn(25 * speedreduce,false);
        }
        else if(theta < angle){
            tankTurn(25 * speedreduce,true);
        }
        delay(1);
        Enes100.println(abs(theta-angle));
    }
     Enes100.println("Angle Reached");
     stop();
}