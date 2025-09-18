#include "Enes100.h"


void setup() {

bool connecttoENES = False;
  if (connecttoENES){
    int wifiModuleTX = 0;
    int wifiModuleRX = 0;
    int roomNumber = 1116;
    int markerId = 0;
    Enes100.begin("R6", DATA, markerId, roomNumber, wifiModuleTX, wifiModuleRX);
    Enes100.print("Connected to Vision? " + Enes100.isConnected());
}




}

void loop() {
  // put your main code here, to run repeatedly:

}
