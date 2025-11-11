#include "Functions.h"


void setup() {
bool runAuton = false;
bool connecttoENES = true;


runProgram(runAuton);
initalizeProgram(connecttoENES);
portConfiguration();
}


void loop() {
  serialCommunication();
  stop();
  delay(10);
  if(getState() > 0){
    switch(getState()){
      case 1:
        incrementState();
        break;
      case 2:
        incrementState();
        break;
      case 3:
        incrementState();
        break;
      case 4:
        incrementState();
        break;
      case 5:
        incrementState();
        break;
      case 6:
        incrementState();
        break;
      case 7:
        incrementState();
        break;
      case 8:
        incrementState();
        break;
      case 9:
        incrementState();
        break;
      case 10:
        incrementState();
        break;
    }
  }
}





