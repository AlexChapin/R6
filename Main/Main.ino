#include "Functions.h"


void setup() {
bool runAuton = false;
bool connecttoENES = false;


runProgram(runAuton);
initalizeProgram(connecttoENES);
portConfiguration();
}

void loop() {
  serialCommunication();
  stop();
  delay(10);
  if(getState() > 0){
    if(getState() == 0){
      pathfindToObjective();
    }
    else if(getState() == 1){

    }
    else if(getState() == 2){

    }
    else if(getState() == 3){

    }
    else if(getState() == 4){

    }
    else if(getState() == 5){

    }
    else if(getState() == 6){

    }
    else if(getState() == 7){

    }
  }
}





