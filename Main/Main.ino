#include "Functions.h"


void setup() {
bool runAuton = false;
bool connectToENES = false;


runProgram(runAuton);
initalizeProgram(connectToENES);
portConfiguration();
}


void loop() {
  serialCommunication();
  stop();
  delay(10);
  if(getState() > 0){
    switch(getState()){
      case 1:
        pathfindToObjective();
        incrementState();
        break;
      case 2:
        fullScore();
        incrementState();
        break;
      case 3:
        obstacleNavigate();
        incrementState();
        break;
      case 4:
        victory();
        incrementState();
      }
    }
}