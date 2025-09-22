#include "Functions.h"


void setup() {
  
bool connecttoENES = false;
initalizeProgram(connecttoENES);

}

void loop() {
serialCommunication();
stop();
delay(500);
}






