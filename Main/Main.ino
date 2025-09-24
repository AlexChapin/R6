#include "Functions.h"


void setup() {
  
bool connecttoENES = false;
initalizeProgram(connecttoENES);
portConfiguration();
}

void loop() {
serialCommunication();
stop();
}






