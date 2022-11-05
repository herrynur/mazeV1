#include "mazeV1.h"

int detection = 0;

void mazeInit() {
  //no init
}

void mazeLoop() {
    detection = resultObject(1000);
    Serial.println(detection);
    
    if (detection == 1)
      majutimer(50, 50, 1000);
    else if (detection == 2)
      majutimer(50, -50, 1000);
    else if (detection == 3)
      majutimer(-50, 50, 1000);
    else
      motorBerhenti();
}
