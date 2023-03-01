#include "StepperESP.h"
#include "Utiles.h"
void setup() {
  Serial.begin(115200);
  delay(2000);
  // put your setup code here, to run once:
  InitMotors();
  InitTimer1();
  
  //gotoPos(1000, 10);
  
  addPos(500, 500, 100, 98, 0);
  addPos(0, 0, 100, 98, 1); 
  //addPos(800, 0, 500, 70, 2);
  //addPos(0, 0, 500, 90, 3);
}

void loop() {
  runBlock();
}
