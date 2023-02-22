#include "BPV.h"
#include "Utiles.h"
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  InitBPV();
  InitMotors();
  //gotoPos(1000, 10);
  
  addPos(0, 200, 500, 30, 0);
  addPos(800, 200, 500, 50, 1);
  addPos(800, 0, 500, 70, 2);
  addPos(0, 0, 500, 90, 3);
}

void loop() {
  runBlock();
}
