#ifndef _BPV_h
#define _BPV_h

#include "Arduino.h"
//M1 --> Motor x
//M2 --> Motor y
// [x,y] --> [M1,M2]
#define M1_STEP 33
#define M1_DIR 25

#define M2_STEP 18
#define M2_DIR 21

//*****************************************//
//****************** BPV ******************//
//*****************************************//
struct BPV {
  byte name[18];
  byte offSet_X;
  byte offSet_Y;
  BPV() {
    offSet_X = 0;
    offSet_X = 0;
  }
};

void InitBPV();
//*****************************************//
//***************** STTEPER ***************//
//*****************************************//
struct STEPPER {
  //Configuración software
  uint16_t posStart = 0;
  uint16_t posNow = 0;
  uint16_t posTarget = 0;

  uint16_t speedStart = 0;
  uint16_t speedNow = 0;
  uint16_t speedTarget = 0;

  uint16_t speedCont = 0;

  uint8_t dir = 0;
  uint8_t movementDone = 1;

  //Configuración física
  uint8_t pinDir = 0;
  uint8_t pinStep = 0;
};
extern STEPPER stepper1X;

struct POS {
  uint16_t posX;
  uint16_t posY;
  uint16_t wait;
  uint16_t vel;
  POS() {
    posX = 0;
    posY = 0;
    wait = 0; //Wait in same place. Then start moving
    vel = 0;
  }
};

void IRAM_ATTR onTimer1();
void InitMotors();
void gotoPos(uint16_t target, uint16_t vel, uint8_t istepper);
void gotoPosXY(uint16_t posX, uint16_t posY, uint16_t vel);
void addPos(uint16_t posX, uint16_t posY, uint16_t wait, uint16_t vel, uint8_t i);
void runBlock();


void M1_Dir(uint8_t pin, uint8_t dir);
void M2_Dir(uint8_t pin, uint8_t dir);
void M1_Dir(int dir);
void M2_Dir(int dir);
void resetStepperValues(STEPPER* stepper, uint8_t pinStep, uint8_t pinDir);

#endif