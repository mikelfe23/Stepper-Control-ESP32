#ifndef _StepperESP_h
#define _StepperESP_h

#include "Arduino.h"
#include "Utiles.h"
#include <math.h>

#ifdef ESP8266

#else

#include "freertos/portmacro.h"
#include "esp_attr.h"
#include "esp32-hal-timer.h"

#endif

//M1 --> Motor x
//M2 --> Motor y
// [x,y] --> [M1,M2]
#define M_RST 17      //nOUT
#define M_SLEEP 16    //nOUT

#define M1_STEP 33    //OUT
#define M1_DIR 26     //OUT
#define M1_ENABLE 25  //nOUT
#define M1_FAULT 27   //In

#define M2_STEP 18
#define M2_DIR 21


#define M2_ENABLE 19  //nOUT
#define M2_FAULT 34   //In

#define MAXSTEPS 500
#define MINSTEPS 1
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

  uint16_t stepsCont = 0;
  uint16_t stepsTarget = 0;

  uint8_t dir = 0;
  uint8_t movementDone = 1;

  //Configuración física
  uint8_t pinDir = 0;
  uint8_t pinStep = 0;
  uint8_t pinEn = 0;
  uint8_t pinFault = 0;
  uint8_t pinRST = 0;
  uint8_t pinSleep = 0;
};


struct POS {
  uint16_t posX;
  uint16_t posY;
  uint16_t wait;
  uint16_t vel;
  POS() {
    posX = 0;
    posY = 0;
    wait = 0;  //Wait in same place. Then start moving
    vel = 0;
  }
};
//*****************************************//
//*****************  ESP32  ***************//
//*****************************************//

#ifdef ESP8266
void InitTimer1();
#else
void IRAM_ATTR onTimer1();
void InitTimer1();

#endif
//*****************************************//
//************ ESP8266 & ESP32 ************//
//*****************************************//
void InitMotors();
void gotoPos(uint16_t target, uint16_t vel, uint8_t istepper);
void gotoPosXY(uint16_t posX, uint16_t posY, uint16_t vel);
void addPos(uint16_t posX, uint16_t posY, uint16_t wait, uint16_t vel, uint8_t i);
void runBlock();

uint16_t LaCAJA(STEPPER* steppe, uint16_t stepsMin, uint16_t stepsMax);
uint16_t traducirVelocidad(uint16_t speed, uint16_t stepsMin, uint16_t stepsMax);
void M1_Dir(uint8_t pin, uint8_t dir);
void M2_Dir(uint8_t pin, uint8_t dir);
void M1_Dir(int dir);
void M2_Dir(int dir);
void resetStepperValues(STEPPER* stepper, uint8_t pinStep, uint8_t pinDir, uint8_t pinEn, uint8_t pinFault, uint8_t pinRST, uint8_t pinSleep);
#endif


//Si me muevo en xy la velocidad que define el la hipotenusa
/*
    Tengo que obetner los puntos de inicio y final. asjusar sus velocidades vel0 y vel 1 para que cuadren con la de la hipotenusa
    h^2  = k0^2 + k1^2
  */

//En 32 microesteping
//velMax 1000
//velMin    10 Puede ser 0. ay que 250kz -> 4us es menos que los 100us que le meto con el timer
//Tiempo entre pasos = vel. 1 paso son 100u
// 10 pasos es velocidad datos
// 100 pasos +- velocidad media
// 300 lenta
