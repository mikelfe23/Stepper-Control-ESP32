#include "esp32-hal-timer.h"
/*#include "freertos/portmacro.h"
#include "esp_attr.h"*/
#include "BPV.h"
#include "Utiles.h"
#include <math.h>
hw_timer_t* timerMotor = NULL;
static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

STEPPER stepper1X;
STEPPER stepper[2];

POS posArray[20];
uint8_t runBlockIndex = 0;

void InitBPV() {
  debugPrint("\n[BPV]Iniciando timer:", (uint16_t)1);
  timerMotor = timerBegin(0, 80, true);  //12,5ns * 80 = 1us
  timerAttachInterrupt(timerMotor, &onTimer1, true);
  timerAlarmWrite(timerMotor, 100, true);  //void timerAlarmWrite(hw_timer_t *timer, uint64_t alarm_value, bool autoreload);
}

void InitMotors() {  //Definir aqui los pines desde el main o en el BPV.h
  debugPrint("\n[BPV]Iniciando Motores:", (uint16_t)1);

  resetStepperValues(&stepper[0], M1_STEP, M1_DIR);
  pinMode(stepper[0].pinStep, OUTPUT);
  pinMode(stepper[0].pinDir, OUTPUT);

  resetStepperValues(&stepper[1], M2_STEP, M2_DIR);
  pinMode(stepper[1].pinStep, OUTPUT);
  pinMode(stepper[1].pinDir, OUTPUT);

  timerAlarmEnable(timerMotor);
}



void gotoPos(uint16_t target, uint16_t vel, uint8_t istepper) {
  // int steps = (int)stepper[i].posTarget - (int)stepper[i].posNow;

  stepper[istepper].speedTarget = vel;
  stepper[istepper].speedCont = vel;

  stepper[istepper].posTarget = target;
  stepper[istepper].movementDone = 0;

  debugPrint("\n[BPV] IR a:", (uint16_t)1);
  debugPrint("\n[BPV] stepper[istepper].posNow     :", stepper[istepper].posNow);
  debugPrint("\n[BPV] stepper[istepper].posTarget  :", stepper[istepper].posTarget);
  debugPrint("\n[BPV] stepper[istepper].speedTarget:", stepper[istepper].speedTarget);
  debugPrint("\n[BPV] stepper[istepper].speedCont  :", stepper[istepper].speedCont);
  debugPrint("\n[BPV] stepper[istepper].dir        :", stepper[istepper].dir);
}
void gotoPosXY(uint16_t posX, uint16_t posY, uint16_t vel) {
  float pulseCount = 0;
  //Si me muevo en xy la velocidad que define el la hipotenusa
  /*
    Tengo que obetner los puntos de inicio y final. asjusar sus velocidades vel0 y vel 1 para que cuadren con la de la hipotenusa
    h^2  = k0^2 + k1^2
  */

  //En 32 microesteping
  //velMax 1000
  //velMin    10
  //Tiempo entre pasos = vel. 1 paso son 100u
  // 10 pasos es velocidad datos
  // 100 pasos +- velocidad media
  // 300 lenta

  vel = constrain(vel, 0, 100);
  pulseCount = map(vel, 0, 100, 300, 0);


  //float kX = stepper[0].posTarget - stepper[0].posNow;
  //float kY = stepper[1].posTarget - stepper[1].posNow;
  stepper[0].posStart = stepper[0].posTarget;
  stepper[1].posStart = stepper[1].posTarget;

  float kX = abs(posX - stepper[0].posStart);
  float kY = abs(posY - stepper[1].posStart);


  float h = sqrt(pow(kX, 2) + pow(kY, 2));

  debugPrint("\n[BPV] IR a kX :");
  Serial.print(kX);
  debugPrint("\n[BPV] IR a kY :");
  Serial.print(kY);
  debugPrint("\n[BPV] IR a h  :");
  Serial.print(h);


  float pulseCountX = 0;
  float pulseCountY = 0;

  if (kX != 0)
    pulseCountX = pulseCount * (h / kX);
  if (kY != 0)
    pulseCountY = pulseCount * (h / kY);

  debugPrint("\n[BPV] IR a X:", posX);
  debugPrint("\n[BPV] IR a Y:", posY);
  debugPrint("\n[BPV] IR a pulseCount :", pulseCount);
  debugPrint("\n[BPV] IR a pulseCountX:", pulseCountX);
  debugPrint("\n[BPV] IR a pulseCountY:", pulseCountY);


  stepper[0].speedTarget = pulseCountX;
  stepper[0].speedCont = pulseCountX;
  stepper[0].posTarget = posX;

  stepper[1].speedTarget = pulseCountY;
  stepper[1].speedCont = pulseCountY;
  stepper[1].posTarget = posY;
}

void IRAM_ATTR onTimer1() {
  // portENTER_CRITICAL(&mutex);
  for (int i = 0; i <= 1; i++) {
    digitalWrite(stepper[i].pinStep, LOW);

    if ((stepper[i].posTarget != stepper[i].posNow) && (stepper[i].movementDone == 0)) {  //Ir hacia el punto de convergencia. Mover motor
      //stepper[i].movementDone = 0;
      if (stepper[i].posTarget >= stepper[i].posNow) {
        stepper[i].dir = 1;
        digitalWrite(stepper[i].pinDir, HIGH);
      } else {
        stepper[i].dir = 0;
        digitalWrite(stepper[i].pinDir, LOW);
      }

      if (stepper[i].speedCont != 0) {
        stepper[i].speedCont--;

      } else {
        if (stepper[i].dir == 1) {
          stepper[i].posNow++;
        } else {
          stepper[i].posNow--;
        }
        float stepsToDo = stepper[i].posTarget - stepper[i].posStart;
        float stepsDone = stepper[i].posTarget - stepper[i].posNow;

        if (stepsDone < stepsToDo * 0.2) {
          stepper[i].speedCont = stepper[i].speedTarget;

        } else if (stepsDone < stepsToDo * 0.8) {
          stepper[i].speedCont = stepper[i].speedTarget;
        
        } else {
          stepper[i].speedCont = stepper[i].speedTarget;
        
        }


        digitalWrite(stepper[i].pinStep, HIGH);
      }

    } else {  //Resetear los valores relativos.
      stepper[i].movementDone = 1;
    }
  }
  // portEXIT_CRITICAL(&mutex);
}


void addPos(uint16_t posX, uint16_t posY, uint16_t wait, uint16_t vel, uint8_t i) {
  posArray[i].posX = posX;
  posArray[i].posY = posY;
  posArray[i].wait = wait;
  posArray[i].vel = vel;
}

void runBlock() {
  if (stepper[0].movementDone && stepper[1].movementDone) {  //TODO Parar interrpciones
    if (posArray[runBlockIndex].vel == 0) {
      runBlockIndex = 0;

    } else {
      delay(posArray[runBlockIndex].wait);

      gotoPosXY(posArray[runBlockIndex].posX, posArray[runBlockIndex].posY, posArray[runBlockIndex].vel);
      runBlockIndex++;
      stepper[0].movementDone = 0;
      stepper[1].movementDone = 0;
    }
  } else {
    Serial.print("\nMoving...");
    delay(20);
  }
}
void resetStepperValues(STEPPER* stepper, uint8_t pinStep, uint8_t pinDir) {
  stepper->posStart = 0;
  stepper->posNow = 0;
  stepper->posTarget = 0;
  stepper->speedStart = 0;
  stepper->speedNow = 0;
  stepper->speedTarget = 0;

  stepper->speedCont = 0;

  stepper->dir = 0;
  stepper->movementDone = 1;
  stepper->pinStep = pinStep;
  stepper->pinDir = pinDir;
}

//Motor dir definition
void M1_Dir(uint8_t pin, uint8_t dir) {
  digitalWrite(pin, dir);
}
void M2_Dir(uint8_t pin, uint8_t dir) {
  digitalWrite(pin, dir);
}

void M1_Dir(int dir) {
  digitalWrite(M1_DIR, dir);
}
void M2_Dir(int dir) {
  digitalWrite(M2_DIR, dir);
}