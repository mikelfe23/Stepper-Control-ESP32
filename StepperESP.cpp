#include "StepperESP.h"

STEPPER stepper[2];
POS posArray[20];
uint8_t runBlockIndex = 0;

#ifdef ESP8266
void InitTimer1() {
  debugPrint("\n NO ESTA BIEN DEFINIDO EL HARDWARE");
}
#else

hw_timer_t* timerMotor = NULL;
//static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

void InitTimer1() {
  //Iniciar Timers
  debugPrint("\n[BPV]Iniciando timer:", (uint16_t)1);
  timerMotor = timerBegin(0, 80, true);  //12,5ns * 80 = 1us
  timerAttachInterrupt(timerMotor, &onTimer1, true);
  timerAlarmWrite(timerMotor, 100, true);
  //Habilitarlo despues de iniciar los motores
  timerAlarmEnable(timerMotor);
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

      if (stepper[i].stepsCont != 0) {
        stepper[i].stepsCont--;

      } else {
        if (stepper[i].dir == 1) {
          stepper[i].posNow++;
        } else {
          stepper[i].posNow--;
        }

        stepper[i].stepsCont = traducirVelocidad(stepper[i].speedTarget, MINSTEPS, MAXSTEPS);
        //stepper[i].stepsCont = LaCAJA(&stepper[i], MINSTEPS, MAXSTEPS);

        digitalWrite(stepper[i].pinStep, HIGH);
      }

    } else {  //Resetear los valores relativos.
      stepper[i].movementDone = 1;
    }
  }
  // portEXIT_CRITICAL(&mutex);
}

#endif

void InitMotors() {  //Definir aqui los pines desde el main o en el BPV.h
  //Iniciamos motores
  debugPrint("\n[BPV]Iniciando Motores:", (uint16_t)1);

  resetStepperValues(&stepper[0], M1_STEP, M1_DIR, M1_ENABLE, M1_FAULT, M_RST, M_SLEEP);
  pinMode(stepper[0].pinStep, OUTPUT);
  pinMode(stepper[0].pinDir, OUTPUT);
  pinMode(stepper[0].pinEn, OUTPUT);
  pinMode(stepper[0].pinFault, INPUT);

  resetStepperValues(&stepper[1], M2_STEP, M2_DIR, M2_ENABLE, M2_FAULT, M_RST, M_SLEEP);
  pinMode(stepper[1].pinStep, OUTPUT);
  pinMode(stepper[1].pinDir, OUTPUT);
  pinMode(stepper[1].pinEn, OUTPUT);
  pinMode(stepper[1].pinFault, INPUT);

  pinMode(M_RST, OUTPUT);
  pinMode(M_SLEEP, OUTPUT);

  digitalWrite(M_RST, HIGH);
  digitalWrite(M_SLEEP, HIGH);

  digitalWrite(stepper[0].pinEn, LOW);
  digitalWrite(stepper[1].pinEn, LOW);
}

void gotoPosXY(uint16_t posX, uint16_t posY, uint16_t vel) {
  vel = constrain(vel, 0, 100);

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


  float velX = 0;
  float velY = 0;

  if (kX != 0)
    velX = vel * (kX / h);
  if (kY != 0)
    velY = vel * (kY / h);

  debugPrint("\n[BPV] IR a X:", posX);
  debugPrint("\n[BPV] IR a Y:", posY);
  debugPrint("\n[BPV] IR a vel :", vel);
  debugPrint("\n[BPV] IR a velX:", velX);
  debugPrint("\n[BPV] IR a velY:", velY);

  stepper[0].speedStart = 0;
  stepper[0].speedTarget = velX;
  stepper[0].stepsCont = 0;
  stepper[0].posTarget = posX;

  stepper[1].speedStart = 0;
  stepper[1].speedTarget = velY;
  stepper[1].stepsCont = 0;
  stepper[1].posTarget = posY;

  debugPrint("\n[BPV] stepper[0].posTarget:", stepper[0].posTarget);
  debugPrint("\n[BPV] stepper[0].posStart:", stepper[0].posStart);

  debugPrint("\n[BPV] stepper[1].posTarget:", stepper[1].posTarget);
  debugPrint("\n[BPV] stepper[1].posStart:", stepper[1].posStart);
}

uint16_t traducirVelocidad(uint16_t speed, uint16_t stepsMin, uint16_t stepsMax) {
  return map(speed, 0, 100, stepsMax, stepsMin);
}

uint16_t LaCAJA(STEPPER* stepper, uint16_t stepsMin, uint16_t stepsMax) { //TODO tiene un bug que genera una excepción
  uint16_t escalado = 1;
  float porcentaje = 0.1f;
  float stepsToDo = abs(stepper->posTarget - stepper->posStart);
  float stepsDone = abs(stepper->posNow - stepper->posStart);
  float malda = (stepper->speedTarget - stepper->speedStart) / (stepsToDo * porcentaje);

  if (stepsDone < stepsToDo * porcentaje) {
    stepper->speedNow = escalado * stepper->speedStart + malda * stepsDone;
    //stepper->stepsCont = stepper->speedStart - malda * stepsDone;

  } else if (stepsDone <= (stepsToDo * (1.0f - porcentaje))) {
    stepper->speedNow = escalado * stepper->speedTarget;

  } else {
    stepper->speedNow = escalado * stepper->speedTarget - malda * ((stepsToDo * porcentaje) - (stepsToDo - stepsDone));
    //stepper->stepsCont = stepper->speedStart - malda * (stepsToDo - stepsDone);
  }


  uint16_t stepsCount = (uint16_t)round(mapfloat(stepper->speedNow, 0, escalado * 100, stepsMax, stepsMin));
  return stepsCount;
}


void addPos(uint16_t posX, uint16_t posY, uint16_t wait, uint16_t vel, uint8_t i) {
  posArray[i].posX = posX;
  posArray[i].posY = posY;
  posArray[i].wait = wait;
  posArray[i].vel = vel;
}
void clearPos(uint8_t i) {
  posArray[i].posX = 0;
  posArray[i].posY = 0;
  posArray[i].wait = 0;
  posArray[i].vel = 0;
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

    debugPrint("\nStepsContX: ", stepper[0].stepsCont);
    debugPrint("\tStepsContY: ", stepper[1].stepsCont);
    debugPrint("\nSpeedTarget: ", stepper[1].speedTarget);
  }
}
void resetStepperValues(STEPPER* stepper, uint8_t pinStep, uint8_t pinDir, uint8_t pinEn, uint8_t pinFault, uint8_t pinRST, uint8_t pinSleep) {
  stepper->posStart = 0;
  stepper->posNow = 0;
  stepper->posTarget = 0;
  stepper->speedStart = 0;
  stepper->speedNow = 0;
  stepper->speedTarget = 0;

  stepper->stepsCont = 0;

  stepper->dir = 0;
  stepper->movementDone = 1;
  stepper->pinStep = pinStep;
  stepper->pinDir = pinDir;
  stepper->pinEn = pinEn;
  stepper->pinFault = pinFault;
  stepper->pinRST = pinRST;
  stepper->pinSleep = pinSleep;
}
