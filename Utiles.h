
#ifndef _Utiles_h
#define _Utiles_h

#include "Arduino.h"
#define PRINT 1
void ponerACero(byte* a, byte len);
void debugPrint(String msg, byte* a);
void debugPrint(String msg);
void debugPrint(String msg, uint16_t val);
//void debugPrint(String msg, float val);
void debugPrintHEX(String msg, byte* a, byte len);
byte cmpArray(byte* a, byte* b, byte len);
byte calculateCheckSum(uint8_t* data);
byte calculateCheckSum(uint8_t* data,uint8_t tam);


#endif