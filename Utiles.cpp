#include "Utiles.h"

void ponerACero(byte* a, byte len) {
  for (int n = 0; n < len; n++) {
    a[n] = 0;  //TODO mirar que casca aquí
  }
}

byte cmpArray(byte* a, byte* b, byte len) {  //Iguales --> 1 //Diferentes --> 0
  byte equal = 1;
  for (int i = 0; i < len; i++) {
    if (a[i] != b[i]) {
      equal = 0;
      break;
    }
  }
  return equal;
}
void debugPrint(String msg, byte* a) {
  if (PRINT == 1) {
    Serial.print(msg);
    Serial.print("# ");
    Serial.print(String((char*)a));
    Serial.print(" #");
  }
}
void debugPrint(String msg, uint16_t val) {
  if (PRINT == 1) {
    Serial.print(msg);
    Serial.print("# ");
    Serial.print(val);
    Serial.print(" #");
  }
}
/*void debugPrint(String msg, float val) {
  if (PRINT == 1) {
    Serial.print(msg);
    Serial.print("# ");
    Serial.print(val);
    Serial.print(" #");
  }
}*/
void debugPrint(String msg) {
  if (PRINT == 1) {
    Serial.print(msg);
  }
}

void debugPrintHEX(String msg, byte* a, byte len) {
  if (PRINT == 1) {
    Serial.print(msg);
    Serial.print("#");
    Serial.print(' ');
    for (int i = 0; i < len; i++) {
      Serial.print(a[i], HEX);
      Serial.print(' ');
    }

    Serial.print("#");
  }
}
void debugPrint(String msg1, char* a1, String msg2, char* a2) {
  if (PRINT == 1) {
    Serial.print(msg1);
    Serial.print(a1);
    Serial.print(msg2);
    Serial.print(a2);
  }
}

byte calculateCheckSum(uint8_t* data) {
  byte checksum = 0;
  debugPrint("\n[CALCULATE CHECKSUM] tam:",sizeof(data));
  for (int i = 0; i < sizeof(data) - 1; i++) {  //TODO No tiene porque ir el checksum al final del array. Porque en memoria se ordena como se puede
    checksum = checksum + data[i];
  }
  return checksum;
}
byte calculateCheckSum(uint8_t* data,uint8_t tam) {
  byte checksum = 0;
  debugPrint("\n[CALCULATE CHECKSUM] tam:",tam);
  for (int i = 0; i < tam - 1; i++) {  //TODO No tiene porque ir el checksum al final del array. Porque en memoria se ordena como se puede
    checksum = checksum + data[i];
  }
  return checksum;
}
