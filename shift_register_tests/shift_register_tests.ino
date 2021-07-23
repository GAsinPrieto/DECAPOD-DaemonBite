#include "shift_74597.h"

#define QH 2
#define SCK 4
#define RCK 9
#define SLOAD 9 //SRLOAD
//#define SCLR

shift_74597 myShifter = shift_74597(QH, SCK, RCK, SLOAD);//, SCLR);

void setup() {
  // put your setup code here, to run once:
  myShifter.init();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  myShifter.load();
  char myInput0 = myShifter.getByte();
  char myInput1 = myShifter.getByte();
  char myInput2 = myShifter.getByte();

  Serial.print("primero: ");
  Serial.print((myInput0&0xFF),BIN);
  Serial.print(" - segundo: ");
  Serial.print((myInput1&0xFF),BIN);
  Serial.print(" - tercero: ");
  Serial.print((myInput2&0xFF),BIN);
  Serial.println("");
  
  delay(1000);
}
