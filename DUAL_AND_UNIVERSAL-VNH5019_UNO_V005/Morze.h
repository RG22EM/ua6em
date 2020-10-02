#pragma once
#include <Arduino.h>

#ifndef BUZZER
#define BUZZER 13
#endif 

void toneS() {
  int i=0; do{ tone(BUZZER,1000,DIT); delay(DIT+PAUSE);i++;} while(i<3); delay(2*PAUSE);
  }
