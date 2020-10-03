/*
 * Программа сервотестера для ARDUINO NANO
 * 02.11.2018 - переход на библиотеку TimeMeasure.h
 * АКА  ЕвгенийП www.arduino.ru (в папке скетча)
 */

 // Измерение проводим через пин 2
#include  "TimeMeasure.h"
volatile uint16_t t = 0;
void setup() {
  Serial.begin(57600);
  initTimeMeasuring();
}

//
//  Если готов результат измерения, печатаем
//
void loop() {
  const uint16_t res = measureResult();
   t = ticks2Microseconds(res);
  if (t) Serial.println(t);
}
