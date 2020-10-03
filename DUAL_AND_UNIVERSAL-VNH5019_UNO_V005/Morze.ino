void toneRazdray() {
  //tone(BUZZER,(razdray*10),1000);   // старая функция тональности
  tone(BUZZER, ((razdray / 5 - 9) * 440), 1000);
  delay(1000);
}

void toneSaveeprom() {
  int i = 0;
  do {
    tone(BUZZER, 1000, DIT);
    delay(DIT + PAUSE);
    tone(BUZZER, 1000, DASH);
    delay(DASH + PAUSE);
    tone(BUZZER, 1000, DIT);
    delay(DIT + PAUSE);
    delay(2 * PAUSE); i++;
  } while (i < 3);
  delay(2 * PAUSE);
}

void toneSOS() {
  do {
    int i = 0; do {
      tone(BUZZER, 1000, DIT);
      delay(DIT + PAUSE);
      i++;
    } while (i < 3); delay(2 * PAUSE);
    int j = 0; do {
      tone(BUZZER, 1000, DASH);
      delay(DASH + PAUSE);
      j++;
    } while (j < 3); delay(2 * PAUSE);
    int k = 0; do {
      tone(BUZZER, 1000, DIT);
      delay(DIT + PAUSE);
      k++;
    } while (k < 3); delay(3 * PAUSE);
  } while (Icnt > 3);
}

void tone_isk(void) {
  tone(BUZZER, 1000, DASH);
  delay(DASH + PAUSE);
  tone(BUZZER, 1000, DASH);
  delay(DASH + PAUSE);
  tone(BUZZER, 1000, DASH);
  delay(DASH + PAUSE);
  delay(PAUSE + PAUSE + PAUSE);
  tone(BUZZER, 1000, DASH);
  delay(DASH + PAUSE);
  tone(BUZZER, 1000, DIT);
  delay(DIT + PAUSE);
  tone(BUZZER, 1000, DASH);
  delay(DASH + PAUSE);
}

void toneS() {
  int i=0; do{ tone(BUZZER,1000,DIT); delay(DIT+PAUSE);i++;} while(i<3); delay(2*PAUSE);
  }
