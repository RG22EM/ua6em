 /*
  Контроллер управления Н мостами на основе микросхемы VNH5019 30A  для судомоделизма
  Шилд Ардуино UNO - два канала
  Схема шилда взята с сайта - https://www.pololu.com/file/0J513/dual_VNH5019_shield_ash02a_schematic.pdf
   Обработчики прерываний - https://tsibrov.blogspot.com/2019/06/arduino-interrupts-part2.html
   D8 .. D13 - генерируют запрос прерывания PCINT0
   A0 .. A5 - генерируют запрос прерывания PCINT1
   D0 .. D7 - генерируют запрос прерывания PCINT2t
   на MID требуется введение гистерезиса
   22.12.2019 - ввёл, 12 единиц (от 500)
   25.12.2019 - проба библиотеки Гайвера - https://community.alexgyver.ru/resources/biblioteka-gyverbutton.1/
   31.12.2019 - режим калибровки отлажен
   27.09.2020 - вынесено в GIT
   - https://github.com/RG22EM/ua6em/tree/master/DUAL_AND_UNIVERSAL-VNH5019_UNO_V005
   - добавлена функция расчета уставок включения раздрая после калибровки
     аппаратуры, данные берутся из энергозависимой памяти.
   02.10.2020
   - изменена функция тона для раздрая, теперь каждое изменение сигнализирует тоном
     на одну ноту выше, по круговому циклу 50% - 440гц 55%-880гц и т.д.
   - изменена функция рассчета раздрая для Рудера взависимости от установленных
     на радиоаппаратуре расходов
*/
// Включить отладку, включить режим калибровки, проверить,
// в мониторе порта, что все значения запомнились кореектно
// в рабочем скетче - закомментировать
#define DEBUG     // Режим отладка включен

// ***  Выбор для третьего двигателя ПОДРУЛЬКА-МИД  ***
// При снятии комментария третий двигатель работает как MID
// и берёт данные от ручки газа (канал CH1), включается в работу в зависимости
// от установок MID_UP и MID_DOWN, если определение закомментировано, то третий
// двигатель получает данные из канала CH3 (режим подруливания)
#define MIDS

// *** При снятии комментария аналоговые пины измеряют защиту по току ***
//#define ZASHITA // Включить процедуру защиты по току

// Работа с кнопками возможна любой из двуз библиотек
#define GYVER     // Через библиотеку Гайвера иначе QWONE

//#define REVERSE_CH2 // Снять комментарии, если канал CH2 требует реверса
// работа в этом режиме не проверялась

//#define REVERSE_CH3 // Снять комментрарии, если канал CH3 требует реверса
// работа в этом режиме не проверялась

// ************ Настройка таймеров ************
//#define PWM_490
//#define PWM_980
#define PWM_4000
//#define PWM_8000
//
//  Делители таймеров на 490Гц
#ifdef PWM_490  // 490Гц CLK/64
constexpr byte prescaler0 = bit(CS00) | bit(CS01);
constexpr byte prescaler1 = bit(CS10) | bit(CS11);
constexpr byte prescaler2 = bit(CS22);
#endif

//  Делители таймеров на 1 кГц
//
#ifdef PWM_980  //  на самом деле 980Гц CLK/64 для FAST PWM
constexpr byte prescaler0 = bit(CS00) | bit(CS01);
constexpr byte prescaler1 = bit(CS10) | bit(CS11);
constexpr byte prescaler2 = bit(CS22);
#endif

//  Делители таймеров на 4 и 8 кГц
//
#ifdef PWM_4000 // 4 кГц  CLK/8
constexpr byte prescaler0 = bit(CS01);
constexpr byte prescaler1 = bit(CS11);
constexpr byte prescaler2 = bit(CS21);
#endif

#ifdef PWM_8000 // 8кГц (на самом деле 7812.5 Гц) CLK/8 для FAST PWM
constexpr byte prescaler0 = bit(CS01);
constexpr byte prescaler1 = bit(CS11);
constexpr byte prescaler2 = bit(CS21);
#endif

// pins on Arduino UNO ( Atmega328P )
// подключаем библиотеки
#include <EEPROM.h>
#include <util/atomic.h>

// RC input channels
#define CH1 3   //D3 throttle
#define CH2 5   //D5 rudder (PCINT2 - групповой) PCINT21
#define CH3 17  //D17(A3) PCINT11 (PCINT1 - групповой)

// Определения констант для раздрая, если использовать
// возможность установки расходов придётся вводить
// переменные и расчет уставок
#define CH2RR 1850 // Точка включения раздрая Right по умолчанию 
#define CH2RL 1150 // Точка включения раздрая Left по умолчанию
uint16_t ch2rr = 1850;
uint16_t ch2rl = 1150;
uint16_t ch2null = 1500;

//motor direction pins
#define DIR_LEFT2    4    // M1INB  direction out for LEFT motor
#define DIR_LEFT1    2    // M1INA
#define PWM_LEFT     9    // M1PWM  PWM outputs
// LOW level = motor OFF, HIGH level = motor ON
#define  M1ENAB      6    // M1ENA/B   - HIGH разрешение выхода

#define DIR_RIGHT2   8    // M2INB  direction out for RIGHT motor
#define DIR_RIGHT1   7    // M2INA
#define PWM_RIGHT   10    // M2PWM PWM outputs
// LOW level = motor OFF, HIGH level = motor ON
#define  M2ENAB     12    // M2ENA/B  - HIGH разрешение выхода  

#define DIR_RUDER1  18    //A4  MRINA
#define DIR_RUDER2  19    //A5  MRINB
#define PWM_RUDER   11    //    MRPWM PWM outputs
#define MRENAB            //    MRENA/B  - HIGH разрешение выхода  

#define DIR_MID1    18    //A4  MIDINA
#define DIR_MID2    19    //A5  MIDINB
#define PWM_MID     11    //    MIDPWM PWM outputs
#define MIDENAB           //    MIDENA/B  - HIGH разрешение выхода 
#define MID_UP    1650    // значение PWM для включения MID вперёд
#define MID_DOWN  1350    // значение PWM для включения MID назад  
#define MID_DELTA   12    // дельта на гистерезис 12 микросекунд 

#define BUZZER      13    //D11 Пищалка для настройки раздрая (пассивная)
#define BUZLONG   1000    // Время звучания зуммера в миллисекундах
#define BUTTON      16    // D16 (A2) кнопка программирования 
//#define LED         13    // D13 моргаем светодиодом (c подрулькой не используется)
#define CSD1        A0    // Ток защиты двигателя 1 (140 милливольт на Ампер)
#define CSD2        A1    // Ток защиты двигателя 2 (140 милливольт на Ампер)
#define CSD3        A6    // Ток защиты двигателя 3 (140 милливольт на Ампер)
#define VPP         A7    // Напряжение аккумулятора
#include "GyverButton.h"

// EEPROM адресация
#define ADREPROM     1    // ячейки памяти - razdray
#define ADRICNT      2    // ячейки памяти - Icnt 

#define MIN 1000       // 1150
#define MAX 2000       // 1850
#define HALF ((MAX-MIN)/2)
#define MID  ((MIN+MAX)/2)
#define STOP_ZONE 30    // 80
#define PWM_MAX 510
#define SOScounter 3    // Счётчик числа срабатываний защиты по току 
#define IReg 5          // Ток защиты в Амперах
#define Amper 1000      // Миллиампер в Ампере

#define DIT   100
#define DASH  300
#define PAUSE 100

volatile byte sharedFlag1 = 0;
volatile byte sharedFlag2 = 0;
volatile byte sharedFlag3 = 0;
volatile byte razdray = 80;
volatile byte Icnt = 0;
volatile byte delta = 0;

volatile int sharedCh1 = 1500;
volatile int sharedCh2 = 1500;
volatile int sharedCh3 = 1500;
volatile int mid_up;
volatile int mid_down;
volatile unsigned int THROTTLE_UP = 1500;
volatile unsigned int THROTTLE_DOWN = 1500;
volatile unsigned int RUDDER_UP = 1500;
volatile unsigned int RUDDER_DOWN = 1500;
volatile unsigned int CH3_UP = 1500;
volatile unsigned int CH3_DOWN = 1500;

#define DELAY_REVERS 73
unsigned long timeL; // переменная под  millis()
unsigned long timeR; // переменная под  millis()
unsigned long mill; // переменная под  millis()
byte flagTimeL = 0;
byte flagTimeR = 0;

struct MyAppa {
  byte  flagAppa;
  uint16_t throttleUp;
  uint16_t throttleDown;
  uint16_t rudderUp;
  uint16_t rudderDown;
  uint16_t ch3Up;
  uint16_t ch3Down;
};
MyAppa appa;
int eeAddress = 5;

/*** Обработчик кнопки ***/
//------Cl_Btn----------------------
enum {sbNONE = 0, sbClick, sbLong}; /*состояние не изменилось/клик/долгое нажатие*/
class Cl_Btn {
  protected:
    const byte pin;
    byte state;
    bool bounce = 0;
    bool btn = 1, oldBtn;
    unsigned long past;
    const uint32_t time = 500 ;
    bool flag = 0;
    uint32_t past_flag = 0 ;
  public:
    Cl_Btn(byte p): pin(p) {}
    /*инициализация-вставить в setup()*/
    void init() {
      pinMode(pin, INPUT_PULLUP);
    }
    /*работа-вставить в loop()*/
    void run() {
      state = sbNONE;
      bool newBtn = digitalRead(pin);
      if (!bounce && newBtn != btn) {
        bounce = 1;
        past = mill;
      }
      if (bounce && mill - past >= 10) {
        bounce = 0 ;
        oldBtn = btn;
        btn = newBtn;
        if (!btn && oldBtn) {
          flag = 1;
          past_flag = mill;
        }
        if (!oldBtn && btn && flag && mill - past_flag < time ) {
          flag = 0;
          state = sbClick;
        }
      }
      if (flag && mill - past_flag >= time ) {
        flag = 0;
        state = sbLong;
      }
    }
      byte read() {
      return state;
    }
};

#ifdef GYVER
GButton Btn1(BUTTON);
#else
Cl_Btn Btn1(/*пин*/BUTTON); //Экземпляр обработчика для кнопки
#endif

//------Cl_Potentiometer----- используем для измерения тока с датчиков
class Cl_Potentiometer {
    const byte _pin;
    int *const _pnt;// указатель на переменную которая меняет значение по положению ручки потенциометра
    uint32_t past = 0;
  public:
    Cl_Potentiometer(byte pin, int *pnt): _pin(pin), _pnt(pnt) {}
    void setup() {
      *_pnt = map(analogRead(_pin), 0, 1023, 0, 31000); // 31 Ампер
    }
    void loop() {
      if (uint32_t m = millis() - past > 100) {
        *_pnt = map(analogRead(_pin), 0, 1023, 0, 31000);// 31 Ампер
        past = m; //millis(); // Измеряем раз в 100 миллисекунд
      }
    }
};
//Cl_Potentiometer Potentiometer(/*пин потенциометра*/A0,/*переменая*/&time);// подключить потенциометр
int tok1 = 0;
int tok2 = 0;
int tok3 = 0;
Cl_Potentiometer Csd1(/*пин потенциометра*/CSD1,/*переменая*/&tok1);
Cl_Potentiometer Csd2(/*пин потенциометра*/CSD2,/*переменая*/&tok2);
Cl_Potentiometer Csd3(/*пин потенциометра*/CSD3,/*переменая*/&tok3);

//------Cl_Voltmeter----- используем для измерения напряжения
class Cl_Voltmeter {
    const byte _pin;
    int *const _pnt;// указатель на переменную которая меняет значение
    uint32_t past = 0;
  public:
    Cl_Voltmeter(byte pin, int *pnt): _pin(pin), _pnt(pnt) {}
    void setup() {
      *_pnt = map(analogRead(_pin), 0, 1023, 0, 5000); // 5 вольт
    }
    void loop() {
      if (uint32_t m = millis() - past > 100) {
        *_pnt = map(analogRead(_pin), 0, 1023, 0, 5000);// 5 вольт
        past = m; //millis(); // Измеряем раз в 100 миллисекунд
      }
    }
};
//Cl_Voltmeter Voltmeter(/*пин вольтметра*/A0,/*переменая*/&volt);// подключить вольтметр
int volt = 0;
Cl_Voltmeter Upp(/*пин вольтметра*/VPP,/*переменая*/&volt);

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
  
/*
   Подпрограмма калибровки аппаратуры, вход в процедуру включение
   регулятора  с установленным джампером на пине A7.
   Процедура обработки прерываний INT0 и INT1 должны быть уже
   инициализированы.
*/
void calibrates (void) {
  // pinMode(6,INPUT_PULLUP); // было с цифрового порта
  // читаем из аналогового порта A7 (VPP)
  pinMode(13, OUTPUT);
  int k = 0;
  int d;
  static uint16_t chk1;
  static uint16_t chk2;
  static uint16_t chk3;
  do { // здесь ждём 1 секунду или соединения джампа
    d = analogRead(VPP); // Установлена ли перемычка
    if (d < 50) {
      d = 0;
    }
    delay(10); k++;
  } while (k < 100 && d);
/* *
      Serial.print("k=");
      Serial.println(k);
      Serial.print("d=");
      Serial.println(d);
       Serial.print("THROTTLE_UP=");
        Serial.println(THROTTLE_UP);
        Serial.print("THROTTLE_DOWN=");
        Serial.println(THROTTLE_DOWN);
          Serial.print("RUDDER_UP=");
          Serial.println(RUDDER_UP);
          Serial.print("RUDDER_DOWN=");
          Serial.println(RUDDER_DOWN);
           Serial.println("*********************");
*/
  if (!d) {  // здесь процедура калибровки считываем данные стиков в
             // шесть переменных, сохраняем их в EEPROM

    unsigned long change = millis();
    toneS(); // подать звуковой сигнал, что мы в режиме калибровка
    do {
      if (sharedFlag1) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          chk1 = sharedCh1;
        }
      }
#ifdef DEBUG          
          Serial.print("chk1 = "); 
          Serial.println(chk1);
#endif          
          /* *sharedFlag1=0;* */
      if (chk1 > 0) {
        if ((chk1 > 1500 && chk1 > THROTTLE_UP))THROTTLE_UP = chk1;
      }
      if (chk1 > 0) {
        if ((chk1 < 1500 && chk1 < THROTTLE_DOWN))THROTTLE_DOWN = chk1;
      }
      
      if (sharedFlag2) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          chk2 = sharedCh2;
        }
      }
#ifdef DEBUG          
          Serial.print("chk2 = "); 
          Serial.println(chk2);
#endif          
          /* * sharedFlag2=0; * */
      if (chk2 > 0) {
        if ((chk2 > 1500 && chk2 > RUDDER_UP))RUDDER_UP = chk2;
      }
      if (chk2 > 0) {
        if ((chk2 < 1500 && chk2 < RUDDER_DOWN))RUDDER_DOWN = chk2;
      }
      
      if (sharedFlag3) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          chk3 = sharedCh3;
        }
      }
#ifdef DEBUG
          Serial.print("chk3 = "); 
          Serial.println(chk3);
#endif          
          /* * sharedFlag3=0; * */
      
      if (chk3 > 0) {
        if ((chk3 > 1500 && chk3 > CH3_UP))CH3_UP = chk3;
      }
      if (chk3 > 0) {
        if ((chk3 < 1500 && chk3 < CH3_DOWN))CH3_DOWN = chk3;
      }
      delay(57);
    } while ((CH3_UP < 1900 || CH3_DOWN > 1100 || RUDDER_UP < 1900 || RUDDER_DOWN > 1100 || THROTTLE_UP < 1900 || THROTTLE_DOWN > 1100) &&
             ((millis() - change) < 15000));

#ifdef DEBUG
    Serial.print("THROTTLE_UP=");
    Serial.println(THROTTLE_UP);
    Serial.print("THROTTLE_DOWN=");
    Serial.println(THROTTLE_DOWN);
    Serial.print("RUDDER_UP=");
    Serial.println(RUDDER_UP);
    Serial.print("RUDDER_DOWN=");
    Serial.println(RUDDER_DOWN);
    Serial.print("CH3_UP=");
    Serial.println(CH3_UP);
    Serial.print("CH3_DOWN=");
    Serial.println(CH3_DOWN);
#endif
    appa.flagAppa = 88;
    appa.throttleUp = THROTTLE_UP;
    appa.throttleDown = THROTTLE_DOWN;
    appa.rudderUp = RUDDER_UP;
    appa.rudderDown = RUDDER_DOWN;
    appa.ch3Up = CH3_UP;
    appa.ch3Down = CH3_DOWN;

    EEPROM.put(eeAddress, appa);
    toneSaveeprom();

    // выход из калибровки - крутиться  в цикле до снятия джампера
    int s;
    do {
      delay(100);
      digitalWrite(13, !digitalRead(13));
#ifdef DEBUG
      Serial.println("I'M in LOOP");
#endif
      s = analogRead(VPP);
      if (s < 50) {
        s = 0;
      }
    } while ((!s /* analogRead(VPP) */)); // крутимся в цикле пока стоит перемычка
    tone_isk();
#ifdef DEBUG
    Serial.println("Calibrate is OK");
#endif
  } else {
#ifdef DEBUG
    Serial.println("I'M in WORK");
#endif
  }
} // END CALIBRATES


/****************** INT1 *******************/
void int1() {             // Interrupt service routine INT1
  static unsigned long ulStart;
  static unsigned long rch1;
  if (digitalRead(CH1))  {
    ulStart = micros();
  }
  else  {
    rch1 = (int)(micros() - ulStart);
    if (rch1 < 2200 && rch1 > 800) {
      sharedCh1 = rch1;
      sharedFlag1 = 1;
    } else {
      sharedFlag1 = 0;
    }
  }
} // END INT1 (пин 3)


void setCH2() {
  PCICR  |= (1 << PCIE2); // инициализируем порт для приёма CH2
  PCMSK2 |= (1 << PCINT21/*D5*/);
}

void readCH2() {
  static unsigned long ulStart;
  static unsigned long rch2;
  if (digitalRead(CH2))  {
    ulStart = micros();
  }
  else  {
    rch2 = (int)(micros() - ulStart);
    if (rch2 < 2200 && rch2 > 800) {
    sharedCh2=rch2;
    
#ifdef REVERSE_CH2
    sharedCh2 = map(rch2, 800, 2200, 2200, 800);
#endif
    sharedFlag2 = 1;
    
#ifdef DEBUG    
    Serial.print("CH2= ");
    Serial.println(rch2);
#endif    
    } else {
      sharedFlag2 = 0;
    }
  }
}


void setCH3() {
  PCICR  |= (1 << PCIE1); // инициализируем порт для приёма CH3
  PCMSK1 |= (1 << PCINT11/*D17 A3*/);
}

void readCH3() {
  static unsigned long ulStart;
  static unsigned long rch3;
  if (digitalRead(CH3))  {
    ulStart = micros();
  }
  else
  {
    rch3 = (int)(micros() - ulStart);
    if (rch3 < 2200 && rch3 > 800)
    {
    sharedCh3=rch3;
    
#ifdef REVERSE_CH3
       sharedCh3 = map(rch3, 800, 2200, 2200, 800);
#endif
       sharedFlag3 = 1;
#ifdef DEBUG
       Serial.print("CH3= ");
       Serial.println(rch3);
#endif       
    } else {
       sharedFlag3 = 0;
           }
  }
}


/*** Обработчик прерывания для канала CH2 и CH3 (руля и подруливания) ***/
ISR(PCINT2_vect) {
  readCH2();
}

ISR(PCINT1_vect) {
  readCH3();
}

// *** Работает с моторами ***
void setMotor(int outLeft, int outRight, int outRuder) {
#ifdef MIDS     // Как Средний 
  if (delta == 0) { // первое включение в работу двигателя MID
    if (outRuder > mid_up || outRuder < mid_down ) {
      digitalWrite(DIR_RUDER1, outRuder < 0  ? LOW : HIGH);
      delayMicroseconds(4);        // блокируем сквозняки на всякий случай
      digitalWrite(DIR_RUDER2, outRuder < 0  ? HIGH : LOW);
      outRuder = abs(outRuder) >> 1;   // преобразуем делением на 2 в диапазон 0-255
      OCR2A = outRuder;  //  pin 11
      //Serial.print(outMid);
      //Serial.print(",");
      delta = 1; // мотор включен
    } else {
      digitalWrite(DIR_RUDER1, LOW);
      delayMicroseconds(4);
      digitalWrite(DIR_RUDER2, LOW);
      OCR2A = outRuder;
    }
  } else {
    if (outRuder > mid_up - MID_DELTA || outRuder < mid_down + MID_DELTA ) {
      digitalWrite(DIR_RUDER1, outRuder < 0  ? LOW : HIGH);
      delayMicroseconds(4);        // блокируем сквозняки на всякий случай
      digitalWrite(DIR_RUDER2, outRuder < 0  ? HIGH : LOW);
      outRuder = abs(outRuder) >> 1;   // преобразуем делением на 2 в диапазон 0-255
      OCR2A = outRuder;  //  pin 11
      //Serial.print(outMid);
      //Serial.print(",");
      delta = 1; // мотор включен
    } else {
      digitalWrite(DIR_RUDER1, LOW);
      delayMicroseconds(4);
      digitalWrite(DIR_RUDER2, LOW);
      OCR2A = outRuder;
      delta = 0;
    }
  }
#else
  // Мотор подрульки
  if (outRuder == 0) { //Режим СТОП
    digitalWrite(DIR_RUDER1, LOW);
    delayMicroseconds(4);
    digitalWrite(DIR_RUDER2, LOW);
    outRuder = abs(outRuder) >> 1; // преобразуем делением на 2 в диапазон 0-255
    OCR2A = outRuder; //  pin 11  Частота ШИМ по выбору
  } else { // Рабочий режим
    digitalWrite(DIR_RUDER1, outRuder < 0 ? LOW : HIGH);
    delayMicroseconds(4);        // блокируем сквозняки на всякий случай
    digitalWrite(DIR_RUDER2, outRuder < 0 ? HIGH : LOW);
    outRuder = abs(outRuder) >> 1; // преобразуем делением на 2 в диапазон 0-255
    OCR2A = outRuder; //  pin 11  Частота ШИМ по выбору // так не работает
    // analogWrite(11,outRuder); // так работает
    Serial.print("RUDER = ");
    Serial.println(outRuder);
  }
#endif

  if ((outLeft || outRight) == 0) {
    // Левый СТОП
    digitalWrite(DIR_LEFT1, LOW);
    delayMicroseconds(4);
    digitalWrite(DIR_LEFT2, LOW);
    outLeft = abs(outLeft) >> 1; // преобразуем делением на 2 в диапазон 0-255
    OCR1A = outLeft; //  pin 9  Частота ШИМ по выбору

    // Правый СТОП
    digitalWrite(DIR_RIGHT1, LOW);
    delayMicroseconds(4);
    digitalWrite(DIR_RIGHT2, LOW);
    outRight = abs(outRight) >> 1; // преобразуем делением на 2 в диапазон 0-255
    OCR1B  = outRight; //  pin 10 Частота ШИМ по выбору

  } else { // *** Если не режим СТОП то ***

    // Левый
    digitalWrite(DIR_LEFT1  , outLeft < 0 ? LOW : HIGH);
    delayMicroseconds(4);        // блокируем сквозняки на всякий случай
    digitalWrite(DIR_LEFT2 , outLeft < 0 ? HIGH : LOW);
    outLeft = abs(outLeft) >> 1; // преобразуем делением на 2 в диапазон 0-255
    OCR1A = outLeft; //  pin 9  Частота ШИМ 490гц
    // Правый
    digitalWrite(DIR_RIGHT1 , outRight < 0  ? LOW : HIGH);
    delayMicroseconds(4);        // блокируем сквозняки на всякий случай
    digitalWrite(DIR_RIGHT2 , outRight < 0  ? HIGH : LOW);
    outRight = abs(outRight) >> 1; // преобразуем делением на 2 в диапазон 0-255
    OCR1B  = outRight; //  pin 10 Частота ШИМ 490гц
  }
} // *** END SetMotor ***

int stopZone(int in) {
  if (in < -STOP_ZONE) return in + STOP_ZONE; // уменьшить значение на величину
  if (in >  STOP_ZONE) return in - STOP_ZONE; // стоп зоны, если в зоне то - 0
  return 0; // регулировка начинается от 0 стоп зоны до максимума (-вел.стоп зоны)
}


/******************************************************************************/
/*** SETUP ***/
/******************************************************************************/
void setup() {
  Serial.begin(57600);

  pinMode(M1ENAB, OUTPUT);        // Мотор левого борта
  pinMode(DIR_LEFT1, OUTPUT);
  pinMode(DIR_LEFT2, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  digitalWrite(M1ENAB, LOW);   // Задействовать левый мотор
  digitalWrite(DIR_LEFT1, LOW);
  digitalWrite(DIR_LEFT2, LOW);
  digitalWrite(M1ENAB, HIGH);

  pinMode(M2ENAB, OUTPUT);        // Мотор правого борта
  pinMode(DIR_RIGHT1, OUTPUT);
  pinMode(DIR_RIGHT2, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  digitalWrite(M2ENAB, LOW);   // Задействовать правый мотор
  digitalWrite(DIR_RIGHT1, LOW);
  digitalWrite(DIR_RIGHT2, LOW);
  digitalWrite(M2ENAB, HIGH);

  pinMode(DIR_RUDER1, OUTPUT);      // Мотор подрульки или MID
  pinMode(DIR_RUDER2, OUTPUT);
  pinMode(PWM_RUDER, OUTPUT);
  digitalWrite(DIR_RUDER1, LOW); // Задействовать мотор подрульки
  digitalWrite(DIR_RUDER2, LOW);

  pinMode(BUZZER, OUTPUT);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT_PULLUP);
  pinMode(CH3, INPUT_PULLUP);
  
#ifndef GYVER
pinMode(BUTTON,INPUT);  //Подключается и инициалирируется в объекте КНОПКА
#endif

  attachInterrupt(1, int1, CHANGE);// канал газа
  setCH2();                        // канал руля
  setCH3();                        // канал подрульки

  calibrates();

  mid_up   = map(MID_UP, 1000, 2000, -500, 500);  // найдём значения для включени MID
  mid_down = map(MID_DOWN, 1000, 2000, -500, 500);

  GTCCR = (1 << TSM) | (1 << PSRASY) | (1 << PSRSYNC); // для синхронизации всех таймеров

#ifdef PWM_4000
  TCCR1B = _BV(CS11);  // установить делитель CLK/8
  TCCR2B = _BV(CS21); //  установить делитель CLK/8
#endif

#ifdef PWM_490
  TCCR1B = _BV(CS11) | _BV(CS10); // установить делитель CLK/64
  TCCR2B = _BV(CS22); //  установить делитель CLK/64
#endif

  TCCR1A |= _BV(COM1A1); // connect pin  9 to timer 1 channel A
  TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
  TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A

  GTCCR = 0; // Запускаем синхронно все таймера

  // *** Получить данные о величине раздрая из EEPROM  ***
  uint8_t value = EEPROM.read(ADREPROM); // Получить данные о раздрае
  if (value < 50 || value > 95) {
    EEPROM.write(ADREPROM, razdray);
    //  Serial.println("SAVE-EPROM");
  } else {
    razdray = value;
  }

  EEPROM.get(eeAddress, appa);  // Получить данные калибровки
  if (appa.flagAppa == 88) {
    THROTTLE_UP   = appa.throttleUp;
    THROTTLE_DOWN = appa.throttleDown;

    // Получение данных из епром и их расчет для Рудера и раздрая
    /*    RUDDER_UP     = appa.rudderUp;
        ch2rr = RUDDER_UP - 150;        // Точка включения раздрая справа
        RUDDER_DOWN   = appa.rudderDown;
        ch2rl = RUDDER_DOWN + 150;      // Точка включения раздрая слева
                                        // Верно для RUDDER_UP > RUDDER_DOWN
    */
    RUDDER_UP     = appa.rudderUp;
    RUDDER_DOWN   = appa.rudderDown;
    ch2null = (RUDDER_UP + RUDDER_DOWN) / 2;
    ch2rr = map(RUDDER_UP, 1700, 2000, 1650, 1850);    // Точка включения раздрая справа
    ch2rl = map(RUDDER_DOWN, 1000, 1250, 1150, 1350);  // Точка включения раздрая слева

    CH3_UP        = appa.ch3Up;
    CH3_DOWN      = appa.ch3Down;

    if (value == 95) {
      Serial.print("THROTTLE_UP=");
      Serial.println(THROTTLE_UP);
      Serial.print("THROTTLE_DOWN=");
      Serial.println(THROTTLE_DOWN);
      Serial.print("RUDDER_UP=");
      Serial.println(RUDDER_UP);
      Serial.print("RUDDER_DOWN=");
      Serial.println(RUDDER_DOWN);
      Serial.print("CH3_UP=");
      Serial.println(CH3_UP);
      Serial.print("CH3_DOWN=");
      Serial.println(CH3_DOWN);
    }
  } else {
    THROTTLE_UP   = 2000;
    THROTTLE_DOWN = 1000;
    RUDDER_UP     = 2000;
    RUDDER_DOWN   = 1000;
    CH3_UP        = 2000;
    CH3_DOWN      = 1000;
  }

  // *** Заинициализировать обработчики кнопок и датчиков ***
#ifdef GYVER
#else
  Btn1.init();  // Подключили кнопку
#endif

  Csd1.setup(); // Датчик тока левого двигателя
  Csd2.setup(); // Датчик тока правого двигателя
  Csd3.setup(); // Датчик тока двигателя подруливания
  Upp.setup();  // Датчик напряжения питания (аккумулятора)

} //END SETUP


/***********************************************************************************/

void loop() {
  static uint16_t ch1;
  static uint16_t ch2;
  static uint16_t ch3;
  int outLeft = 0;
  int outRight = 0;
  int outRuder = 0;
  int outMid = 0;
  ch1 = 1500;
  ch2 = 1500;
  ch3 = 1500;
  mill = millis();

#ifdef GYVER
  Btn1.tick();
#else
  Btn1.run();
#endif

  Upp.loop();
  Csd1.loop();
  Csd2.loop();
  Csd3.loop();

  /* Проверяем токовую защиту если опция включена*/
#ifdef ZASHITA
  if (tok1 > IReg * Amper || tok2 > IReg * Amper) {
    // Serial.println("SOS!");
    Icnt++;
    if (Icnt >= SOScounter) {
      setMotor(0, 0, 0);      // Выключить регуляторы
      // Serial.println("SOS! Stop Mashinen");
    } toneSOS();
  } // Если много срабатываний по току - остановка
  // сброс счетка Icnt=0 вызовет продолжение программы
#endif
  /* */

  /***** Установка раздрая *****/
#ifdef GYVER
  if (Btn1.isSingle()) {
    razdray += 5; toneRazdray(); Serial.println("IS RAZDRAY");
    if (razdray > 95) {
      razdray = 50;
    }
  }

  if (Btn1.isHolded()) {
    toneSaveeprom(); EEPROM.write(ADREPROM, razdray);
  }
#else
  if (Btn1.read() == sbClick)
  { // Вызвать процедуру установки раздрая
    razdray += 5; toneRazdray(); Serial.println("IS RAZDRAY");
    if (razdray > 95) { razdray = 50; }
  }
  
  if (Btn1.read() == sbLong)   { // Вызвать процедуру записи в EPROM
    toneSaveeprom(); EEPROM.write(ADREPROM, razdray);
                          }
#endif   
  //[1000..2000]
  if (sharedFlag1) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ch1 = sharedCh1;/*sharedFlag1=0;*/
    }
  }
  // ch1 = sharedCh1;
  // Serial.println(ch1);
  if (sharedFlag2) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ch2 = sharedCh2;/*sharedFlag2=0;*/
    }
  }
  // ch2 = sharedCh2;
  // Serial.println(ch2);
  if (sharedFlag3) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ch3 = sharedCh3;/*sharedFlag2=0;*/
    }
  }
  // ch3 = sharedCh3;
  // Serial.println(ch3);

  //limit[1000..2000]
  ch1 = constrain(ch1, MIN, MAX);
  ch2 = constrain(ch2, MIN, MAX);
  ch3 = constrain(ch3, MIN, MAX);

  //[1000..2000] -> [-500..500]
  ch1 = stopZone(ch1 - MID);
//  ch2 = stopZone(ch2 - MID);
  ch2 = stopZone(ch2 - ch2null);
  ch3 = stopZone(ch3 - MID);

  //mix motors
  outRight = ch1;
  outLeft  = ch1;
  outRuder = ch3;
  outMid   = ch1;

  // if(ch2>1850 || ch2<1150)
  // if (ch2 > (ch2rr - MID)) { //PWM>=1850
    if (ch2 > (ch2rr - ch2null)) { 
    outRight = -(ch1 * (razdray / 5) / 20); // чтобы уложится в 32768 при вычислениях
    if (flagTimeR == 0) {           // первое вхождение в процедуру раздрая
      flagTimeR = 1;
      timeR = millis();
    }
    if (flagTimeR == 1 && millis() - timeR <= DELAY_REVERS) { // второе и последующие
      // вхождения в процедуру раздрая, переключение отрабатываем по таймеру
      outRight = outRight * (millis() - timeR) / DELAY_REVERS; //плавный реверс
    }
  } else {
    if (flagTimeR == 1) {
      flagTimeR = 0;
      timeR = millis();
    }
    if (flagTimeR == 0 && millis() - timeR <= DELAY_REVERS) {
      outRight = outRight * (millis() - timeR) / DELAY_REVERS;
    }
    // обратное переключение
  } // проверить выполнение процедуры при многократном пересечении точки раздрая

  //  int RZR = CH2RR - MID;
  // if (ch2 < (ch2rl - MID)) { //PWM<=1150
  if (ch2 < (ch2rl - ch2null)) {
    outLeft = -(ch1 * (razdray / 5) / 20); //*/ чтобы уложится в 32768 при вычислениях
    if (flagTimeL == 0) { // первое вхождение в процедуру раздрая
      flagTimeL = 1;
      timeL = millis();
    }
    if (flagTimeL == 1 && millis() - timeL <= DELAY_REVERS) { // второе и последующие
      // вхождения в процедуру раздрая, переключение отрабатываем по таймеру
      outLeft = outLeft * (millis() - timeL) / DELAY_REVERS; //плавный реверс
    }
  } else {
    if (flagTimeL == 1 ) {
      flagTimeL = 0;
      timeL = millis();
    }
    if (flagTimeL == 0 && millis() - timeL <= DELAY_REVERS) {
      outLeft = outLeft * (millis() - timeL) / DELAY_REVERS;
    }
  } // проверить выполнение процедуры при многократном пересечении точки раздрая

  //remap RC range to PWM range
  outRight  = map(outRight, -HALF + STOP_ZONE, HALF - STOP_ZONE, -PWM_MAX, PWM_MAX);
  outLeft   = map(outLeft, -HALF + STOP_ZONE, HALF - STOP_ZONE, -PWM_MAX, PWM_MAX);
  outRuder  = map(outRuder, -HALF + STOP_ZONE, HALF - STOP_ZONE, -PWM_MAX, PWM_MAX);
  outMid    = map(outMid,  -HALF + STOP_ZONE, HALF - STOP_ZONE, -PWM_MAX, PWM_MAX);


  //limit to PWM range
  outRight  = constrain(outRight, -PWM_MAX, PWM_MAX);
  outLeft   = constrain(outLeft, -PWM_MAX, PWM_MAX);
  outRuder  = constrain(outRuder, -PWM_MAX, PWM_MAX);
  outMid    = constrain(outMid,  -PWM_MAX, PWM_MAX);

#ifdef MIDS
  setMotor(outLeft, outRight, outMid);
#else
  setMotor(outLeft, outRight, outRuder);
#endif
#ifdef DEBUG 
        Serial.print(outLeft);
        Serial.print(",");
        Serial.println(outRight);
        Serial.print(", - ");
        Serial.println(outRuder);
#endif   
     
}
// END LOOP
// END PROGRAMM
