/*
 * Программа сервотестера для ARDUINO NANO
 * 28.10.2018 - сделано ядро измерения импульса по 8 пину
 * исключительно для 1-го таймера для входа ICP
 * подключение библиотеки Servo.h - не работает
 * 
 */
 #define freq 16
 
 #include <Servo.h>
 Servo myservo;
 int pos = 0;
 
 unsigned long res = 0;
 unsigned int pwm =0;
 float disp_res = 0;
 
void setup() {
Serial.begin(9600);
myservo.attach(9);
pinMode (8,INPUT);
/* Генератор
pinMode(11,OUTPUT);
TCCR2A=(1<<COM2A1)|(1<<COM2A0)|(1<<WGM20);
TCCR2B=1<<CS20;
TIMSK2=0;
OCR2A=1;
*/

}

void loop() {

res = (asm_func());
pwm = res/freq;
disp_res = res/freq;
gs_09();
Serial.println(pwm);
Serial.println(disp_res);
}


uint16_t asm_func(){
uint16_t data;
asm volatile (         
"cli"                     "\n\t" // дабы никто не мешал
"sts 0x80,__zero_reg__"   "\n\t" // TCCR1A=0
"ldi r17,0x41"            "\n\t" // RISING ICP detect , start timer1
"sts 0x81,r17"            "\n\t" // TCCR1B= (1<<ICES1)|(1<<CS10)
"sbic 0x16,5"             "\n\t" // TIFR1& (1<<ICF1) Да?
"sbi 0x16,5"              "\n\t" // тогда clear ICF1
"wait_begin:"             "\n\t" // ждём флага ICF 
"sbis 0x16,5"             "\n\t" // TIFR1& (1<<ICF1) Да?
"rjmp wait_begin"         "\n\t" //пропускаем ход
"lds r22, 0x86"           "\n\t" //сохранить ICR1L  
"lds r23, 0x87"           "\n\t" //сохранить ICR1H
"sbi 0x16,5"              "\n\t" //clear ICF1
"ldi r19,0x01"            "\n\t" // FALLING ICP detect
"sts 0x81,r19"            "\n\t" // TCCR1B=(0<<ICES1)|(1<<CS10)
"wait_end:"               "\n\t" // ждём флага ICF 
"sbis 0x16,5"             "\n\t" // TIFR1& (1<<ICF1) Да?
"rjmp wait_end"           "\n\t" // пропускаем ход
"lds r24, 0x86"           "\n\t" //save ICR1L
"lds r25, 0x87"           "\n\t" //save ICR1H
"sub r24, r22"            "\n\t" //ICR1L_new-ICR1L_old
"sbc r25, r23"            "\n\t" //ICR1H_new-ICR1H_old
"sei"                     "\n\t"
: "=w" (data)
:
: 
);
return data;
}

void gs_09() {
 // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pwm);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
 // }
 // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
 //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
 //   delay(15);                       // waits 15ms for the servo to reach the position
  }
