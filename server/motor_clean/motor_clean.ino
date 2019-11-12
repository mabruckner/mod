 #include<EEPROM.h>
#include<Servo.h>
#include "types.h"

/*bool a = false;
bool b = false;
int counts = 0;*/
float sp = 1.0;
int stage = 0;
int tar = 144;
float t =0;
int ticks=0;

Encoder encA(16, 10);
Encoder encB(15, 14);
Bridge bridgeA(3, 4);
Bridge bridgeB(6, 7);
Servo servo;
//Servo servo(5, 125, 6, 12);
//PID pidA(4.0, 5.0, 0.5);
//PID pidB(4.0, 5.0, 0.5); 
PID pidA(2.0, 0.5, 0.0, 2.0);
PID pidB(0.5, 0.5, 0.0, 2.0); 

int switches[4] = {A0, A1, A2, A3};

int ab = A2;
int AB = A3;

bool enabled = false;

void setup() {
    servo.attach(5);
    servo.write(30);
  Serial.begin(9600, SERIAL_8E2);
  for(int i=0; i<4; i++){
    pinMode(switches[i], INPUT);
  }
  bridgeA.scale = 0.4;
  bridgeB.scale = 0.3;
  pidA.target = 0.0;
  pidB.target = 0.0;

  OCR0A = 0xAF;
  //OCR0A = 0x5;
  TIMSK0 |= _BV(OCIE0A);

  bridgeA.set(0.0);
  bridgeB.set(0.0);
  for(int i=5; i>0; i--){
    Serial.println(i);
    delay(1000);
  }
  Serial.println("READY");
  //TCCR1B = (TCCR1B & 0xf8) | 0x3;
  //TCCR2B = (TCCR2B & 0xf8) | 0x02;
  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000001; ?? do we need this?
  TCCR4B = TCCR4B & B11111000 | B00000010; // for PWM frequency of 31372.55 Hz

}


SIGNAL(TIMER0_COMPA_vect) {
  ticks += 1;
  //servo.tick();
 
  if((ticks & 0xf) == 0) {
    encA.tick();
  }else if((ticks & 0xf)== 0x8) {
    encB.tick();
  }
}

void status() {
  int s = ticks;
  Serial.print("status ");
  Serial.print(encA.position);
  Serial.print(" ");
  Serial.print(encA.error);
  Serial.print(" ");
  Serial.print(encA.get_signed_inv_vel());
  Serial.print(" ");
  Serial.print(encB.position);
  Serial.print(" ");
  Serial.print(encB.error);
  Serial.print(" ");
  Serial.print(encB.get_signed_inv_vel());
  for(int i=0;i<4;i++){
    Serial.print(" ");
    Serial.print(digitalRead(switches[i]));
  }
  Serial.println("");
  Serial.println(ticks - s);
}

void testloop() {
  Serial.print(encA.position);
  Serial.print("_");
  Serial.print(encA.error);
  Serial.print("_");
  Serial.print(encB.position);
  Serial.print("_");
  Serial.println(encB.error);
  Serial.println(digitalRead(16));
  Serial.println(digitalRead(10));
  Serial.println(ticks);
  delay(1000);
}

int prev = 0;
void loop() {
  delay(1000);
  int diff = ticks - prev;
  prev = ticks;
  float delta = ((float) diff) / 62500.0;
  float pA = ((float) encA.position) / 24.0;
  float vA = encA.get_vel(62500.0 / 16) / 24.0;
  float pB = ((float) encB.position) / 24.0;
  float vB = encB.get_vel(62500.0 / 16) / 24.0;
  t += delta;
  float controlA = pidA.output_velinput(pA, vA, delta);
  float controlB = pidB.output_velinput(pB, vB, delta);
  bridgeA.set(controlA);
  bridgeB.set(controlB);
  
  while(Serial.available()) {
    char s = Serial.read();
    if(s == 'm') {
      long a = Serial.parseInt();
      long b = Serial.parseInt();
      pidA.target = ((float) a) / 24.0;
      pidB.target = ((float) b) / 24.0;
    }
    if(s == 'a') {
      long a = Serial.parseInt();
      pidA.target = ((float) a) / 24.0;
    }
    if(s == 'b') {
      long b = Serial.parseInt();
      pidB.target = ((float) b) / 24.0;
    }
    if(s == 's') {
      status();
    }
    if(s == 'i') {
      Serial.print("id ");
      Serial.print(EEPROM.read(0), DEC);
      Serial.println();
    }
    if(s == 'd') {
      byte val = (byte)Serial.parseInt();
      EEPROM.write(0, val);
    }
    if(s == 'x') {
      byte val = (byte)Serial.parseInt();
      servo.write((int)val);
    }
  }
}
