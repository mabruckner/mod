#include<EEPROM.h>

/*bool a = false;
bool b = false;
int counts = 0;*/
float sp = 1.0;
int stage = 0;
int tar = 144;
float t =0;

class Encoder {
  public:
    Encoder(int, int);
    void reverse();
    void tick();
    int position;
    int error;
  private:
    int Apin;
    int Bpin;
    bool Astate;
    bool Bstate;
};

Encoder::Encoder(int a, int b) {
  pinMode(a, INPUT);
  pinMode(b, INPUT);
  Apin = a;
  Bpin = b;
  position = 0;
  error = 0;
  Astate = digitalRead(a);
  Bstate = digitalRead(b);
}

void Encoder::reverse() {
  int Tpin = Apin;
  bool Tstate = Astate;
  Apin = Bpin;
  Astate = Bstate;
  Bpin = Tpin;
  Bstate = Tstate;
}

void Encoder::tick() {
  bool An = digitalRead(Apin);
  bool Bn = digitalRead(Bpin);
  if(An != Astate) {
    if(An == Bstate) {
      position += 1;
    } else {
      position -= 1;
    }
  }
  if(Bn != Bstate) {
    if(Bn != Astate) {
      position += 1;
    } else {
      position -= 1;
    }
  }
  if(An != Astate && Bn != Bstate) {
    error += 1;
  }
  Astate = An;
  Bstate = Bn;
}

class PID {
  public:
    PID(float, float, float);
    float output(float, float);
    void reset();
    float target;
  private:
    float P;
    float I;
    float D;
    float previous;
    float acc;
    bool init;
};

PID::PID(float p, float i, float d) {
  P = p;
  I = i;
  D = d;
  target = 0.0;
  reset();
}

void PID::reset() {
  previous = 0.0;
  acc = 0.0;
  init = true;
}

float PID::output(float inp, float delta) {
  float diff = target - inp;
  if (init) {
    init = false;
    previous = diff;
    return 0.0;
  }
  float derivative = (diff - previous) / delta;
  acc += delta * (diff + previous) / 2.0; // trapezoidal integration
  previous = diff;
  return (diff * P + acc * I + derivative * D);
}


class Bridge {
  public:
    Bridge(int, int, int);
    void set(float);
    float scale;
  private:
    int PWM;
    int A;
    int B;
};

Bridge::Bridge(int pwm, int a, int b) {
  PWM = pwm;
  A = a;
  B = b;
  pinMode(PWM, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  scale = 1.0;
}

void Bridge::set(float value) {
  float safe = scale * min(fabs(value), 1.0);
  analogWrite(PWM, (int)(255.0*safe));
  if(value > 0.0) {
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
  }else if (value<0.0) {
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
  }else {
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
  }
}


Encoder encA(16, 10);
Encoder encB(15, 14);
Bridge bridgeA(3, 4, 5);
Bridge bridgeB(6, 7, 8);
PID pidA(4.0, 5.0, 0.5);
PID pidB(4.0, 5.0, 0.5); 

int switches[4] = {A0, A1, A2, A3};

int ab = A2;
int AB = A3;

bool enabled = false;

class Parser {
  public:
    Parser();
    void push(char);
  private:
    char command;
    int len;
    char buf[16];
};

Parser::Parser() {
  command = ' ';
  len = 0;
}

void Parser::push(char val) {
  if(len == 0) {
    command = val;
    switch (val) {
      case 'm': // move to
        len = 1;
        break;
      case 'p': // get position
        break;
      case 'h': // home
        break;
      case 'e': // enable
        enabled = true;
        break;
      case 'd': // disable
        enabled = false;
        break;
      default:
        break;
    }
  }
  if(len != 0) {
    buf[len-1] = val;
    len++;
    switch (command) {
      case 'm':
        if(len > 8) {
          pidA.target = *((float*) buf);
          pidB.target = *((float*) (buf+4));
          len = 0;
        }
        break;
      default:
        len = 0;
    }
  }
}

Parser parse();

void setup() {
  Serial.begin(9600);
  for(int i=0; i<4; i++){
    pinMode(switches[i], INPUT);
  }
  bridgeA.scale = 0.2;
  bridgeB.scale = 0.2;
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
  Serial.println(TCCR1B);
  Serial.println(TCCR1B);
  Serial.println(TCCR1B);
  //TCCR1B = (TCCR1B & 0xf8) | 0x3;
  //TCCR2B = (TCCR2B & 0xf8) | 0x02;
  
  //testmotor();
  //home();
  //status();
  
  /*bridgeB.set(0.90);
  delay(200);
  bridgeB.set(0.0);*/
}

SIGNAL(TIMER0_COMPA_vect) {
  encA.tick();
  encB.tick();
}

void testmotor() {
  Serial.println(encA.position);
  bridgeA.set(0.8);
  delay(200);
  bridgeA.set(0.0);
  Serial.println(encA.position);
  delay(1000);
  Serial.println(encB.position);
  bridgeB.set(0.8);
  delay(200);
  bridgeB.set(0.0);
  Serial.println(encB.position);
  delay(3000);
}

void home() {
  for(int i=0;i<4;i++){
    Serial.print(i);
    Serial.println(digitalRead(i));
  }
  bridgeA.set(0.0);
  bridgeB.set(0.0);
  for(int i=5; i>0; i--){
    Serial.println(i);
    delay(1000);
  }
  bridgeA.set(0.8);
  bridgeB.set(0.8);
  bool done = false;
  while(!done) {
    for(int i=0;i<4;i++){
      if(digitalRead(switches[i])){
        done=true;
      }
    }
  }
  encA.position = (encA.position-encB.position)/2;
  encB.position = -encA.position;
  bridgeA.set(0.0);
  bridgeB.set(0.0);
    for(int i=0;i<4;i++){
      if(digitalRead(switches[i])){
        Serial.println(i);
      }
    }
    delay(50);
  bridgeA.set(-0.8);
  bridgeB.set(-0.8);
    delay(150);
  done = false;
  while(!done) {
    for(int i=0;i<4;i++){
      if(digitalRead(switches[i])){
        done=true;
      }
    }
  }
  int width = encA.position + encB.position;
  int ta = (3*encA.position - encB.position)/4;
  encB.position = (3*encB.position - encA.position)/4;
  encA.position = ta;
  bridgeA.set(0.0);
  bridgeB.set(0.0);
  Serial.println(encA.position);
  Serial.println(encB.position);
  delay(50);
  bridgeA.set(0.8);
  bridgeB.set(0.8);
  delay(150);
  bridgeA.set(0.0);
  bridgeB.set(0.0);

  pidA.target = 0.0;
  pidB.target = 0.0;
  while(true) {
    delay(10);
    float pA = ((float) encA.position) / 24.0;
    float pB = ((float) encB.position) / 24.0;
    if(fabs(pA) < 0.5 && fabs(pB) < 0.5) {
      break;
    }
    float controlA = pidA.output(pA, 0.01);
    float controlB = pidB.output(pB, 0.01);
    bridgeA.set(controlA);
    bridgeB.set(controlB);
  }
  bridgeA.set(0.0);
  bridgeB.set(0.0);
  delay(1000);
  bridgeA.set(0.8);
  bridgeB.set(-0.8);
  done = false;
  while(!done) {
    for(int i=0;i<4;i++){
      if(digitalRead(switches[i])){
        done=true;
      }
    }
  }
  encA.position = (encA.position + encB.position)/2;
  encB.position = encA.position;
  bridgeA.set(0.0);
  bridgeB.set(0.0);
  Serial.println(encA.position);
  Serial.println(encB.position);
  delay(50);
  bridgeA.set(-0.8);
  bridgeB.set(0.8);
  delay(150);
  done = false;
  while(!done) {
    for(int i=0;i<4;i++){
      if(digitalRead(switches[i])){
        done=true;
      }
    }
  }
  int tA = (3*encA.position + encB.position)/4;
  encB.position = (encA.position + 3*encB.position)/4;
  encA.position = tA;
  bridgeA.set(0.0);
  bridgeB.set(0.0);
  delay(50);
  bridgeA.set(0.8);
  bridgeB.set(-0.8);
  delay(150);
  bridgeA.set(0.0);
  bridgeB.set(0.0);

  Serial.println("DONE");
  Serial.println(encA.position);
  Serial.println(encB.position);
  
  /*
  Serial.println(encA.position);
  Serial.println(encB.position);
  Serial.println(encA.error);
  Serial.println("BEGIN");
  bridgeA.set(0.8);
  byte tick = 0;
  bool done = false;
  while(!done) {
    for(int i=0;i<4;i++){
      if(digitalRead(i)){
        done=true;
      }
    }
    tick ++;
  }
  encA.position = 0;
  bridgeA.set(-0.0);
  Serial.println("END");
  Serial.println(encA.position);
  Serial.println(encA.error);*/
}

void status() {
  Serial.print("status ");
  Serial.print(encA.position);
  Serial.print(" ");
  Serial.print(encA.error);
  Serial.print(" ");
  Serial.print(encB.position);
  Serial.print(" ");
  Serial.print(encB.error);
  for(int i=0;i<4;i++){
    Serial.print(" ");
    Serial.print(digitalRead(switches[i]));
  }
  Serial.println("");
}

void testloop() {
  Serial.print(encA.position);
  Serial.print("_");
  Serial.print(encA.error);
  Serial.print("_");
  Serial.print(encB.position);
  Serial.print("_");
  Serial.println(encB.error);
  delay(100);
}
void loop() {
  delay(10);
  float delta = 0.01;
  float pA = ((float) encA.position) / 24.0;
  float pB = ((float) encB.position) / 24.0;
  t += delta;
  float controlA = pidA.output(pA, delta);
  float controlB = pidB.output(pB, delta);
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
  
  }
  
  /*if((int)(t*100.0) % 20 == 0){
    Serial.println(encA.error);
  }
  for(int i=0;i<4;i++){
    Serial.println(digitalRead(switches[i]));
  }*/
  // put your main code here, to run repeatedly:

}
