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



Encoder encA(16, 10);
Encoder encB(15, 14);

int switches[4] = {A0, A1, A2, A3};

int ab = A2;
int AB = A3;

bool enabled = false;


void setup() {
  Serial.begin(9600);
  for(int i=0; i<4; i++){
    pinMode(switches[i], INPUT);
  }
  
  OCR0A = 0xAF;
  //OCR0A = 0x5;
  TIMSK0 |= _BV(OCIE0A);

  Serial.println(TCCR1B);
  TCCR1B = (TCCR1B & 0xf8) | 0x3;

}

SIGNAL(TIMER0_COMPA_vect) {
  encA.tick();
  encB.tick();
}

void loop() {
  Serial.print(encA.position);
  Serial.print("_");
  Serial.print(encA.error);
  Serial.print("_");
  Serial.print(encB.position);
  Serial.print("_");
  Serial.println(encB.error);
  delay(100);
}
