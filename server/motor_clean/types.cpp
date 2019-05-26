#include "types.h"
#include "Arduino.h"

/*Servo::Servo(int s_pin, int s_width, int s_start, int s_stop) {
  pinMode(pin, INPUT);
  pin = s_pin;
  width = s_width;
  start = s_start;
  stop = s_stop;
  is_on = false;
  current = start;
  t = 0;
}

void Servo::tick() {
  t += 1;
  if(t > width) {
    t = 0;
    digitalWrite(pin, HIGH);
    is_on = true;
  }
  if(is_on && t > current) {
    digitalWrite(pin, LOW);
    is_on = false;
  }
}

void Servo::set(float value) {
  current = (int)(value*(float)stop + (1.0-value)*(float)start);
}

void Servo::set_byte(char value) {
  current = start + ((stop-start)*(int)value)>>8;
}

void Servo::set_absolute(int value) {
  current = value;
}*/



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
  if( count < 0x4000 ) {
      count += 1;
      if(inv_vel < count) {
          inv_vel = count;
      }
  }
  if(An != Astate) {
    if(An == Bstate) {
      position += 1;
      positive = true;
    } else {
      position -= 1;
      positive = false;
    }
  }
  if(Bn != Bstate) {
    if(Bn != Astate) {
      position += 1;
      positive = true;
    } else {
      position -= 1;
      positive = false;
    }
  }
  if(An != Astate || Bn != Bstate) {
      inv_vel = count;
      count = 0;
  }
  if(An != Astate && Bn != Bstate) {
    error += 1;
  }
  Astate = An;
  Bstate = Bn;
}

float Encoder::get_vel(float ticks_per_second) {
    if(inv_vel == 0x4000) {
        return 0.0;
    } else {
        float mag = ticks_per_second / (float)inv_vel;
        if(!positive) {
            mag *= -1.0;
        }
        return mag;
    }
}

int Encoder::get_signed_inv_vel() {
    if (positive) {
        return inv_vel;
    } else {
        return -inv_vel;
    }
}

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



Bridge::Bridge(int pwm, int a) {
  PWM = pwm;
  A = a;
  pinMode(PWM, OUTPUT);
  pinMode(A, OUTPUT);
  scale = 1.0;
}

void Bridge::set(float value) {
  float safe = scale * min(fabs(value), 1.0);
  analogWrite(PWM, (int)(255.0*safe));
  if(value > 0.0) {
    digitalWrite(A, HIGH);
  }else if (value<0.0) { 
    digitalWrite(A, LOW);
  }
}


