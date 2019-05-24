

/*class Servo {
  public:
    Servo(int, int, int, int);
    void tick();
    void set(float);
    void set_byte(char);
    void set_absolute(int);
    int start;
    int stop;
    int width;
    int current;
  private:
    int t;
    int pin;
    bool is_on;
};*/

class Encoder {
  public:
    Encoder(int, int);
    void reverse();
    void tick();
    float get_vel(float);
    int position;
    int error;
    int inv_vel;
    bool positive;
  private:
    int Apin;
    int Bpin;
    bool Astate;
    bool Bstate;
    int count;
};

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


class Bridge {
  public:
    Bridge(int, int);
    void set(float);
    float scale;
  private:
    int PWM;
    int A;
};

