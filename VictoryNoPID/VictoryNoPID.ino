#include <SimplyAtomic.h>
//A is Left Motor
//B is Right Motor
#define BUTTON 21

#define PWMA 17
#define AIN1 16
#define AIN2 19
#define STANDBY 4
#define PWMB 14
#define BIN1 32
#define BIN2 15

#define ENCA1 23
#define ENCA2 18
#define ENCB1 35
#define ENCB2 36

volatile int posA = 0;
volatile int posB = 0;

int posPrev = 0;
long prevT = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

double deltaT = 0;
float eintegral = 0;

void setup() {
  Serial.begin(115200);
  //BUTTON
  pinMode(BUTTON, INPUT_PULLUP);

  //MOTOR A
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  //MOTOR B
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  //STANDBY PIN
  pinMode(STANDBY, OUTPUT);
  
  pinMode(ENCA1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoderB, RISING);
}

float currentTime = 0;
float endTime = 400.0;
void loop() {
  digitalWrite(STANDBY, HIGH);
  currentTime = micros()/1.0e6;
  Serial.print("Time: ");
  Serial.println(currentTime);
  if(currentTime >= endTime)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, HIGH);
  }
  else
  {
    pid(60);
    
  }
  
  
}

void setMotor(int dir, int pwm, int pwmPin, int mot1, int mot2) {
  if (dir == 1) {
    digitalWrite(mot1, HIGH);
    digitalWrite(mot2, LOW);
  } else if (dir == -1) {
    digitalWrite(mot1, LOW);
    digitalWrite(mot2, HIGH);
  } else {
    digitalWrite(mot1, HIGH);
    digitalWrite(mot2, HIGH);
  }
  analogWrite(pwmPin, pwm);
}

void readEncoderA() {
  int b = digitalRead(ENCA2);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  posA += increment;

  long currT = micros();
  deltaT = ((double) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}

void readEncoderB() {
  int b = digitalRead(ENCB2);
  if (b>0) {
    posB++;
  }
  else {
    posB--;
  }
}
void pid(float speed) {
  int pos = 0;
  float velocity = 0;
  ATOMIC()
  {
    pos = posA;
    velocity = velocity_i;
  }

  v1Filt = 0.854*v1Filt + .07281*velocity + .0728*v1Prev;
  v1Filt = v1Filt;
  v1Prev = velocity;

  float target = speed;
  float kp = 10;
  float ki = 20;
  float e = (v1Filt/145.0*60.0) - target;
  eintegral = eintegral + e*deltaT;
  float u = kp*e + ki*eintegral;

  int dir = 1;
  if (u<0)
  {
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr>255)
  {
    pwr = 255;
  }

  if(target != 0)
  {
    setMotor(dir, pwr, PWMA, AIN1, AIN2);
    setMotor(-dir, pwr, PWMB, BIN1, BIN2);
  }
  Serial.print(velocity/145.0*60.0);
  Serial.print(" ");
  Serial.print(v1Filt/145.0*60.0);
  Serial.println();
  delay(1);
}
