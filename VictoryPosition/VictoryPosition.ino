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

long prevT = 0;

float eprevA = 0;
float eprevB = 0;
float eintegralA = 0;
float eintegralB = 0;


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

void loop() {
  digitalWrite(STANDBY, HIGH);
  if(digitalRead(BUTTON) == 0)
  {
    move(255);
    delay(1000);
    move(0);
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
  } else {
    increment = -1;
  }
  posA += increment;
}

void readEncoderB() {
  int b = digitalRead(ENCB2);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  } else {
    increment = -1;
  }
  posB += increment;
}

void move(float target)
{
  int pos = 0;
  prevT = micros();
  while(true)
  {
    
    pidA(target);
    ATOMIC()
    {
      pos = posA;
    }
    if(pos == target-1)
    {
      break;
    }
  }
}

void pidA(float target) {
  
  float kp = 1.95;
  float kd = .04;
  float ki = 0;

  //Shared by both pids
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/1.0e6;
  prevT = currT;

  //error
  int e = posA - target;

  //derivative
  float dedt = (e - eprevA) / deltaT;

  //integral
  eintegralA = eintegralA + e*deltaT;

  //signal
  float u = kp*e + kd*dedt + ki*eintegralA;

  float pwr = fabs(u);
  if(pwr > 255)
  {
    pwr = 255;
  }

  int dir = 1;
  if(u < 0 )
  {
    dir = -1;
  }

  setMotor(dir, pwr, PWMA, AIN1, AIN2);

  eprevA = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.println(posA);
}