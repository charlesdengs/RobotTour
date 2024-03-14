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

volatile float velocityAI = 0;
volatile long prevTA = 0;
volatile float velocityBI = 0;
volatile long prevTB = 0;


float v1FiltA = 0;
float v1PrevA = 0;

float v1FiltB = 0;
float v1PrevB = 0;


double deltaTA = 0;
double deltaTB = 0;

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
    delay(1000);
    turnLeft(76);

  }
  
  
  
}

void move(int target)
{
  bool cond1 = false;
  bool cond2 = false;
  prevTA = micros();
  prevTB = micros();
  velocityAI = 0;
  velocityBI = 0;
  v1FiltA = 0;
  v1PrevA = 0;
  v1FiltB = 0;
  v1PrevB = 0;
  eintegralA = 0;
  eintegralB = 0;
  while(true)
  {
    pidA(60);
    pidB(60);
    ATOMIC()
    {
      if(abs(posA*0.95) >= abs(target))
      {
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
      }
      if(abs(posB*1.05) >= abs(target))
      {
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
      }
    }
    Serial.println(posA);
    Serial.print(" ");
    Serial.println(posB);
    if(cond1 || cond2)
    {
      posA = 0;
      posB = 0;
      break;
    }
  }
  delay(1000);
  prevTA = micros();
  prevTB = micros();

}

void moveBack(int target)
{
  bool cond1 = false;
  bool cond2 = false;
  prevTA = micros();
  prevTB = micros();
  velocityAI = 0;
  velocityBI = 0;
  v1FiltA = 0;
  v1PrevA = 0;
  v1FiltB = 0;
  v1PrevB = 0;
  eintegralA = 0;
  eintegralB = 0;
  while(true)
  {
    pidA(-60);
    pidB(-60);
    ATOMIC()
    {
      if(abs(posA*0.95) >= abs(target))
      {
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
      }
      if(abs(posB*1.05) >= abs(target))
      {
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
      }
    }
    Serial.println(posA);
    Serial.print(" ");
    Serial.println(posB);
    if(cond1 || cond2)
    {
      posA = 0;
      posB = 0;
      break;
    }
  }
  delay(1000);
  prevTA = micros();
  prevTB = micros();

}

void turnLeft(int target)
{
  bool cond1 = false;
  bool cond2 = false;
  prevTA = micros();
  prevTB = micros();
  velocityAI = 0;
  velocityBI = 0;
  v1FiltA = 0;
  v1PrevA = 0;
  v1FiltB = 0;
  v1PrevB = 0;
  eintegralA = 0;
  eintegralB = 0;

  while(true)
  {
    pidA(-60);
    pidB(60);
    ATOMIC()
    {
      if(abs(posA*0.95) >= abs(target))
      {
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
      }
      if(abs(posB*1.05) >= abs(target))
      {
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
      }
    }
    Serial.println(posA);
    Serial.print(" ");
    Serial.println(posB);
    if(cond1 || cond2)
    {
      posA = 0;
      posB = 0;
      break;
    }
  }
  delay(1000);
  prevTA = micros();
  prevTB = micros();

}

void turnRight(int target)
{
  bool cond1 = false;
  bool cond2 = false;
  prevTA = micros();
  prevTB = micros();
  velocityAI = 0;
  velocityBI = 0;
  v1FiltA = 0;
  v1PrevA = 0;
  v1FiltB = 0;
  v1PrevB = 0;
  eintegralA = 0;
  eintegralB = 0;

  while(true)
  {
    pidA(60);
    pidB(-60);
    ATOMIC()
    {
      if(abs(posA*0.95) >= abs(target))
      {
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
      }
      if(abs(posB*1.05) >= abs(target))
      {
        pidB(0);
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
        pidA(0);
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
      }
    }
    Serial.println(posA);
    Serial.print(" ");
    Serial.println(posB);
    if(cond1 || cond2)
    {
      posA = 0;
      posB = 0;
      break;
    }
  }
  delay(1000);
  prevTA = micros();
  prevTB = micros();

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
  deltaTA = ((double) (currT - prevTA))/1.0e6;
  velocityAI = increment/deltaTA;
  prevTA = currT;
}

void readEncoderB() {
  int b = digitalRead(ENCB2);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  posB += increment;

  long currT = micros();
  deltaTB = ((double) (currT - prevTB))/1.0e6;
  velocityBI = increment/deltaTB;
  prevTB = currT;
}

void pidA(float speed) {
  int pos = 0;
  float velocity = 0;
  ATOMIC()
  {
    pos = posA;
    velocity = velocityAI;
  }

  v1FiltA = 0.854*v1FiltA + .07281*velocity + .0728*v1PrevA;
  v1PrevA = velocity;

  float target = speed;
  float kp = 16;
  float ki = .8;
  float e = (v1FiltA/145.0*60.0) - target;
  eintegralA = eintegralA + e*deltaTA;
  float u = kp*e + ki*eintegralA;


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
  }
  //Serial.print(v1FiltB/145.0*60.0);
  //Serial.println();
  delay(1);
}

void pidB(float speed) {
  int pos = 0;
  float velocity = 0;
  ATOMIC()
  {
    pos = posB;
    velocity = velocityBI;
  }

  v1FiltB = 0.854*v1FiltB + .07281*velocity + .0728*v1PrevB;
  v1PrevB = velocity;

  float target = speed;
  float kp = 16;
  float ki = .8;
  float e =  target - (v1FiltB/145.0*60.0);
  eintegralB = eintegralB + e*deltaTB;
  float u = kp*e + ki*eintegralB;


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
    setMotor(dir, pwr, PWMB, BIN1, BIN2);
  }
  //Serial.print(v1FiltB/145.0*60.0);
  //Serial.println();
  delay(1);
}
