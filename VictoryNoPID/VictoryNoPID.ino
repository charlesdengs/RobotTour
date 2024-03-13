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

float currT = 0;
float endT = 0;

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
  if(digitalRead(BUTTON) == 0)
  {
    digitalWrite(STANDBY, HIGH);
    move(400);
    delay(1000);
    move(1000);
  }
}

void move(int target)
{
  bool cond1 = false;
  bool cond2 = false;
  while(true)
  {
    setMotor(-1, 100, PWMA, AIN1, AIN2);
    setMotor(1, 100, PWMB, BIN1, BIN2);
    ATOMIC()
    {
      if(abs(posA*1.054) >= abs(target))
      {
        setMotor(0,0,PWMA,AIN1,AIN2);
        cond1 = true;
      }
      if(abs(posB*1.056) >= abs(target))
      {
        setMotor(0,0,PWMB, BIN1, BIN2);
        cond2 = true;
      }
    }
    Serial.print(posA);
    Serial.print(" ");
    Serial.println(posB);
    if(cond1 && cond2)
    {
      posA = 0;
      posB = 0;
      break;
    }
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