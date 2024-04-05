#include <SimplyAtomic.h>
#include "ICM_20948.h"
ICM_20948_I2C myICM;
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
int encoderRange = 145;
//turn = 84
//move to account for offset of rotational axis after moving axis of rotation to midpoin = 230
//move without turn = 257
//move without turn actual = 268
//move with turn = 300
float caliX = 0;
float caliZ = 0;

volatile int posA = 0;
volatile int posB = 0;
int posAPrev = 0;
int posBPrev = 0;

long prevT = 0;

float eprevA = 0;
float eprevB = 0;
float eintegralA = 0;
float eintegralB = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(Wire, 1);


    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  myICM.swReset();
  delay(250);
  myICM.sleep(false);
  myICM.lowPower(false);
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
  

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
  Serial.println(posA);
  if(digitalRead(BUTTON) == 0)
  {
    calibrate();
    delay(1000);
    //move(305, "FORWARD");
    move(88, "LEFT");
    //move(230, "FORWARD");
    //move(268,"FORWARD");
    //move(88,"RIGHT");
    
    Serial.println("done");
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

void move(float target, String dir)
{
  posA = 0;
  posB = 0;
  int pos = 0;
  float goal = 0;
  float currentT = 0;
  float previousT = millis();
  float prevTarget = 0;
  float timer = 2000.0 + millis();
  prevT = micros();
  while(true)
  {
    currentT = millis();
    if(currentT >= previousT + 250 && abs(goal)<abs(target))
    {
      goal += target/4.0;
      previousT += 250;
    }
    
    if (dir == "FORWARD") {
      pidA(target);
      pidB(target);
    } else if (dir == "LEFT") {
      pidA(-target);
      pidB(target);
    } else if (dir == "RIGHT") {
      pidA(target);
      pidB(-target);
    } else if (dir == "BACK") {
      pidA(-target);
      pidB(-target);
    }
    myICM.getAGMT();
    if(dir == "FORWARD" || dir == "BACK")
    {
      if(moving(&myICM) && goal == target)
      {
        setMotor(0,0,PWMB, BIN1, BIN2);
        setMotor(0,0,PWMA, AIN1, AIN2);
        prevTarget = target;
        ATOMIC()
        {
          posAPrev = posA;
          posBPrev = posB;
        }
        Serial.println(timer - millis());
        delay(timer - millis());
        break;
      }
    }
    else if(dir == "LEFT" || dir == "RIGHT")
    {
      if(turning(&myICM) && goal == target)
      {
        setMotor(0,0,PWMB, BIN1, BIN2);
        setMotor(0,0,PWMA, AIN1, AIN2);
        /*prevTarget = target;
        ATOMIC()
        {
          posAPrev = posA;
          posBPrev = posB;
        }*/
        Serial.println(timer - millis());
        delay(timer - millis());
        break;
      }
    }
  }
}

bool moving(ICM_20948_I2C *sensor)
{
  if(fabs(sensor->accX()) <= caliX)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool turning(ICM_20948_I2C *sensor)
{
  if(fabs(sensor->gyrZ()) <= fabs(caliZ))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void pidA(float target) {
  
  float kp = 2.0;
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

  Serial.print("A: ");
  Serial.println(posA);
}

void pidB(float target) {
  
  float kp = 4.9;
  float kd = .04;
  float ki = 0;

  //Shared by both pids
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/1.0e6;
  prevT = currT;

  //error
  int e = target - posB;

  //derivative
  float dedt = (e - eprevB) / deltaT;

  //integral
  eintegralB = eintegralB + e*deltaT;

  //signal
  float u = kp*e + kd*dedt + ki*eintegralB;

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

  setMotor(dir, pwr, PWMB, BIN1, BIN2);

  eprevB = e;

  Serial.print("B: ");
  Serial.println(posB);
}

void calibrate()
{
  float totalX = 0;
  float totalZ = 0;
  for(int i = 0; i<1000; i++)
  {
    myICM.getAGMT();
    totalX += testX(&myICM);
    totalZ += testZ(&myICM);
  }

  caliX = totalX/1000;
  caliZ = totalZ/1000;
  Serial.println(caliX);
  Serial.println(caliZ);
}

float testX(ICM_20948_I2C *sensor)
{
  return sensor->accX();
}

float testZ(ICM_20948_I2C *sensor)
{
  return sensor->gyrZ();
}