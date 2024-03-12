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

void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTON, INPUT_PULLUP);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STANDBY, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(BUTTON) == 0)
  {
    delay(1000);
    Serial.println("hello");
    move();
    delay(1000);
  }
  stop();
}

void move()
{
  digitalWrite(STANDBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);

  Serial.println("hello");
}
void stop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
