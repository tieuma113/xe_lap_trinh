#include <Arduino.h>

#define sensor1 A4
#define sensor2 A1
#define sensor3 A2
#define sensor4 A3
#define sensor5 A0

#define motor1b 2
#define motor1f 3
#define motor2b 4
#define motor2f 5

#define en2     9
#define en1     11

#define Kp      20
#define Ki      0
#define Kd      5

#define BASE_SPEED  85
#define BALANCE     0.87

int error = 0,last_error = 0;
int P = 50, I = 0, D = 0;
int mode = 0;
uint8_t lastState = 0;

int PID();
int check();
void run();
void motor1Forward(int speed);
void motor2Forward(int speed);
void motor1Stop();
void motor2Stop();

void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(motor1f, OUTPUT);
  pinMode(motor1b, OUTPUT);
  pinMode(motor2f, OUTPUT);
  pinMode(motor2b, OUTPUT);

  pinMode(en2, OUTPUT);
  pinMode(en1, OUTPUT);

  Serial.begin(9600);
  lastState = digitalRead(sensor3);
}

void loop() {
  // put your main code here, to run repeatedly:
  run();
  // Serial.println(mode);
  //Serial.println(error);
  // motor2Forward(100);
  // motor1Forward(0.87*100);
  // uint8_t sen1 = digitalRead(sensor1);
  // uint8_t sen2 = digitalRead(sensor2);
  // uint8_t sen3 = digitalRead(sensor3);
  // uint8_t sen4 = digitalRead(sensor4);
  // uint8_t sen5 = digitalRead(sensor5);
  //  Serial.println(sen1);
  // Serial.println(sen2);
  // Serial.println(sen3);
  // Serial.println(sen4);
  // Serial.println(sen5);
  // Serial.println("===============");
  // delay(1000);
  // motor1Forward(BASE_SPEED1);
  // motor2Forward(BASE_SPEED2);
}

void motor1Forward(int speed) {
  analogWrite(en1, speed);
  digitalWrite(motor1f,HIGH);
  digitalWrite(motor1b,LOW);
}

void motor2Forward(int speed) {
  analogWrite(en2, speed);
  digitalWrite(motor2f,HIGH);
  digitalWrite(motor2b,LOW);
}

void motor1Stop(){
  digitalWrite(motor1f,LOW);
  digitalWrite(motor1b,LOW);
}

void motor2Stop(){
  digitalWrite(motor2f,LOW);
  digitalWrite(motor2b,LOW);
}

void rotateLeft(){
  analogWrite(en2, 75);
  digitalWrite(motor2f,HIGH);
  digitalWrite(motor2b,LOW);
  digitalWrite(motor1f,LOW);
  digitalWrite(motor1b,LOW);
}

void rotateRight(){
  analogWrite(en1, 75);
  digitalWrite(motor1f,HIGH);
  digitalWrite(motor1b,LOW);
  digitalWrite(motor2f,LOW);
  digitalWrite(motor2b,LOW);
}

int sen3Change(){
  uint8_t sen3 = digitalRead(sensor3);
  int countC = 0;
  if (sen3 != lastState){
    if (sen3 == 0)
      countC = 1;
  }
  lastState = sen3;
  return countC;
}

int PID(){
  P = error;
  I += error;
  D = error - last_error;
  last_error = error;
  int pid = Kp*P + Ki*I + Kd*D;
  return pid;
}

int check() {
  uint8_t sen1 = digitalRead(sensor1);
  uint8_t sen2 = digitalRead(sensor2);
  uint8_t sen3 = digitalRead(sensor3);
  uint8_t sen4 = digitalRead(sensor4);
  uint8_t sen5 = digitalRead(sensor5);
  int er = 0;
  // if ((sen1 == 0 && sen2 == 1 && sen3 == 1 && sen4 == 1 && sen5 == 1) || (sen1 == 0 && sen2 == 0 && sen3 == 1 && sen4 == 1 && sen5 == 1))
  //   er = -3;
  if (sen2 == 0)
    er += -1;
  if (sen3 == 0)
    er += 0;
  if (sen4 == 0)
    er = 1;
  // else if ((sen1 == 1 && sen2 == 1 && sen3 == 1 && sen4 == 1 && sen5 == 0) || (sen1 == 1 && sen2 == 1 && sen3 == 1 && sen4 == 0 && sen5 == 0))  
  //   er = 3;
  // else if ( (sen1 == 0 && sen2 == 0 && sen3 == 0 && sen4 == 0 && sen5 == 0))
  //   er = 4;
  if (sen1 == 0 && mode == 0)
    mode  = 1;
  if (sen5 == 0 && mode == 0)
    mode = 2;
  // if (sen1 == 0 && sen5 == 0)
  //   mode = 0;

  return er;
}

void run (){
  error = check();
  if (mode == 0){
    int pid = PID();
    int motor1Speed = BASE_SPEED * BALANCE + pid;
    int motor2Speed = BASE_SPEED - BALANCE * pid;
    if (motor1Speed >= 255){
      motor1Speed = 255;
    }
    if (motor2Speed >= 255){
      motor2Speed = 255;
    }
    motor1Forward(motor1Speed);
    motor2Forward(motor2Speed);
  }else if (mode == 1){
    rotateRight();
    if(sen3Change())
      mode = 0;
  }else if (mode == 2){
    rotateLeft();
    if (sen3Change())
      mode = 0;
  }
}