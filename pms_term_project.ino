#include <Servo.h>
#include <AFMotor.h>

#define Echo A0
#define Trig A1
#define motor 10
#define Speed 150
#define spoint 103
#define IR A2

char value;
int distance;
int Left;
int Right;
int Straight;
int L = 0;
int R = 0;
int F = 0;
int L1 = 0;
int R1 = 0;
int buzzer=13;
int led=12;
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(IR, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}
void loop() {
  Bluetoothcontrol();
  //voicecontrol();
  int irval = digitalRead(IR);
  //Serial.println(irval);
    if (irval == 0){
        Stop();
        digitalWrite(buzzer, HIGH);
    }
    if (irval == 1){
        digitalWrite(buzzer, LOW);
    }
     F = frontsee();
      servo.write(spoint);
    if (F < 10) {
        Stop();
        digitalWrite(buzzer, HIGH);
        delay(500);
        digitalWrite(buzzer, LOW);
      }  
}

void Bluetoothcontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
   // Serial.println(value);
  if (value == 'F') {
    forward();
  } else if (value == 'B') {
    backward();
  } else if (value == 'L') {
    left();
  } else if (value == 'R') {
    right();
  } else if (value == 'S') {
    Stop();
  }
  else if (value == 'W') {
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
  }
  else if (value == 'V') {
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
  }
  }
}

void voicecontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    if (value == '^') {
      F = frontsee();
      servo.write(spoint);
      if (F >= 10 ) {
        forward();
      } else if (F < 10) {
        Stop();
        digitalWrite(buzzer, HIGH);
        delay(500);
        digitalWrite(buzzer, LOW);
      }
    } else if (value == '-') {
      int irval = digitalRead(IR);
    if (irval == 0){
      Stop();
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    } else {
      backward();
    }
    } else if (value == '<') {
      L = leftsee();
      servo.write(spoint);
      if (L >= 10 ) {
        left();
        delay(2500);
        Stop();
      } else if (L < 10) {
        Stop();
        digitalWrite(buzzer, HIGH);
        delay(500);
        digitalWrite(buzzer, LOW);
      }
    } else if (value == '>') {
      R = rightsee();
      servo.write(spoint);
      if (R >= 10 ) {
        right();
        delay(2500);
        Stop();
      } else if (R < 10) {
        Stop();
        digitalWrite(buzzer, HIGH);
        delay(500);
        digitalWrite(buzzer, LOW);
      }
    } else if (value == '*') {
      Stop();
    } else if (value == ',') {
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
    }else if (value == ')') {
      digitalWrite(led, HIGH);

    }else if (value == '(') {
      digitalWrite(led, LOW);

    }
  }
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; //time convert distance
  return cm;
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void right() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void left() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}
int rightsee() {
  servo.write(20);
  delay(200);
  Left = ultrasonic();
  return Left;
}

int leftsee() {
  servo.write(180);
  delay(200);
  Right = ultrasonic();
  return Right;
}
int frontsee() {
  Straight = ultrasonic();
  return Straight;
  }
