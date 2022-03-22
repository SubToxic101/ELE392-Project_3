#include <Servo.h>
//int servoPin = A2;
Servo servo1;
Servo servo2;

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  servo1.attach(A2, 500, 2500);
  servo2.attach(A3, 500, 2500);
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //mainServo.write(0);
  servo1.write(0);
  servo2.write(0);
  delay(1000);
  //mainServo.write(180);
  servo1.write(180);
  servo2.write(1800);
  delay(1000);
  Serial.println("running");
}
