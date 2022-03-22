#include <Servo.h>
//int servoPin = A2;
Servo mainServo;

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, OUTPUT);
  mainServo.attach(A2, 500, 2500);
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //mainServo.write(0);
  mainServo.writeMicroseconds(600);
  delay(1000);
  //mainServo.write(180);
  mainServo.writeMicroseconds(2400);
  delay(1000);
  Serial.println("running");
}
