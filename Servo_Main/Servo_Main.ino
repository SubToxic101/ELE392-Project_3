#include <Servo.h>
int servoPin = A2;
Servo mainServo;

void setup() {
  // put your setup code here, to run once:
  pinMode(servoPin, OUTPUT);
  mainServo.attach(pin, 500, 2500);
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo.write(0);
  delay(1000);
  servo.write(180);
  delay(1000);
}
