 #include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
//int servoPin = A2;
Servo servo1;
Servo servo2;
int set_servo = 0;
int testset = 0;
int t = 0;
sensors_event_t event;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  servo1.attach(A2, 500, 2500);
  servo2.attach(A3, 500, 2500);

  servo1.write(0);
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
  
  bno.getEvent(&event);
  
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly

  /*
   if (Serial.available() > 0) {
    testset = Serial.parseInt();
    Serial.println(testset);
    set_servo = testset;
  }
  */
  
  set_servo = 160;
  /* Get a new sensor event */
  bno.getEvent(&event);

  /* The processing sketch expects data as roll, pitch, heading 
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

   Also send calibration data for each sensor. 
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
  */
  
  
  t = millis();
  Serial.print(t);
  Serial.print(", ");
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.println("");
  servo1.write(set_servo);
  delay(20);
}
