 #include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>
//int servoPin = A2;
Servo servo1;
Servo servo2;
int set_servo = 0;
int testset = 0;
int t = 0;
int firstprint = 0;
sensors_event_t event;
double servo1In = 0;
double servo1Out = 0;
double Setpoint_1 = 0;
double servo2In = 0;
double servo2Out = 0;
double Setpoint_2 = 0;

int Kp1 = -355;
int Ki1 = -88.1;
int Kd1 = -20.9;

int Kp2 = 395;
int Ki2 = 172;
int Kd2 = 14.3;


PID servoPitch(&servo1In, &servo1Out, &Setpoint_1, Kp1, Ki1, Kd1, DIRECT);
PID servoYaw(&servo2In, &servo2Out, &Setpoint_2, Kp2, Ki2, Kd2, DIRECT);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  Setpoint_1 = 75;
  Setpoint_2 = 0;
  servo1.attach(A2, 500, 2500);
  servo2.attach(A3, 500, 2500);

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
  servoPitch.SetSampleTime(22);
  servoYaw.SetSampleTime(22);
  servoPitch.SetOutputLimits(0, 180);
  servoYaw.SetOutputLimits(0, 180);
  servoPitch.SetMode(AUTOMATIC);
  servoYaw.SetMode(AUTOMATIC);
  delay(3000);
}

void loop() {
  /* Get a new sensor event */
  bno.getEvent(&event);

  t = millis();
  Serial.print(t);
  Serial.print(", ");
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /*
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

  servo1In = (double)event.orientation.y;
  servo2In = (double)event.orientation.x;
  servoPitch.Compute();
  servoYaw.Compute();

  Serial.println(servo1Out);
  Serial.println(servo2Out);
  servo1.write(servo1Out);
  servo2.write(servo2Out);
  
  if (firstprint == 0) {
    Serial.println("Starting....");
    firstprint = 1;
  }
  delay(20);
}
