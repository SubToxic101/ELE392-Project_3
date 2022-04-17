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
int loopCount = 0;
sensors_event_t event;
double servo1In = 0;
double servo1Out = 0;
double Setpoint_1 = 0;
double servo2In = 0;
double servo2Out = 0;
double Setpoint_2 = 0;

double anglStart = 0;
double anglFix = 0;

double Kp1 = 0;
double Ki1 = 3;
double Kd1 = 0.008;

double Kp2 = 1;
double Ki2 = 0;
double Kd2 = 0;


PID servoPitch(&servo1In, &servo1Out, &Setpoint_1, Kp1, Ki1, Kd1, DIRECT);

PID servoYaw(&servo2In, &servo2Out, &Setpoint_2, Kp2, Ki2, Kd2, DIRECT);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  Setpoint_1 = 0.0;
  Setpoint_2 = 180.0;
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
  servoPitch.SetSampleTime(50);
  servoYaw.SetSampleTime(50);
  //servoPitch.SetOutputLimits(0, 180);
  servoYaw.SetOutputLimits(-500, 500);
  servoPitch.SetMode(AUTOMATIC);
  servoYaw.SetMode(AUTOMATIC);
  servo1.write(75);
  servo2.write(0);
  delay(1000);
  bno.getEvent(&event);
  anglStart = (double)event.orientation.x;
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

  /*
  if (servo2In < anglStart) {
    servo2In = servo2In + (360-anglStart);
  }
  else if (servo2In > anglStart) {
    servo2In = 360 - (anglStart + (360-servo2In));
  }
  */
 
  Serial.print("anglStart:  ");
  Serial.println(anglStart);
  Serial.print("servo2In:  ");
  Serial.println(servo2In);
  
  servoPitch.Compute();
  servoYaw.Compute();
 
  

  Serial.println(servo1Out);
  Serial.println(servo2Out);
  servo1.write(servo1Out);
  servo2.write(180+servo2Out);
  
  if (firstprint == 0) {
    Serial.println("Starting....");
    firstprint = 1;
  }
  delay(50);
}
