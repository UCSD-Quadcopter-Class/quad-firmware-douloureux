#include <serLCD.h>
#include "radio.h"
#include "Adafruit_Simple_AHRS.h"
#include <Wire.h>
#include <SPI.h>
#include <quad_firmware.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
char errorCode[] = "douluo";
double duration = 10;
double lastTime = 0;

double pitchHistory[10];
int curPitchHistoryIndex = 0;
double pitchSum = 0;
double rollHistory[10];
int curRollHistoryIndex = 0;
double rollSum = 0;

double rollAver;
double pitchAver;

double FR = 0;
double FL = 0;
double BR = 0;
double BL = 0;

struct Copter {
  int yaw;
  int throttle;
  int roll;
  int pitch;
  boolean b1;
  boolean b2;
  double pen1;
  double pen2;
  char errorCode[7];
};

struct PIDController {
  double input;
  double setPoint;
  double kp;
  double ki;
  double kd;
  double output;
  double iTerm;
  double lastError;
};


struct Copter cptr;

// pid[0] is for yaw, pid[1] is for pitch, pid[2] is for roll
struct PIDController pid[3];


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

}


void setputPID(struct PIDController* pid) {
  pid[0].kp = YAW_P;
  pid[0].ki = YAW_I;
  pid[0].kd = YAW_D;
  pid[1].kp = PITCH_P;
  pid[1].ki = PITCH_I;
  pid[1].kd = PITCH_D;
  pid[2].kp = ROLL_P;
  pid[2].ki = ROLL_I;
  pid[2].kd = ROLL_D;
}


// this function is only for test
void setupPID_t(struct PIDController* pid, double Constant_P, double Constant_I, double Constant_D) {
  pid->kp = Constant_P;
  pid->ki = Constant_I;
  pid->kd = Constant_D;
}

double pitchAverage(double pitch, int index) {
  int i = 0;
  pitchSum = pitch + pitchSum - pitchHistory[index];
  pitchHistory[index]  = pitch;
  //  for(; i < 10; i++){
  //    Serial.print(pitchHistory[i]);
  //    Serial.print("  ");
  //  }
  //  Serial.println("");
  return pitchSum / 10;
}
double rollAverage(double roll) {
  rollSum = roll + rollSum - rollHistory[curRollHistoryIndex];
  rollHistory[curRollHistoryIndex]  = roll;
  curRollHistoryIndex == (curRollHistoryIndex + 1) % 10;
  return rollSum / 10;
}


void pidCalculation(struct PIDController* pid, double* pidOutputs) {
  double input = 0;
  double setPoint = 0;
  double error = 0;

  duration = abs(millis() - lastTime) / 1000;

  for (int i = 0; i < 3; i++) {

    pidOutputs[i] = 0;
    input = pid[i].input;
    setPoint = pid[i].setPoint;
    error = input - setPoint;

    // Calculate P term
    pidOutputs[i] += error * pid[i].kp;
    
    // Calculate I term
    pid[i].iTerm += error * pid[i].ki * duration;
    pidOutputs[i] += pid[i].iTerm * pid[i].ki;

    // Calculate D term
    pidOutputs[i] += (error - pid[i].lastError) * pid[i].kd / duration;
    
//
//    if (i == 2) {
//      Serial.print("\tError: "); Serial.print(error);
//      Serial.print("\tPterm: "); Serial.print(error * pid[i].kp);
//      Serial.print("\tDterm: "); Serial.print((error - pid[i].lastError) * pid[i].kd / duration);
//      Serial.print("\toutput: "); Serial.println(pidOutputs[i]);
//    }
    pid[i].lastError = error;
  }
  lastTime = millis();

}

void rf_receive() {
  struct Copter temp;
  uint8_t s = rfRead( (uint8_t*)(&temp) , (uint8_t)sizeof(Copter));

  if (strcmp(errorCode, temp.errorCode) == 0 && s == sizeof(Copter)) {
    cptr = temp;
        Serial.print("y: "); Serial.print(cptr.yaw);
        Serial.print("t: "); Serial.print(cptr.throttle);
        Serial.print("r: "); Serial.print(cptr.roll);
        Serial.print("p: "); Serial.print(cptr.pitch);
        Serial.print("b1: "); Serial.print(cptr.b1);
        Serial.print("b2: "); Serial.print(cptr.b2);
        Serial.print("pen1: "); Serial.print(cptr.pen1);
        Serial.print("pen2: "); Serial.println(cptr.pen2);
  }
  else {
    //    Serial.println("ERROR!!!!");
  }
}


void setup() {

  Serial.begin(115200);          //  setup serial
  while ( !Serial );
  Serial.println("Status: Online");
  rfBegin(13);
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  setupSensor();
  setputPID(pid);
  lastTime = millis();
  pinMode(PE5, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PE3, OUTPUT);
  pinMode(PE4, OUTPUT);
}

void loop() {
  sensors_vec_t orientation;
  ahrs.getOrientation(&orientation);
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);


  setupPID_t(&(pid[0]), cptr.pen2, 0, cptr.pen1);
  setupPID_t(&(pid[1]), cptr.pen2, 0, cptr.pen1);
  setupPID_t(&(pid[2]), cptr.pen2, 0, cptr.pen1);


  float roll_g = g.gyro.x;
  float pitch_g = g.gyro.y;
  double pidOutputs[3];
  double roll;
  double pitch;
  curPitchHistoryIndex = (curPitchHistoryIndex + 1) % 10;
  pitchAver = pitchAverage(orientation.pitch, curPitchHistoryIndex);
  roll = 0.97 * (pid[1].input - g.gyro.x * 0.01) + 0.03 * orientation.roll;
  pitch = 0.97 * (pid[2].input - g.gyro.y * 0.01) + 0.03 * pitchAver;



  pid[0].input = 0;
  pid[1].input = roll;
  pid[2].input = pitch;
  
  //  pid[0].setPoint = cptr.yaw;
  //  pid[1].setPoint = cptr.pitch;
  pid[2].setPoint = cptr.roll;

//  Serial.println(pid[2].setPoint);
//  pid[0].setPoint = 0;
//  pid[1].setPoint = 0;
//  pid[2].setPoint = 0;

  //  Serial.print("\tpen2: "); Serial.print(orientation.pitch);
  //  Serial.print("\tpen2: "); Serial.println(pitch);

  pidCalculation(pid, pidOutputs);
  FR = cptr.throttle - 15 + pidOutputs[2];
  FL = cptr.throttle - 15 + pidOutputs[2];
  BL = cptr.throttle + 10 - pidOutputs[2];
  BR = cptr.throttle + 10 - pidOutputs[2];



  if (FR > 250) FR = 250;
  if (FR < 0 ) FR = 0;
  if (BL > 250) BL = 250;
  if (BL < 0) BL = 0;

  FL = FR;
  BR = BL;


  if (cptr.b1) {
    analogWrite(PE5, FR ); //FR
    analogWrite(PB5, FL); // FL
    analogWrite(PE3, BL); //BL
    analogWrite(PE4, BR); //BR
  }
  else {
    analogWrite(PE5, 0); //FR
    analogWrite(PB5, 0); // FL
    analogWrite(PE3, 0); //BL
    analogWrite(PE4, 0); //BR
  }
  if (rfAvailable())
    rf_receive();
}




