#include <serLCD.h>
#include "radio.h"
#include "Adafruit_Simple_AHRS.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());


int pb5 = 8;
int pe5 = 5;
int pe3 = 3;
int pe4 = 4;

double FL = 0;
double FR = 0;
double BL = 0;
double BR = 0;

double roll_input = 0;
double pitch_input = 0;

double roll_last_input = 0;
double pitch_last_input = 0;

double roll_cur = 0;
double pitch_cur = 0;

double roll_error = 0;
double pitch_error = 0;

double roll_output = 0;
double pitch_output = 0;

double i_term = 0;
double i_min = 0;
double i_max = 0;

double kp = 0;
double ki = 0;
double kd = 0;

double duration = 1;

double last_time = 0;

typedef struct{
  int y;
  int t;
  int r;
  int p;

} Copter;

Copter cptr;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

}

void setupPID(double a, double b, double c, double min_value, double max_value){
  kp = a;
  ki = b;
  kd = c;

  i_min = min_value;
  i_max = max_value;
  last_time = millis();
}

void pid(){
  if(millis() - last_time < duration) return;
  last_time = millis();
  roll_error = roll_input - roll_cur;
  i_term += roll_error * ki;
  //Serial.println(i_term);
  if(i_term > i_max ) i_term = i_max;
  if(i_term < i_min ) i_term = i_min;
  
  
  roll_output = kp * roll_error + i_term - (roll_input - roll_last_input) * kd; 
  
  
  Serial.print("\tP: ");Serial.print(kp * roll_error);
  Serial.print("\tI: ");Serial.print(i_term);
  Serial.print("\tD: ");Serial.print((roll_input - roll_last_input) * kd);
  Serial.print("\tOUTPUT: ");Serial.println(roll_output);
  roll_last_input = roll_input;
}



void rf_receive() {
  rfRead( (uint8_t*)(&cptr) , (uint8_t)sizeof(Copter));
  Serial.print("y: "); Serial.println(cptr.y);
  Serial.print("t: "); Serial.println(cptr.t);
  Serial.print("r: "); Serial.println(cptr.r);
  Serial.print("p: "); Serial.println(cptr.p);
}



void setup() {
 
  Serial.begin(115200);          //  setup serial
  while( !Serial );
  Serial.println("Status: Online");
  //Serial.println("Please enter a speed form 0 to 255");
  rfBegin(13);

  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  setupSensor();
  setupPID(1.8++
  
  ,0,0,-100,100);
}

void loop(){

  sensors_vec_t orientation;
  ahrs.getOrientation(&orientation);
  
//Serial.print("Orientation roll: "); Serial.println(orientation.roll);
//Serial.print("Orientation pitch: "); Serial.println(orientation.pitch);
//  Serial.println("");
  roll_input = 0;
  roll_cur = orientation.pitch;
  pid();
//  
//  analogWrite(pe5, (160-roll_output)*1.1); //FR
//  analogWrite(pb5, (160-roll_output)*1.1); // FL
//  analogWrite(pe3, 160+roll_output); //BL
//  analogWrite(pe4, 160+roll_output); //BR

  analogWrite(pe5, 165-roll_output); //FR
  analogWrite(pb5, 160-roll_output); // FL
  analogWrite(pe3, 140+roll_output); //BL
  analogWrite(pe4, 145+roll_output); //BR 



//  if(rfAvailable())
//    rf_receive();
}




