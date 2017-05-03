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

int FL = 0;
int FR = 0;
int BL = 0;
int BR = 0;

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

double duration = 100;

double cur_time = 0;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

}

void setupPID(int a, int b, int c, int min_value, int max_value){
  kp = a;
  ki = b;
  kd = c;

  i_min = min_value;
  i_max = max_value;
}

void pid(){
  if(cur_time - millis() < duration) return;
  cur_time = millis();
  roll_error = roll_input - roll_cur;
  i_term += roll_error * ki;

  if(i_term > i_max ) i_term = i_max;
  if(i_term < i_min ) i_term = i_min;

  roll_output = kp * roll_error + i_term - (roll_input - roll_last_input) * kd;
  
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
  setupPID(2,1,3,-100,100);
  cur_time = millis();
}

void loop(){

  sensors_vec_t orientation;
  ahrs.getOrientation(&orientation);
  
  Serial.print("Orientation roll: "); Serial.println(orientation.roll);
  Serial.print("Orientation pitch: "); Serial.println(orientation.pitch);
  Serial.println("");
  
  roll_cur = orientation.pitch;
  pid();
  analogWrite(pe4, 100+roll_output); //FR
  analogWrite(pb5, 100+roll_output); // FL
  analogWrite(pe3, 100+roll_output); //BR
  analogWrite(pe5, 100+roll_output); //BL
  Serial.print("pid: "); Serial.println(roll_output);
}




//void loop() {
//  // put your main code here, to run repeatedly:
//  //analogWrite(motorPin, 120);
//  if( Serial.available()){
//    throttle = Serial.parseInt();
//    Serial.print("Throttle is:");
//    Serial.println(throttle);
//    //rfWrite(Serial.read());
//  }
//  if( throttle >= 0 && throttle <= 255 ) {
//    analogWrite(pb5, 250);
//    analogWrite(pe5, 150);
//    analogWrite(pe3, 250);
//    analogWrite(pe4, 250);
//  } 
//  if(rfAvailable()){
//   throttle = rfRead();
//   Serial.println(throttle);
//   analogWrite(pb5, throttle);
//   analogWrite(pe5, throttle);
//   analogWrite(pe3, throttle);
//   analogWrite(pe4, throttle);
//  }
//}



