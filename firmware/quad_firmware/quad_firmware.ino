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


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

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


void loop(){

  sensors_vec_t orientation;

  ahrs.getOrientation(&orientation);
  
  Serial.print("Orientation roll: "); Serial.println((int)orientation.roll);
  Serial.print("Orientation pitch: "); Serial.println((int)orientation.pitch);
  Serial.print("Orientation heading: "); Serial.println((int)orientation.heading);
  Serial.println("");
  delay(4000);
}


