 #include <serLCD.h>
#include "radio.h"
#include "Adafruit_Simple_AHRS.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

  int y1;
  int t1;
  int r1;
  int p1;
  int b11;
  int b21;
  double pen11;
  double pen21;

  int yR;
  int tR;
  int rR;
  int pR;
  int b1R;
  int b2R;
  double pen1R;
  double pen2R;



  int y;
  int t;
  int r;
  int p;
  int b1;
  int b2;
  double pen1;
  double pen2;  

int initialize = 0;
int pb5 = 8;
int pe5 = 5;
int pe3 = 3;
int pe4 = 4;

double range = 10;

double FL = 150;
double FR = 150;
double BL = 120;
double BR = 130;

double roll_input = 0;
double pitch_input = 0;

double roll_last_error = 0;
double pitch_last_error = 0;

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

unsigned int last_time = 0;
int loop_count = 0;


float pitch;
float roll;

typedef struct{
  int y;
  int t;
  int r;
  int p;
  boolean b1;
  boolean b2;
  double pen1;
  double pen2;

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
  
}

void pid(){
  if(abs(millis() - last_time) < duration) return;
  last_time = millis();
  
  roll_error = roll_input - roll_cur;
  i_term += roll_error * ki*duration;
  
  if(i_term > i_max ) i_term = i_max;
  if(i_term < i_min ) i_term = i_min;
  
  
  roll_output = kp * roll_error + i_term - ( roll_error - roll_last_error) * kd/duration; 

  
//  if(roll_output > 50 ) roll_output = 50;
//  if(roll_output < -50 ) roll_output = -50;

  Serial.print("\terror: ");Serial.print( roll_error);
  //Serial.print("\tP: ");Serial.print(kp * roll_error);
  //Serial.print("\tI: ");Serial.print(i_term);
  Serial.print("\tD: ");Serial.println((roll_error - roll_last_error) * kd/duration);
 // Serial.print("\tOUTPUT: ");Serial.println(roll_output);

  roll_last_error = roll_error;
}


int rfError(){

    yR = cptr.y;
    tR = cptr.t;
    rR = cptr.r;
    pR = cptr.p;
    pen1R = cptr.pen1;
    pen2R = cptr.pen2;
  if( initialize == 0 ) { 
    initialize = 1; 
    y1 = cptr.y;
    t1 = cptr.t;
    r1 = cptr.r;
    p1 = cptr.p;
    pen11 = cptr.pen1;
    pen21 = cptr.pen2;
   
    return 1; 
  }

  if( (0 < abs(y1 - cptr.y) < 30)  ) { y1 = cptr.y; }
  else if( 0 < (abs(cptr.y - yR) < 30) ) { y1 = cptr.y; cptr.y = yR; }
  else { cptr.y = y1; }
  
  if( (0 <abs(t1 - cptr.t) < 30) ) { t1 = cptr.t;}
  else if( (0 <abs(cptr.t - tR) < 30) ) { t1 = cptr.t; cptr.t = tR; }
  else { cptr.t = t1;}
  
  if( (0 <abs(r1 - cptr.r) < 30) ) { r1 = cptr.r; }
  else if( (abs(cptr.r - rR) < 30) ) { r1 = cptr.r; cptr.r = rR; }
  else{ cptr.r = r1; }
  
  if( (0 <abs(p1 - cptr.p) < 30) ) { p1 = cptr.p; }
  else if( (0 <abs(cptr.p - pR) < 30)) { p1 = cptr.p; cptr.p = pR; }
  else{ cptr.p = p1; }
  
  if( ((0 < abs(pen11 - cptr.pen1) < 80)) && 0<pen11<200) { pen11 = cptr.pen1; }
  else if( (0 <abs(cptr.pen1 - pen1R) < 80) && 0<pen11<200 ) { pen11 = cptr.pen1; cptr.pen1 = pen1R; }
  else{ cptr.pen1 = pen11; }
  
  if( (0 <abs(pen21 - cptr.pen2 ) < 80) ) { pen21 = cptr.pen2; }
  else if( (0 <abs(cptr.pen2 - pen2R) < 80) ) { pen21 = cptr.pen2; cptr.pen2 = pen2R; }
  else{ cptr.pen2 = pen21; }
    

  return 1;
  
}

void rf_receive() {
  rfRead( (uint8_t*)(&cptr) , (uint8_t)sizeof(Copter));
  if(  rfError() ) {
//    Serial.print("y: "); Serial.print(cptr.y);
//    Serial.print("t: "); Serial.print(cptr.t);
//    Serial.print("r: "); Serial.print(cptr.r);
//    Serial.print("p: "); Serial.print(cptr.p);
//    Serial.print("b1: "); Serial.print(cptr.b1);
//    Serial.print("b2: "); Serial.print(cptr.b2);
//    Serial.print("pen1: "); Serial.print(cptr.pen1);
//    Serial.print("pen2: "); Serial.println(cptr.pen2);

  }
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
  last_time = millis();
  pinMode(pe5, OUTPUT);
  pinMode(pb5, OUTPUT);
  pinMode(pe3, OUTPUT);
  pinMode(pe4, OUTPUT);
  setupPID(cptr.pen2,0,20,-100,100);
}

void loop(){
  sensors_vec_t orientation;
  ahrs.getOrientation(&orientation);


  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  setupPID(cptr.pen2,0,cptr.pen1,-100,100);
 

  float roll_g = g.gyro.x;
  float pitch_g = g.gyro.y;

  roll = 0.9*(roll + g.gyro.x * 0.001) + 0.1 * orientation.roll;
  pitch = 0.9*(pitch + g.gyro.y * 0.001) + 0.1 * orientation.pitch;

//  Serial.print(roll); 
//  Serial.println(pitch);
  roll_input = 0;
  roll_cur = pitch;
  pid();
  //Serial.println(cptr.b1);
  if(cptr.b1){
    analogWrite(pe5, 180 - roll_output); //FR
    analogWrite(pb5, 180 - roll_output); // FL
    analogWrite(pe3, 160+ roll_output); //BL
    analogWrite(pe4, 160+ roll_output); //BR
  }
  else{
    analogWrite(pe5, 0); //FR
    analogWrite(pb5, 0); // FL
    analogWrite(pe3, 0); //BL
    analogWrite(pe4, 0); //BR
  }
  if(rfAvailable())
    rf_receive();
}




