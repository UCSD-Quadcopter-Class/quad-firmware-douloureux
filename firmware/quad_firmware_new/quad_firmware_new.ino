#include <serLCD.h>
#include "radio.h"
#include "Adafruit_Simple_AHRS.h"
#include <Wire.h>
#include <SPI.h>
#include <quad_firmware.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// declare led ring
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, LEDPIN, NEO_GRB + NEO_KHZ800);
double LEDDelayTime = 0;
double LEDCurTime = 0;
int j = 0;
int LEDCurPatterns = 0;

bool oldButtonValue = false;


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
    if (pid[i].iTerm > 100) pid[i].iTerm = 100;
    if (pid[i].iTerm < -100) pid[i].iTerm = -100;
    pidOutputs[i] += pid[i].iTerm * pid[i].ki;

    // Calculate D term
    pidOutputs[i] += (error - pid[i].lastError) * pid[i].kd / duration;

    if (i == 2) {
      Serial.print("\tError: "); Serial.print(error);
      Serial.print("\tPterm: "); Serial.print(error * pid[i].kp);
      //Serial.print("\tIterm: "); Serial.print(error * pid[i].iTerm);
      Serial.print("\tDterm: "); Serial.print((error - pid[i].lastError) * pid[i].kd / duration);
      Serial.print("\toutput: "); Serial.println(pidOutputs[i]);
    }
    pid[i].lastError = error;
  }
  lastTime = millis();

}

void rf_receive() {
  struct Copter temp;
  uint8_t s = rfRead( (uint8_t*)(&temp) , (uint8_t)sizeof(Copter));

  if (strcmp(errorCode, temp.errorCode) == 0 && s == sizeof(Copter)) {
    cptr = temp;
    //        Serial.print("y: "); Serial.print(cptr.yaw);
    //        Serial.print("t: "); Serial.print(cptr.throttle);
    //        Serial.print("r: "); Serial.print(cptr.roll);
    //        Serial.print("p: "); Serial.print(cptr.pitch);
    //        Serial.print("b1: "); Serial.print(cptr.b1);
    //        Serial.print("b2: "); Serial.print(cptr.b2);
    //        Serial.print("pen1: "); Serial.print(cptr.pen1);
    //        Serial.print("pen2: "); Serial.println(cptr.pen2);
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
  setupLED();
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
  setupPID_t(&(pid[1]), 4.42, cptr.pen1, 0.73);
  setupPID_t(&(pid[2]), 4.42, cptr.pen1, 0.73);


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

  //pid[0].setPoint = cptr.yaw;
  //pid[1].setPoint = cptr.pitch;
  pid[2].setPoint = cptr.roll;

  // Serial.println(pid[2].setPoint);
  //  pid[0].setPoint = 0;
  //  pid[1].setPoint = 0;
  //  pid[2].setPoint = 0;

  //  Serial.print("\tpen2: "); Serial.print(orientation.pitch);
  //  Serial.print("\tpen2: "); Serial.println(pitch);

  pidCalculation(pid, pidOutputs);
  FR = cptr.throttle + pidOutputs[2];
  FL = cptr.throttle + pidOutputs[2];
  BL = cptr.throttle - pidOutputs[2];
  BR = cptr.throttle - pidOutputs[2];



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
  if (rfAvailable()) rf_receive();

  if (cptr.b2 != oldButtonValue) {\
    oldButtonValue = cptr.b2;
    LEDCurPatterns = (LEDCurPatterns + 1) % NUM_OF_LED_PATTERNS;
    // next funtion will return the pattern name
    nextPattern(LEDCurPatterns);
    //TODO: write a function to show the pattern name on the lcd screen for 2 sec
  
  }

  setPixels(LEDCurPatterns);
  strip.show();
}




//--------------following codes are for LED ring------------------

void setupLED() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  strip.setBrightness(20);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}
//
//void loopLED() {
//  // Some example procedures showing how to display to the pixels:
//  colorWipe(strip.Color(255, 0, 0), 50); // Red
//  colorWipe(strip.Color(0, 255, 0), 50); // Green
//  colorWipe(strip.Color(0, 0, 255), 50); // Blue
////colorWipe(strip.Color(0, 0, 0, 255), 50); // White RGBW
//  // Send a theater pixel chase in...
//  theaterChase(strip.Color(127, 127, 127), 50); // White
//  theaterChase(strip.Color(127, 0, 0), 50); // Red
//  theaterChase(strip.Color(0, 0, 127), 50); // Blue
//
//  rainbow(20);
//  rainbowCycle(20);
//  theaterChaseRainbow(50);
//}

void setLEDDelayTimer(int wait) {
  LEDDelayTime = wait;
}
bool LEDDelay() {
  double timeDiff = abs(millis() - LEDCurTime);
  if (timeDiff > LEDDelayTime) {
    LEDCurTime = millis();
    return true;
  }
  else
    return false;
}

void setPixels(int LEDPattern) {
  int i;
  switch (LEDPattern) {
    case 0:
      if (LEDDelay()) {
        for (i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, Wheel((i + j) & 255));
        }

        j = (j + 1) % 255;
        strip.show();
      }
      setLEDDelayTimer(100);
      break;
    case 1:
      //RED
      if (LEDDelay()) {
        for (i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(127, 0, 0));
        }

        j = (j + 1) % 255;
        strip.show();
      }
      setLEDDelayTimer(100);
      break;
    case 2:
      //White
      if (LEDDelay()) {
        for (i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(127, 127, 127));
        }
        strip.show();
      }
      setLEDDelayTimer(100);
      break;
    case 3:
      //Blue
      if (LEDDelay()) {
        for (i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 0, 127));
        }
        strip.show();
      }
      setLEDDelayTimer(100);
      break;
    case 4:
      //Green
      if (LEDDelay()) {
        for (i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 127, 0));
        }
        strip.show();
      }
      setLEDDelayTimer(100);
      break;
  }
}

char* nextPattern(int LEDPattern) {
  switch (LEDPattern) {
    case 0:
      return "Rainbow";
    case 1:
      return "LOL";
    case 2:
      return "LOL";
    case 3:
      return "LOL";
    case 4:
      return "LOL";
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    //delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    //delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

