#include <serLCD.h>
#include "radio.h"
#include "quad_firmware.h"

double btn1 = 0;
double btn2 = 0;
double yaw_lcd = 0;
double throttle_lcd = 0;
double roll_lcd = 0;
double pitch_lcd = 0;

double yaw_val = 0;
double throttle_val = 0;
double roll_val = 0;
double pitch_val = 0;

double yaw_max = 0;
double throttle_max = 0; 
double roll_max = 0;
double pitch_max = 0; 

double yaw_min = 500;
double throttle_min = 500;
double roll_min = 500;
double pitch_min = 500;

double yaw_d = 0;
double throttle_d = 0;
double roll_d = 0;
double pitch_d = 0;

double p1 = 0;
double p2 = 0;

double pen1 = 0;
double pen2 = 0;


serLCD lcd;

typedef struct{
  int y;
  int t;
  int r;
  int p;
  int b1;
  int b2;
  int pen1;
  int pen2;

} Copter;

Copter rf_copter;

void rf_sent() {

  
  rfWrite( (uint8_t*)(&rf_copter) , (uint8_t)sizeof(rf_copter));

  
}

Copter * cptr;

void rf_receive() {
  rfRead( (uint8_t*)cptr , (uint8_t)sizeof(rf_copter));
}

void setup()

{
  rfBegin(13);
  Serial.begin(115200);          //  setup serial
  lcd.clear();
  lcd.display();
  lcd.setBrightness(20);
  rf_copter.b1 = 1;
  rf_copter.b2 = 1;
  
}

void loop()

{
  delay(1500);
  pinMode(PIN_BTN1, INPUT_PULLUP);            // Button 1
  pinMode(PIN_BTN2, INPUT_PULLUP);            // Button 2

  btn1 = digitalRead(PIN_BTN1); 
  btn2 = digitalRead(PIN_BTN2);

  rf_copter.b1 = btn1;
  rf_copter.b1 = btn2;
   
  
  pen1 = analogRead(PIN_POT1); 
  pen2 = analogRead(PIN_POT2);

  p1 = (((pen1 - POT_MIN) * (POT_MAXEW)) / (POT_MAX - POT_MIN)) + POT_MINEW;
  p2 = (((pen2 - POT_MIN) * (POT_MAXEW)) / (POT_MAX - POT_MIN)) + POT_MINEW;

  rf_copter.pen1 = p1;
  rf_copter.pen2 = p2;
  
  yaw_val = analogRead(PIN_YAW); 
  throttle_val = analogRead(PIN_THROTTLE);    
  roll_val = analogRead(PIN_ROLL);    
  pitch_val = analogRead(PIN_PITCH);  

  if(yaw_val > yaw_max) { yaw_max = yaw_val;}
  if(throttle_val > throttle_max) { throttle_max = throttle_val; }
  if(roll_val > roll_max) { roll_max = roll_val; }
  if(pitch_val > pitch_max) { pitch_max = pitch_val; }
  
  if(yaw_val < yaw_min) { yaw_min = yaw_val;}
  if(throttle_val < throttle_min) { throttle_min = throttle_val; }
  if(roll_val < roll_min) { roll_min = roll_val; }
  if(pitch_val < pitch_min) { pitch_min = pitch_val; }

  throttle_lcd = (throttle_val- throttle_min)*140/(throttle_max - throttle_min);
  roll_lcd = (roll_val - roll_min)*140/(roll_max - roll_min);
  pitch_lcd = (pitch_val - pitch_min) *140/(pitch_max - pitch_min);
  yaw_lcd = (yaw_val - yaw_min) * 140 / (yaw_max - yaw_min);

  rfWrite(throttle_lcd);
  lcd.clear();
  lcd.selectLine(0);
  lcd.print("y: "); lcd.print((signed)yaw_lcd);
  lcd.print("  t: "); lcd.print((signed)throttle_lcd);
  lcd.selectLine(1);
  lcd.print("r: "); lcd.print((signed)roll_lcd);
  lcd.print("  p: "); lcd.print((signed)pitch_lcd);
  
  rf_copter.t = throttle_lcd;
  rf_copter.y = yaw_lcd;
  rf_copter.r = roll_lcd;
  rf_copter.p = pitch_lcd;
  rf_sent();
  
}
