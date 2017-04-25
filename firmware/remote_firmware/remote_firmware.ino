#include <serLCD.h>
#include "radio.h"
int yaw = 0;
int throttle = 1;
int roll = 2;
int pitch = 3;

double yaw_lcd = 0;
double throttle_lcd = 0;
double roll_lcd = 0;
double pitch_lcd = 0;

int yaw_val = 0;
int throttle_val = 0;
int roll_val = 0;
int pitch_val = 0;

int yaw_max = 0;
int throttle_max = 0; 
int roll_max = 0;
int pitch_max = 0; 

int yaw_min = 500;
int throttle_min = 500;
int roll_min = 500;
int pitch_min = 500;

int yaw_d = 0;
int throttle_d = 0;
int roll_d = 0;
int pitch_d = 0;

serLCD lcd;

void setup()

{
  rfBegin(13);
  Serial.begin(9600);          //  setup serial
  lcd.clear();
  lcd.display();
  lcd.setBrightness(20);
}

void loop()

{
  delay(1500);
  yaw_val = analogRead(yaw); 
  throttle_val = analogRead(throttle);    
  roll_val = analogRead(roll);    
  pitch_val = analogRead(pitch);  

  if(yaw_val > yaw_max) { yaw_max = yaw_val; yaw_lcd = ((yaw_val - yaw_min) *32) / ((yaw_max - yaw_min)/8);}
  if(throttle_val > throttle_max) { throttle_max = throttle_val; }
  if(roll_val > roll_max) { roll_max = roll_val; }
  if(pitch_val > pitch_max) { pitch_max = pitch_val; }
  
  if(yaw_val < yaw_min) { yaw_min = yaw_val; yaw_lcd = ((yaw_val - yaw_min) *32) / ((yaw_max - yaw_min)/8);}
  if(throttle_val < throttle_min) { throttle_min = throttle_val; }
  if(roll_val < roll_min) { roll_min = roll_val; }
  if(pitch_val < pitch_min) { pitch_min = pitch_val; }

  
  throttle_lcd = (throttle_val- throttle_min)*32/((throttle_max - throttle_min)/8);
  roll_lcd = (roll_val - roll_min)*32/((roll_max - roll_min)/8);
  pitch_lcd = (pitch_val - pitch_min) *32/((pitch_max - pitch_min)/8);

  
  rfWrite(throttle_lcd);
  lcd.clear();
  lcd.selectLine(0);
  lcd.print("y: "); lcd.print(yaw_lcd);
  lcd.print("  t: "); lcd.print(throttle_lcd);
  lcd.selectLine(1);
  lcd.print("r: "); lcd.print(roll_lcd);
  lcd.print("  p: "); lcd.print(pitch_lcd);
  
  
  

}
