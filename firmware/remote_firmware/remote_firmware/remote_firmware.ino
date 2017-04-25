#include <serLCD.h>
int yaw = 0;
int throttle = 1;
int roll = 2;
int pitch = 3;

int yaw_val = 0;
int throttle_val = 0;
int roll_val = 0;
int pitch_val = 0;

int yaw_val_temp = 0;
int throttle_val_temp = 0;
int roll_val_temp = 0;
int pitch_val_temp = 0;

serLCD lcd;

void setup()

{
  
  Serial.begin(9600);          //  setup serial

}

void loop()

{
  lcd.print("I love BJ KIM");
  /*yaw_val_temp = analogRead(yaw); 
  throttle_val_temp = analogRead(throttle);    
  roll_val_temp = analogRead(roll);    
  pitch_val_temp = analogRead(pitch);    

  if(roll_val != roll_val_temp){
    roll_val = roll_val_temp;
    Serial.print("roll: "); Serial.println(roll_val);
//    Serial.print("throttle: "); Serial.println(throttle_val); 
//    Serial.print("roll_val: "); Serial.println(roll_val); 
//    Serial.print("pitch vale: " ); Serial.println(pitch_val);  
  } 
  */

}
