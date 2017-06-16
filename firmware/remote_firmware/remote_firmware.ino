#include <serLCD.h>
#include <radio.h>
#include <quad_firmware.h>


double yaw_lcd = 0;
double throttle_lcd = 0;
double roll_lcd = 0;
double pitch_lcd = 0;

char error[] = "douluo";

double yaw_val = 0;
double throttle_val = 0;
double roll_val = 0;
double pitch_val = 0;

double p1 = 0;
double p2 = 0;
double pen1 = 0;
double pen2 = 0;
boolean btn1 = 0;
boolean btn2 = 0;

int timer = 0;
serLCD lcd;

struct Copter{
  
  int yaw;
  int throttle;
  int roll;
  int pitch;
  boolean b1;
  int b2;
  double pen1;
  double pen2;
  char errorCode[7];

} ;
Copter rf_copter;

void rf_sent() {
  rfWrite( (uint8_t*)(&rf_copter) , (uint8_t)sizeof(Copter)); 
}

Copter * cptr;

void normalize(struct Copter* cptr, double throttle, double pitch, double yaw, double roll) {
  if( pitch > PITCH_MIDPOINT ) {
    cptr->pitch = (pitch - PITCH_MIDPOINT)/(PITCH_MAX-PITCH_MIDPOINT)*PITCH_CONSTRAIN;
  }
  else{
    cptr->pitch = (pitch - PITCH_MIDPOINT ) / ( PITCH_MIDPOINT - PITCH_MIN ) * PITCH_CONSTRAIN;
  }
  if( yaw > YAW_MIDPOINT ) {
    cptr->yaw = (yaw - YAW_MIDPOINT ) / ( YAW_MAX - YAW_MIDPOINT ) * YAW_CONSTRAIN;
  }
  else{
    cptr->yaw = (yaw - YAW_MIDPOINT)/( YAW_MIDPOINT - YAW_MIN ) * YAW_CONSTRAIN;
  }
  if( roll > ROLL_MIDPOINT ) {
    cptr->roll = (roll - ROLL_MIDPOINT ) / ( ROLL_MAX - ROLL_MIDPOINT ) * ROLL_CONSTRAIN;
  }
  else{
    cptr->roll = (roll - ROLL_MIDPOINT)/( ROLL_MIDPOINT - ROLL_MIN ) * ROLL_CONSTRAIN;
  }
  cptr->throttle = (throttle-THROTTLE_MIN)/(THROTTLE_MAX - THROTTLE_MIN) * THROTTLE_CONSTRAIN;
}

void rf_receive() {
  rfRead( (uint8_t*)cptr , (uint8_t)sizeof(rf_copter));
}

void buttonClicked1(){
  if( rf_copter.b1 == false) rf_copter.b1 = true;
  else if( rf_copter.b1 == true ) rf_copter.b1 = false;
}

void buttonClicked2(){
  if( rf_copter.b2 == false) rf_copter.b2 = true;
  else if( rf_copter.b2 == true ) rf_copter.b2 = false;
}


void setup()
{
  rfBegin(CHANEL);
  Serial.begin(115200);          //  setup serial
  lcd.clear();
  lcd.display();
  lcd.setBrightness(20);
  
  
  rf_copter.b1 = false;
  rf_copter.b2 = false;

  timer = millis();
}

void loop()

{
  delay(10);
  
  pinMode(PIN_BTN1, INPUT_PULLUP);            // Button 1
  pinMode(PIN_BTN2, INPUT_PULLUP);            // Button 2
  btn1 = digitalRead(PIN_BTN1); 
  btn2 = digitalRead(PIN_BTN2);
  if( btn1 == 0 ) { buttonClicked1(); }
  
  rf_copter.b2 = btn2;
  pen1 = analogRead(PIN_POT1); 
  pen2 = analogRead(PIN_POT2);
  p1 = (((pen1 - POT_MIN) * (POT_MAXEW1)) / (POT_MAX - POT_MIN)) + POT_MINEW1;
  p2 = (((pen2 - POT_MIN) * (POT_MAXEW2)) / (POT_MAX - POT_MIN)) + POT_MINEW2;
  if( p1 < 0 ) { p1 = 0; }
  if( p2 < 0 ) { p2 = 0; }
  rf_copter.pen1 = p1;
  rf_copter.pen2 = p2;
  
  yaw_val = analogRead(PIN_YAW); 
  throttle_val = analogRead(PIN_THROTTLE);    
  roll_val = analogRead(PIN_ROLL);    
  pitch_val = analogRead(PIN_PITCH);  
  
//  rf_copter.throttle = throttle_val;
//  rf_copter.pitch = pitch_val;
//  rf_copter.roll = roll_val;
//  rf_copter.yaw = yaw_val;

  normalize(&rf_copter, throttle_val, pitch_val, yaw_val, roll_val);


  lcd.clear();
  lcd.selectLine(0);
  lcd.print("y: "); lcd.print((signed)rf_copter.yaw);
  lcd.print("  t: "); lcd.print((signed)rf_copter.throttle);
  lcd.selectLine(1);
  lcd.print("r: "); lcd.print((signed)rf_copter.roll);
  lcd.print("  p: "); lcd.print((signed)rf_copter.pitch);

  Serial.print("Throttle:");
  Serial.print(rf_copter.throttle);
  Serial.print("  Pitch:");
  Serial.print(rf_copter.pitch);
  Serial.print("  Yaw:");
  Serial.print(rf_copter.yaw);
  Serial.print("  Roll:");
  Serial.print(rf_copter.roll);
  Serial.print("  D:");
  Serial.print(p1);
  Serial.print("  P:");
  Serial.println(p2);
//  Serial.print("Button1:");
//  //Serial.print(btn1);
//  Serial.print(rf_copter.b1);
//  Serial.print("\tButton2:");
//  Serial.println(btn2);
  //Serial.println(rf_copter.b2);
  strcpy(rf_copter.errorCode, error);
  rf_sent();
  
}
