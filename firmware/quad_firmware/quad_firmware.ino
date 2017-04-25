#include <serLCD.h>
#include "radio.h"


int motorPin = 8;
byte throttle = 0;

void setup() {
 
  Serial.begin(9600);          //  setup serial
  while( !Serial );
  Serial.println("Status: Online");
  //Serial.println("Please enter a speed form 0 to 255");
  rfBegin(13);

}


void loop() {
  // put your main code here, to run repeatedly:
  //analogWrite(motorPin, 120);
  /*if( Serial.available()){
    speed = Serial.parseInt();
    Serial.print("Speed is:");
    Serial.println(speed);
    //rfWrite(Serial.read());
  }*/
  if( throttle >= 0 && throttle <= 255 ) {
      analogWrite(motorPin, throttle);
  } 
  if(rfAvailable()){
   throttle = rfRead();
   Serial.println(throttle);
   analogWrite(motorPin, throttle);
  }
}
