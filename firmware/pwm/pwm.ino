#include <serLCD.h>

int motorPin = 8;
int speed = 0;

void setup() {
 
  Serial.begin(9600);          //  setup serial
  while( !Serial );
  Serial.println("Please enter a speed form 0 to 255");
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(motorPin, 120);
  /*if( Serial.available()){
    speed = Serial.parseInt();
    Serial.print("Speed is:");
    Serial.println(speed);
  }
  if( speed >= 0 && speed <= 255 ) {
      analogWrite(motorPin, speed);
    } */
}
