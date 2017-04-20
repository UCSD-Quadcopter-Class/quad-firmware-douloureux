#include <serLCD.h>

void setup() {
  Serial.begin(9600);          //  setup serial
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(3, 127);
}
