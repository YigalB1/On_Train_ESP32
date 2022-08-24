#include <Arduino.h>
int on_board_led = 2;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup starting");
  pinMode(on_board_led,OUTPUT);
}

bool tmp = false;

void loop() {
  Serial.println("in work");
  if (tmp) {
    digitalWrite(on_board_led,LOW);
    tmp = true;
  } // of if()
  else {
    digitalWrite(on_board_led,HIGH);
    tmp = false;
  } // of else()
  delay(500);
  
}