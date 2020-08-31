#include "pitches.h"

int pirPin = 8; // Input for HC-S501
int pirValue; // Place to store read PIR Value
int buzzerPin = 7;
int toneLength = 100; //500 ms duration
void setup() {
  pinMode(buzzerPin,OUTPUT);
  pinMode(pirPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  pirValue = digitalRead(pirPin);
  if (pirValue == HIGH){
    tone(buzzerPin,NOTE_C5,toneLength);
  }

}
