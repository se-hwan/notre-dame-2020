#include "pitches.h"

int buzzerPin = 7;
void setup() {
  // put your setup code here, to run once:
  pinMode(buzzerPin,OUTPUT);
}
int bpm = 250; //speed of song
int yiruma[]= {NOTE_A5,NOTE_B5,NOTE_A5,NOTE_GS5,NOTE_A5,NOTE_A4,NOTE_E5,NOTE_A4,NOTE_A5,NOTE_B5,NOTE_A5,NOTE_GS5,NOTE_A5,NOTE_A4,NOTE_E5,NOTE_A4,NOTE_A5,NOTE_B5,NOTE_A5,NOTE_GS5,NOTE_A5,NOTE_B5,NOTE_CS6,NOTE_D6,NOTE_E6,NOTE_CS6,NOTE_B5,NOTE_A5,NOTE_GS5};
int songLength = sizeof(yiruma);
void loop() {
  for (int i = 0;i<29;i++){
    tone(buzzerPin,yiruma[i],bpm);
    delay(250);
  }
  delay(2000);
}
