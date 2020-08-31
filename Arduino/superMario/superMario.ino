#include "pitches.h"

int buzzerPin = 7;
int ledLow = 5;
int ledM1 = 4;
int ledM2 = 3;
int ledHigh = 2;
void setup() {
  pinMode(buzzerPin,OUTPUT);
  pinMode(ledLow,OUTPUT);
  pinMode(ledM1,OUTPUT);
  pinMode(ledM2,OUTPUT);
  pinMode(ledHigh,OUTPUT);
}


int intro[] = {NOTE_E6,NOTE_E6,NOTE_E6,NOTE_C6,NOTE_E6,NOTE_G6,NOTE_G5};
int beatLength = 225;
int introTempo[] = {150,300,300,150,300,600,600}; 
int chorus1[] = {NOTE_C6,NOTE_G5,NOTE_E5,NOTE_A5,NOTE_B5,NOTE_AS5,NOTE_A5,NOTE_G5,NOTE_E6,NOTE_G6,NOTE_A6,NOTE_F6,NOTE_G6,NOTE_E6,NOTE_C6,NOTE_D6,NOTE_B5};
int chorus1Tempo[] = {450,450,450,300,300,150,300,200,200,200,300,150,300,300,150,150,450};
int chorus2[] = {NOTE_G6,NOTE_FS6,NOTE_F6,NOTE_DS6,NOTE_E6,NOTE_GS5,NOTE_A5,NOTE_C6,NOTE_A5,NOTE_C6,NOTE_D6,NOTE_G6,NOTE_FS6,NOTE_F6,NOTE_DS6,NOTE_E6,NOTE_C7,NOTE_C7,NOTE_C7};
int chorus2Tempo[] = {150,150,150,300,300,150,150,300,150,150,450,150,150,150,300,300,300,150,600};
int chorus3[] = {NOTE_G6,NOTE_FS6,NOTE_F6,NOTE_DS6,NOTE_E6,NOTE_GS5,NOTE_A5,NOTE_C6,NOTE_A5,NOTE_C6,NOTE_D6,NOTE_DS6,NOTE_D6,NOTE_C6};
int chorus3Tempo[] = {150,150,150,300,300,150,150,300,150,150,450,450,450,1350};
int chorus4[] = {NOTE_C6,NOTE_C6,NOTE_C6,NOTE_C6,NOTE_D6,NOTE_E6,NOTE_C6,NOTE_A5,NOTE_G5,NOTE_C6,NOTE_C6,NOTE_C6,NOTE_C6,NOTE_D6,NOTE_E6,NOTE_C6,NOTE_C6,NOTE_C6,NOTE_C6,NOTE_D6,NOTE_E6,NOTE_C6,NOTE_A5,NOTE_G5,NOTE_E6,NOTE_E6,NOTE_E6,NOTE_C6,NOTE_E6,NOTE_G6};
int chorus4Tempo[] = {150,300,300,150,300,150,300,150,600,150,300,300,150,150,1500,150,300,300,150,300,150,300,150,600,150,300,300,150,300,1350};

void lit(int note){
  if(note>=NOTE_C5 && note<=NOTE_F5){
    digitalWrite(ledLow,HIGH);
    delay(10);
  }
  else if(note>=NOTE_G5 && note<=NOTE_C6){
    digitalWrite(ledM1,HIGH);
    delay(10);
  }
  else if(note>=NOTE_D6 && note<=NOTE_F6){
    digitalWrite(ledM2,HIGH);
    delay(10);
  }
  else if(note>=NOTE_G6 && note<=NOTE_C7){
    digitalWrite(ledHigh,HIGH);
    delay(10);
  }
  digitalWrite(ledLow,LOW);
  digitalWrite(ledM1,LOW);
  digitalWrite(ledM2,LOW);
  digitalWrite(ledHigh,LOW);
}

void loop() {
  for (int i = 0;i<7;i++){
    tone(buzzerPin,intro[i],beatLength);
    lit(intro[i]);
    delay(introTempo[i]);
  }
  for (int j=0;j<2;j++){
    for(int k =0;k<17;k++){
      tone(buzzerPin,chorus1[k],beatLength);
      lit(chorus1[k]);
      delay(chorus1Tempo[k]);
    }
  }
  delay(450);
  for(int l =0;l<19;l++){
      
      tone(buzzerPin,chorus2[l],beatLength);
      lit(chorus2[l]);
      delay(chorus2Tempo[l]);
    }
  delay(450);
  for(int m=0;m<14;m++){
      
      tone(buzzerPin,chorus3[m],beatLength);
      lit(chorus3[m]);
      delay(chorus3Tempo[m]);
    }
  for(int n=0;n<(sizeof(chorus4)/sizeof(chorus4[0]));n++){
      
      tone(buzzerPin,chorus4[n],beatLength);
      lit(chorus4[n]);
      delay(chorus4Tempo[n]);
    }
}
