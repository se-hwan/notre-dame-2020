#include "LedControl.h"

LedControl lc=LedControl(12,10,11,1);
/* pin 12 = data pin
 *  pin 10 = clock pin
 *  pin 11 = load pin
 *  1 = number of devices
 */

unsigned long delaytime1=500;
unsigned long delaytime2=50;
int lcAddress = 0;
int sai = 4;
int nd = 8;
int smiley = 9;


void setup() {
  /*initiate displlay*/
  lc.shutdown(lcAddress,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(lcAddress,6);
  /* and clear the display */
  lc.clearDisplay(lcAddress);
  pinMode(sai,INPUT_PULLUP);
  pinMode(nd,INPUT_PULLUP);
  pinMode(smiley,INPUT_PULLUP);

  Serial.begin(9600);
}

void myName() {
  byte sai[8] = {B00000000,B0000000,B11011101,B10010101,B11011101,B01010101,B11010101,B00000000};
  for (int i=0;i<8;i++){
    lc.setRow(lcAddress,i,sai[i]);
    delay(100);
  }
}


void ND() {
  byte logo[8] = {B00000000,B00100100,B11111111,B10110101,B10101101,B11111111,B00100100,B00000000};
  for (int i=0;i<8;i++){
    lc.setRow(lcAddress,i,logo[i]);
    delay(100);
  }
}

void wholesome() {
  byte smile[8] = {B00000000,B00000000,B01100110,B01100110,B00000000,B10000001,B01000010,B00111100};
  for (int i=0;i<8;i++){
    lc.setRow(lcAddress,i,smile[i]);
    delay(100);
  }
}

void loop() {
  if(digitalRead(smiley)==LOW){
    wholesome();
    lc.clearDisplay(lcAddress);
  }
  if(digitalRead(nd)==LOW){
    ND();
    lc.clearDisplay(lcAddress);
  }
  if(digitalRead(sai)==LOW){
    myName();
    lc.clearDisplay(lcAddress);
  }
}
