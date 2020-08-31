
#include "Arduino.h"
//Instantiate a new advanced serial object with 2 transmit slots

float theta;
float theta_prev = 0;
float omega;
float t=0;
float t_prev;
int iterations = 0;
float delta_theta;
volatile float counter = 0; //This variable will increase or decrease depending on the rotation of encoder


void setup() {
  Serial.begin (9600);

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
   //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}


  //DATA RECORDING  
  //Serial.print(t);
  //Serial.print(" ");
  //Serial.print(theta);
  //Serial.print(" ");
  //Serial.println(omega);

void loop() {
  t = float(millis())/1000;
  theta = counter*360/2048;
  if (iterations>0){
    delta_theta = (theta-theta_prev)*PI/(180);
    omega = delta_theta/(t-t_prev);
  }

  //SERIAL PLOTTING
  
  Serial.print(theta);
  Serial.print(" ");
  Serial.println(omega);
  
  theta_prev=theta;
  t_prev = t;
  iterations = iterations+1;
}
   
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
}}
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++; }}
