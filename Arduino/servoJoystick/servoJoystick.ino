#include <Servo.h>
#define joy A0

Servo servo1;
int servo1Pin = 3;
int adVal;
int scaled;
int degree;

void setup() {
  pinMode(joy,INPUT);
  Serial.begin(57600);
  servo1.attach(servo1Pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  adVal = analogRead(joy);
  scaled = map(adVal,0,1023,-100,100);
  degree = map(scaled,-100,100,0,180);
  Serial.print("Analog:");
  Serial.println(adVal);
  Serial.print("Scaled:");
  Serial.println(scaled);
  Serial.print("Degree:");
  Serial.println(degree);
  servo1.write(degree);
  delay(25);
}
