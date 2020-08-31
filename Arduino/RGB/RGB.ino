#define red 6
#define blue 3
#define green 5

void setup(){
  pinMode(red,OUTPUT);
  pinMode(blue,OUTPUT);
  pinMode(green,OUTPUT);
}

void loop(){
  int redVal = 0;
  int blueVal = 255;
  int grnVal = 0;
  for (int i=0;i<255;i++){
    redVal++;
    blueVal--;
    analogWrite(red,redVal);
    analogWrite(blue,blueVal);
    analogWrite(green,grnVal);
    delay(100);
  }
}

