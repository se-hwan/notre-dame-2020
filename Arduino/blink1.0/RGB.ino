#define red 6
#define blue 3
#define green 5

void setup(){
  pinMode(red,OUTPUT);
  pinMode(blue,OUTPUT);
  pinMode(green,OUTPUT);
}

void loop(){
  int redVal = 255;
  int blueVal = 255;
  int grnVal = 255;
  analogWrite(red,redVal);
  analogWrite(blue,blueVal);
  analogWrite(green,grnVal);
}

