import controlP5.*;

ControlP5 cp5;
PFont font;
PFont titleFont;

void setup(){
  size(250,400);
  
  font = createFont("calibri light",20);

  cp5 = new ControlP5(this);
  cp5.addButton("Start").setPosition(50,75).setSize(150,40).setFont(font);
  cp5.addButton("Stop").setPosition(50,150).setSize(150,40).setFont(font);
  cp5.addButton("Position").setPosition(50,225).setSize(150,40).setFont(font);
  cp5.addButton("Velocity").setPosition(50,300).setSize(150,40).setFont(font);
  
}

void draw(){
  background(125,0,125);
  fill(255,255,255);
  titleFont = createFont("calibri light",25);
  textFont(titleFont);
  text("Angle Sensor",50,50);
}
  
  
