#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  Serial.begin(9600);
  pinMode(3,INPUT);
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  int val;
  val = pulseIn(3,HIGH);
  Serial.println(val);
  myservo.writeMicroseconds(val);   // sets the servo position 
  //delay(10);
}
