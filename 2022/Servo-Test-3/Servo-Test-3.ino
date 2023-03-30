#include <IBusBM.h>
#include <Servo.h>

int canales[5],killswitch,flaps,anguloFlaps;
IBusBM IBus; // IBus object
Servo aa,ab,ba,bb,c,d;  // create servo object to control a servo

void setup() {
  
  Serial1.begin(115200);   // remove comment from this line if you change the Serial port in the next line
  IBus.begin(Serial1);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required

  aa.attach(40); // Alerones
  ab.attach(41); // Alerones
  ba.attach(42); //Elevadores
  bb.attach(43); //Elevadores
  c.attach(44); //Motores
  d.attach(45); //Timon
}

void loop() {
  killswitch=IBus.readChannel(4);
  flaps=IBus.readChannel(5);
  
  if(killswitch>1900){
  anguloFlaps=map(flaps,1000,2000,0,500);
  
  canales[0]=IBus.readChannel(0);
  aa.writeMicroseconds(canales[0]+anguloFlaps);
  ab.writeMicroseconds(canales[0]-anguloFlaps);
  
  canales[1]=IBus.readChannel(1);
  ba.writeMicroseconds(canales[1]);
  bb.writeMicroseconds(canales[1]);
  
  canales[2]=IBus.readChannel(2);
  c.writeMicroseconds(canales[2]);
  
  canales[3]=IBus.readChannel(3);
  d.writeMicroseconds(canales[3]);
  }
  
  else{
    aa.writeMicroseconds(1500);
    ab.writeMicroseconds(1500);
    ba.writeMicroseconds(1500);
    bb.writeMicroseconds(1500);
    c.writeMicroseconds(1000);
    d.writeMicroseconds(1500);
  }
}
