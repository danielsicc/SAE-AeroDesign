#include <IBusBM.h>
#include <Servo.h>

float canales[5],killswitch,flaps,flapsMap,alerones,aleronesMap,elevadorInv,elevadorInvMap;
IBusBM IBus; // IBus object
Servo aa,ab,ba,bb,c,d,e;  // create servo object to control a servo

void setup() {
   
  // Serial.begin(115200);   // remove comment from this line if you change the Serial port in the next line
  IBus.begin(Serial);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required

  aa.attach(40); // Alerones
  ab.attach(41); // Alerones
  ba.attach(42); //Elevadores
  bb.attach(43); //Elevadores
  c.attach(44); //Motores
  d.attach(45); //Timon
  e.attach(46);//Dirección
}

void loop() {
  killswitch=IBus.readChannel(4);
  alerones=IBus.readChannel(0);
  aleronesMap=map(alerones,1000,2000,2000,1000);
  elevadorInv=IBus.readChannel(1);
  elevadorInvMap=map(elevadorInv,1000,2000,2000,1000);
  flaps=IBus.readChannel(5);
  flapsMap=map(flaps,1000,2000,0,500);
  
  if(killswitch>1900){
  
  aa.writeMicroseconds(aleronesMap-flapsMap);
  ab.writeMicroseconds(aleronesMap+flapsMap  );

  canales[1]=IBus.readChannel(1);
  ba.writeMicroseconds(elevadorInvMap);
  bb.writeMicroseconds(canales[1]);
  
  canales[2]=IBus.readChannel(2);
  c.writeMicroseconds(canales[2]);
  
  canales[3]=IBus.readChannel(3);
  d.writeMicroseconds(canales[3]);
  e.writeMicroseconds(canales[3]);
  }
  
  else{
    aa.writeMicroseconds(1500);
    ab.writeMicroseconds(1500);
    ba.writeMicroseconds(1500);
    bb.writeMicroseconds(1500);
    c.writeMicroseconds(1000);
    d.writeMicroseconds(1500);
    e.writeMicroseconds(1500);
  }
}
