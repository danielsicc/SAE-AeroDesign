/*Código realizado para el concurso de SAE Aerodesign West 2022 Categoría Avanzada
 * donde se controla el avión pequeño por medio de un transmisor y receptor FlySky
 * Por Jesús Daniel Sicairos López y Eduardo Alejandro Saucedo Salinas.
 */

#include <IBusBM.h>
#include <Servo.h>

int canales[5],killswitch;
IBusBM IBus; // IBus object
Servo aa,ab,b,c,d;  // create servo object to control a servo

void setup() {
  
  //Serial1.begin(115200);   // remove comment from this line if you change the Serial port in the next line
  IBus.begin(Serial);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required

  aa.attach(3); // Alerones
  ab.attach(4); // Alerones
  b.attach(5); //Elevadores
  c.attach(6); //Motores
  d.attach(8); //Timon
}

void loop() {
  killswitch=IBus.readChannel(4);
  
  if(killswitch>1900){
  
  canales[0]=IBus.readChannel(0);
  aa.writeMicroseconds(canales[0]);
  ab.writeMicroseconds(canales[0]);
  
  canales[1]=IBus.readChannel(1);
  b.writeMicroseconds(canales[1]);
  
  canales[2]=IBus.readChannel(2);
  c.writeMicroseconds(canales[2]);
  
  canales[3]=IBus.readChannel(3);
  d.writeMicroseconds(canales[3]);
  }
  
  else{
    aa.writeMicroseconds(1500);
    ab.writeMicroseconds(1500);
    b.writeMicroseconds(1500);
    c.writeMicroseconds(1000);
    d.writeMicroseconds(1500);
  }
}
