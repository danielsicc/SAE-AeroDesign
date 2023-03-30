#include <SoftwareSerial.h> 

SoftwareSerial GPS_Serial(7, -11); // RX, TX

void setup() {
  Serial.begin(9600);
  GPS_Serial.begin(9600); 
}

void loop() {
   char rc;

   if (GPS_Serial.available()){
        rc = GPS_Serial.read();
        Serial.write(rc);
   }
}
