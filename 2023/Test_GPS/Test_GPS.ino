#include <NMEAGPS.h>
#include <GPSport.h>
#include <SPI.h>
#include <LoRa.h>
  

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

float lat=1, latHome, lon=1, lonHome, alt=1;
bool band=false;
int ch5;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.print( F("Started\n") );

  gpsPort.begin(9600);
  pinMode(37, INPUT);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

//--------------------------

void loop(){
  while (gps.available( gpsPort )) {
    fix = gps.read();

    if (fix.valid.location) {
      Serial.print( fix.latitude(), 6 );
      Serial.print( ',' );
      Serial.print( fix.longitude(), 6 );
    }
    else {
      Serial.println("Non-valid Location");
    }
  }
   ch5 = pulseIn(37, HIGH);
   
  if(ch5>1500&&band==false){
    latHome=fix.latitude(), 7;
    lonHome=fix.longitude(), 7;
    band=true;
  }
  else{
    band=false;
  }

  if(band==true){
    Serial.print("LatHome= ");
    Serial.print(latHome,7);
    Serial.print(" LonHome= ");
    Serial.println(lonHome,7);
    LoRa.beginPacket();
    LoRa.print("LatHome= ");
    LoRa.print(latHome,7);
    LoRa.print(" LonHome= ");
    LoRa.println(lonHome,7);
    LoRa.endPacket();
  }
}
