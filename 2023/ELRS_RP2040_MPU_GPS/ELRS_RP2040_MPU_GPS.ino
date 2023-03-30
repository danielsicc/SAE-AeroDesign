#include <CrsfSerial.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Wire.h>

#define GPSSerial Serial1
#define GPSECHO false
#define IMUECHO true

int counter=0;
uint32_t timer1 = millis();
uint32_t timer2 = millis();

SerialPIO Receiver(10, 11);
CrsfSerial crsf(Receiver, 200000);
Servo Aileron, Elevator, Motor, Rudder;
Adafruit_MPU6050 mpu;
Adafruit_GPS GPS(&GPSSerial);

void setup(){
    Serial.begin(115200);

    Serial.println("Init ELRS");
    Receiver.begin(200000);
    Aileron.attach(29);
    Elevator.attach(28);
    Motor.attach(27);
    Rudder.attach(26);
    Aileron.writeMicroseconds(1500);
    Elevator.writeMicroseconds(1500);
    Motor.writeMicroseconds(1000);
    Rudder.writeMicroseconds(1500);
    

    Serial.println("Init MPU");
    Wire.begin();
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
}

void setup1(){
  Serial.println("Init GPS");
  GPS.begin(9600);

  Serial.print("Init LoRaCo");
  Serial2.begin(115200);
}

void loop(){
    // Must call CrsfSerial.loop() in loop() to process data
    crsf.loop();
    Aileron.writeMicroseconds(crsf.getChannel(1));
    Elevator.writeMicroseconds(crsf.getChannel(2));
    Motor.writeMicroseconds(crsf.getChannel(3));
    Rudder.writeMicroseconds(crsf.getChannel(4));

  if (millis() - timer1 > 50) {
      timer1 = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (IMUECHO){
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    }
  }
}

void loop1(){
    char c = GPS.read();
    if (GPSECHO){
      if (c) Serial.print(c);
    }
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))
        return;
    }

    if (millis() - timer2 > 1000) {
      timer2 = millis();
      Serial.println("");
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" Quality: "); Serial.print((int)GPS.fixquality);
      Serial.print(" Satellites: "); Serial.println((int)GPS.satellites);
      if (GPS.fix) {
        Serial.print("Location: "); 
        Serial.print(GPS.latitude, 6); Serial.print(GPS.lat); Serial.print(", ");
        Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.println("");

        Serial2.print("Testing LoRa"); 
        Serial2.println(counter);
        counter++;   
      }
    }

    
}