#include <CrsfSerial.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <math.h>

#define GPSSerial Serial1
#define GPSECHO true
#define IMUECHO false
#define HMCECHO false
#define ELRSECHO false

uint32_t timer1 = millis();
uint32_t timer2 = millis();

SerialPIO Receiver(10, 11);
CrsfSerial crsf(Receiver, 200000);
Servo Aileron, Elevator, Motor, Rudder;
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag;
Adafruit_GPS GPS(&GPSSerial);

float scaler, xv, yv, zv, calibrated_values[3], normal_vector_length, latitud, longitud, latitudDegs, longitudDegs, latObj=32.632648, lonObj=-115.444410, distObj, giroObj;
boolean scaler_flag = false;
sensors_event_t event;

void setup(){
  Serial.begin(9600);

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
  pinMode(6,OUTPUT_12MA);
  pinMode(7,OUTPUT_12MA);
  

  Serial.println("Init MPU");
  Wire.begin();
  if (!mpu.begin()) {
  Serial.println("Failed to initialize MPU6050 chip");
    while (1) {
    }
  }
  mpu.setI2CBypass(true);
  //mpu.setGyroStandby(false, false, false);
  //mpu.enableSleep(false);

  Serial.println("Init HMC");
  if(!mag.begin()){
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Failed to initialize HMC5883L chip");
    while(1);
    }
  Aileron.writeMicroseconds(1500);
  Elevator.writeMicroseconds(1500);
  Motor.writeMicroseconds(1000);
  Rudder.writeMicroseconds(1500);
  }

void setup1(){
  Serial.println("Init GPS");
  GPS.begin(9600);

  Serial.print("Init LoRaCo");
  Serial2.begin(9600);
}

void loop(){
  control();
  if (millis() - timer1 > 20) {
    timer1 = millis();
    brujula();
    gyroaccel();
  }
}

void loop1(){
  gps();
  telemetry();
}

void control(){
  crsf.loop();
  if(crsf.getChannel(7)>1900){
    Aileron.writeMicroseconds(crsf.getChannel(1));
    Elevator.writeMicroseconds(crsf.getChannel(2));
    Motor.writeMicroseconds(crsf.getChannel(3));
    Rudder.writeMicroseconds(crsf.getChannel(4));
  }
  else{
    Aileron.writeMicroseconds(1500);
    Elevator.writeMicroseconds(1500);
    Motor.writeMicroseconds(1000);
    Rudder.writeMicroseconds(1500);
    //Serial.println("ELRS Inactive");
  }
  if (crsf.getChannel(5)>1500){
    digitalWrite(6,LOW);
  }
  else{
    digitalWrite(6,HIGH);
  }
  if (crsf.getChannel(6)>1500){
    digitalWrite(7,LOW);
  }
  else{
    digitalWrite(7,HIGH);
  }
  if (ELRSECHO){
  Serial.print(crsf.getChannel(1));
  Serial.print(" ");
  Serial.print(crsf.getChannel(2));
  Serial.print(" ");
  Serial.print(crsf.getChannel(3));
  Serial.print(" ");
  Serial.print(crsf.getChannel(4));
  Serial.print(" ");
  Serial.print(crsf.getChannel(5));
  Serial.print(" ");
  Serial.print(crsf.getChannel(6));
  Serial.print(" ");
  Serial.println(crsf.getChannel(7));
  }
}

void gyroaccel(){
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

void brujula(){
  getMagData();  

  float heading = atan2(yv, xv);
  float declinationAngle = 0.185;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  float headingDegrees = heading * 180/M_PI; 
  
  if (HMCECHO){
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  }
}

void gps(){
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
      Serial.print("Location (DD MM): "); 
      Serial.print(GPS.latitude, 6); Serial.print(GPS.lat); Serial.print(", ");
      Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);
      latitud = GPS.latitude;
      longitud = GPS.longitude;
      convCords();
      Serial.print("Location (DD): "); 
      Serial.print(latitudDegs, 6); Serial.print(", ");
      Serial.println(longitudDegs, 6);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      distObj = objetivo(latitudDegs, longitudDegs, latObj, lonObj);
      Serial.print("Distance to objective: "); Serial.println(distObj);
      Serial.print("Degrees to objective: "); Serial.println(giroObj);
      Serial.println(""); 
    }
  }
}

void telemetry(){
  Serial2.println("Testing");
}

void convCords(){
 //Initialize the location.
 float lat = latitud;
 float lon = longitud;
 // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
 int firsttwoLat = ((int)lat)/100; //This assumes that f < 10000.
 int firsttwoLon = ((int)lon)/100; //This assumes that f < 10000.
 float nexttwoLat = lat - (float)(firsttwoLat*100);
 float nexttwoLon = lon - (float)(firsttwoLon*100);
 latitudDegs=(float)(firsttwoLat + nexttwoLat/60.0);
 longitudDegs=-(float)(firsttwoLon + nexttwoLon/60.0);
}

float objetivo(float lat1, float lon1, float lat2, float lon2) { 
  
  // Radio de la Tierra en metros
  const float R = 6371000;

  // Convertimos las coordenadas a radianes
  lat1 = lat1 * M_PI / 180;
  lon1 = lon1 * M_PI / 180;
  lat2 = lat2 * M_PI / 180;
  lon2 = lon2 * M_PI / 180;

  // Calculamos la diferencia entre las coordenadas
  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;

  // Aplicamos la fórmula de Haversine
  float a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  float d = R * c;

  return d; //Mandamos la distancia de regreso

  //Cálculo de ángulo con respecto al origen
  giroObj=(atan2(dLat,dLon));
  if (giroObj < 0){
    giroObj += 2 * PI;
  }
  if (giroObj > 2 * PI){
    giroObj -= 2 * PI;
  }
  giroObj = giroObj * 180/M_PI;
}

void getMagData(){
  mag.getEvent(&event);

  xv=event.magnetic.x;
  yv=event.magnetic.y;
  zv=event.magnetic.z;
}

void transformation(float uncalibrated_values[3])    {
  double calibration_matrix[3][3] = {
    {1.315,-0.036,-0.115},
    {0.033,1.358,0.008},
    {0.150,0.022,1.509}  
  };

  double bias[3] = {
    -8.387,
    -2.62,
    -15.539
  };  

  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

void vector_length_stabilasation(){
  if (scaler_flag == false){
    getMagData();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 

  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}