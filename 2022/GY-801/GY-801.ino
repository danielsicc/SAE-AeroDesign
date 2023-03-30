/*
  Create on May 19, 2021
  Create by MohammedDamirchi base of https://github.com/DMohammed/GY-801
  https://electropeak.com/learn/
*/

#include <Wire.h>
#include "HMC5883L.h"
#include <I2Cdev.h>
#include "L3G4200D.h"
#include "ADXL345.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP085.h"

L3G4200D gyro;
ADXL345 accel;
HMC5883L MMC5883;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

int16_t ax, ay, az;
int16_t avx, avy, avz;
double baseline;

void setup()
{
  Serial.begin(9600);
  Serial.println("Hello");
  Wire.begin();
  gyro.initialize();
  accel.initialize();
  gyro.setFullScale(2000);
  bmp.begin();
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}

void loop()
{

    Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For FACULTAD DE INGENIERÍA UABC, MEXICALI, BAJA CALIFORNIA declination angle is +10° 45'E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (10 + (45 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

  delay(100);

  Serial.print("     ");
  gyro.getAngularVelocity(&avx, &avy, &avz);

  Serial.print("gyro X:");
  Serial.print(avx);
  Serial.print("\tY:");
  Serial.print(avy);
  Serial.print("\tZ:");
  Serial.print(avz);

  accel.getAcceleration(&ax, &ay, &az);
  Serial.print("\taccel X:");
  Serial.print(ax);
  Serial.print("\tY:");
  Serial.print(ay);
  Serial.print("\tZ:");
  Serial.print(az);

  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("\tPressure:");
    Serial.print(event.pressure);
    Serial.print(" hPa");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = 1018;
    Serial.print("\tAltitude:");
    Serial.println(bmp.pressureToAltitude(seaLevelPressure,event.pressure));
  }
  delay(200);
}
