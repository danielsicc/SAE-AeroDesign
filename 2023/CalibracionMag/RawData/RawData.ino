#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

sensors_event_t event; 
float xv, yv, zv;
float calibrated_values[3];

void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {1.33,0.021,-0.243},
    {0.032,1.366,0.017},
    {0.077,-0.081,1.471}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    -22.891,
    -109.252,
    -248.098
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

//vector_length_stabilasation() - is the function of the magnetometer vector length stabilasation (stabilisation of the sphere radius)
float scaler;
boolean scaler_flag = false;
float normal_vector_length;
void vector_length_stabilasation(){
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    getData();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //calculate the current scaler
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

void setup(void){
  Serial.begin(9600);

  Wire.begin();
  if (!mpu.begin()) {
  Serial.println("Failed to initialize MPU6050 chip");
    while (1) {
    }
  }
  mpu.setI2CBypass(true);
  if(!mag.begin()){
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
}

void loop(void) {
  float values_from_magnetometer[3];
  getData();
 
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);
  
  vector_length_stabilasation();
  
  Serial.flush(); 
  Serial.print(event.magnetic.x);
  Serial.print(",");
  Serial.print(event.magnetic.y);
  Serial.print(",");
  Serial.print(event.magnetic.z);
  Serial.println();

  delay(100);
}

void getData(){
  mag.getEvent(&event);

  xv=event.magnetic.x;
  yv=event.magnetic.y;
  zv=event.magnetic.z;
}