#include "SoftwareSerial.h"
#include "Wire.h"
#include "HMC5883L.h"

HMC5883L compass;

SoftwareSerial GPS(7,-1); //Rx, Tx
const int tamano=80, anguloMag=180;
float lat, lon, latMeta=32.379589, lonMeta=115.266624;
int sat, cuadrantes;
char sentencia[tamano];
double referencePressure;
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
    getHeading();
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

void setup()
{
  Serial.begin(9600);
  GPS.begin(9600);
  Wire.begin();
  compass.begin();
  magSettings();
}

void loop()
{
  static int i=0;
  if(GPS.available())
  {
    Serial.println("hola");
    char dato=GPS.read();
    if (dato!='\n'&&i<tamano)
    {
      sentencia[i]=dato;
      i++;
    }
    else
    {
      sentencia[i]='\0';
      i=0;
      mostrar();
    }
  }
}

void mostrar()
{
  char campo[20];
  obtener_campo(campo,0);
  if (strcmp(campo,"$GPGGA")==0)
  {
    //LONGITUD
    obtener_campo(campo,4);
    lon=atof(campo)/100;
    Serial.print(lon,6);
    Serial.print(",");
    
    //LATITUD
    obtener_campo(campo,2);
    lat=atof(campo)/100;
    Serial.print(lat,6);
    Serial.print(",");

    //NO_SATÉLITES
    obtener_campo(campo,7);
    sat=atof(campo);
    Serial.println(sat);

    cuadrante();
    angulo();
   
  }
}

void obtener_campo(char* buffer,int indice)
{
  int posicion=0, posicion_campo=0, comas=0;
  while(posicion<tamano)
  {
    if(sentencia[posicion]==',')
    {
      comas++;
      posicion++;
    }
    if(comas==indice)
    {
      buffer[posicion_campo]=sentencia[posicion];
      posicion_campo++;
    }
    posicion++;
  }
  buffer[posicion_campo]='\0';
}

void cuadrante()
{ 
  if(latMeta<lat && lonMeta>lon){     
   Serial.println("1er cuadrante"); 
   cuadrantes=1;  
   }
  if(latMeta<lat && lonMeta<lon){     
   Serial.println("2do cuadrante");
   cuadrantes=2;  
    }
  if(latMeta>lat && lon>lonMeta){     
   Serial.println("3ro cuadrante"); 
   cuadrantes=3; 
 }
  if(lat<latMeta && lon<lonMeta){     
   Serial.println("4to cuadrante"); 
   cuadrantes=4; 
   }   
}

void angulo()
{
  calculateHeading();
  //switch(cuadrantes){
    //case 1:
      
    //case 2:
      
    //case 3:
      
    //case 4:
      
  //}
}

void magSettings()
{
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  checkSettings();
}

void checkSettings()
{
  Serial.print("Selected range: ");
  
  switch (compass.getRange())
  {
    case HMC5883L_RANGE_0_88GA: Serial.println("0.88 Ga"); break;
    case HMC5883L_RANGE_1_3GA:  Serial.println("1.3 Ga"); break;
    case HMC5883L_RANGE_1_9GA:  Serial.println("1.9 Ga"); break;
    case HMC5883L_RANGE_2_5GA:  Serial.println("2.5 Ga"); break;
    case HMC5883L_RANGE_4GA:    Serial.println("4 Ga"); break;
    case HMC5883L_RANGE_4_7GA:  Serial.println("4.7 Ga"); break;
    case HMC5883L_RANGE_5_6GA:  Serial.println("5.6 Ga"); break;
    case HMC5883L_RANGE_8_1GA:  Serial.println("8.1 Ga"); break;
    default: Serial.println("Bad range!");
  }
  
  Serial.print("Selected Measurement Mode: ");
  switch (compass.getMeasurementMode())
  {  
    case HMC5883L_IDLE: Serial.println("Idle mode"); break;
    case HMC5883L_SINGLE:  Serial.println("Single-Measurement"); break;
    case HMC5883L_CONTINOUS:  Serial.println("Continuous-Measurement"); break;
    default: Serial.println("Bad mode!");
  }

  Serial.print("Selected Data Rate: ");
  switch (compass.getDataRate())
  {  
    case HMC5883L_DATARATE_0_75_HZ: Serial.println("0.75 Hz"); break;
    case HMC5883L_DATARATE_1_5HZ:  Serial.println("1.5 Hz"); break;
    case HMC5883L_DATARATE_3HZ:  Serial.println("3 Hz"); break;
    case HMC5883L_DATARATE_7_5HZ: Serial.println("7.5 Hz"); break;
    case HMC5883L_DATARATE_15HZ:  Serial.println("15 Hz"); break;
    case HMC5883L_DATARATE_30HZ: Serial.println("30 Hz"); break;
    case HMC5883L_DATARATE_75HZ:  Serial.println("75 Hz"); break;
    default: Serial.println("Bad data rate!");
  }
  
  Serial.print("Selected number of samples: ");
  switch (compass.getSamples())
  {  
    case HMC5883L_SAMPLES_1: Serial.println("1"); break;
    case HMC5883L_SAMPLES_2: Serial.println("2"); break;
    case HMC5883L_SAMPLES_4: Serial.println("4"); break;
    case HMC5883L_SAMPLES_8: Serial.println("8"); break;
    default: Serial.println("Bad number of samples!");
  }
}

 void calculateHeading()
 {
   float values_from_magnetometer[3];
  
  getHeading();
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);
  
  vector_length_stabilasation();

  float heading = atan2(calibrated_values[1],calibrated_values[0]); 

  float declinationAngle = (10 + (45.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle - (PI/2);

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
  Serial.print("  ");
} 
 
void getHeading()
{ 
  Vector raw = compass.readRaw();
  xv=raw.XAxis;
  yv=raw.YAxis;
  zv=raw.ZAxis;
}
 
