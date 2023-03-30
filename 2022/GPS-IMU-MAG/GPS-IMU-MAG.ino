/* ESTÉ CÓDIGO FUÉ REALIZADO POR EL Dr. JOSÉ MANUEL RAMÍREZ ZARATE, EDUARDO SAUCEDO y DANIEL SICAIROS CON EL OBJETIVO DE REALIZAR PRUEBAS PARA EL REGRESO A CASA DE UN ROVER
 *  TOMANDO COMO REFERENCIA UNA POSICIPON INICIAL EN REFERENCIA A LATITUD Y LONGITUD, POSTERIORMENTE Y DESPUÉS DE MOVER EL ROVER A UNA POSICIÓN
 *  FINAL, ÉSTE CALCULA EL ÁNGULO DE GIRO PARA APUNTAR HACIA E PUNTO DE ORIGEN  COMENZAR EL CAMINO HACIA ÉSTE!!!*/

//GPS
#include <Wire.h>
#include <ADXL345.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

//GY
#include "HMC5883L.h" 
#include "L3G4200D.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP085_U.h"

//RC
#include <Servo.h>

//GPS
SoftwareSerial gpsSerial(3,-1);
ADXL345 accelerometer;

//GY
L3G4200D gyro;
ADXL345 accel;
HMC5883L MMC5883;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

//RC
Servo A,B;
int channel1=4,channel2=5,channel5=6,ch1=0,ch2=0,ch5=0,aa;

//GPS
const int sentenceSize=80;
const int PWM_A_Pin = -9;                         // PWM se conecta al pin 1 del puente-H
const int motor_izq1 = -8;                        // Entrada 2 del puente-H 
const int motor_izq2 = -4;                        // Entrada 7 del puente-H 
const int PWM_B_Pin = -5;                         // PWM se conecta al pin 1 del puente-H
const int motor_der1 = -6;                        // Entrada 2 del puente-H 
const int motor_der2 = -7;                        // Entrada 7 del puente-H 

//GY
int16_t ax, ay, az;
int16_t avx, avy, avz;
double baseline;

int numero_sat=5;
int a=0,band=0,sat=0,band2=0,band3=0,a1=0;
int velocidad;                                   // Establece la velocidad
int pasos;                                       // Establece el tiempo en milisegundos de espera
int p3=0;
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;
int offZ = 0;
int minZ=0;
int maxZ=0;
int distancia0=0;
int band4=0;

boolean scaler_flag = false;
float calibrated_values[3];   
float xv, yv, zv;
float scaler;
float normal_vector_length;
float xx, yy, zz;
float heading1;
float heading2;
float alp3,distancia;
//float tiltCompensate(float xx, float yy, float zz, Vector normAccel);
float correctAngle(float heading);
float correctAngle(float heading);
float noTiltCompensate(float xx,float yy,float zz);

char sentence[sentenceSize];
String msj;

double la,la1,lo,lo1, co, ca;
double total=0,total1=0,pi=3.1416;
double annn,alp0,alp1,alp2,cad1,cop1;

void cuadrante(void);
int angulo(void);
void  calibrate(void);
void mag();
void transformation(float uncalibrated_values[3]) ;
void vector_length_stabilasation();
void movimiento();
void adelante();
void atras();
void parar();
void izq();
void der();
void HOME();
void point();
void getHeading();
void grados2();

void calibrate();


void setup() 
{
  pinMode(motor_izq1, OUTPUT);                     //Configura motor_izq1 como salida 
  pinMode(motor_izq2, OUTPUT);                     //Configura motor_izq2 como salida 
  pinMode(motor_der1, OUTPUT);                     //Configura motor_der1 como salida 
  pinMode(motor_der2, OUTPUT);                     //Configura motor_der2 como salida 
  Serial.begin(9600);
  gpsSerial.begin(9600);

//calibrate();
 // compass.setOffset(offX, offY, offZ); 
  LoRa.begin(420E6);

  //GY
  Wire.begin();
  gyro.initialize();
  accel.initialize();
  gyro.setFullScale(2000);
  bmp.begin();
  MMC5883.begin();
  //MMC5883.calibrate();

  //RC
    A.attach(9);
    B.attach(10);
    pinMode(channel1,INPUT);
    pinMode(channel2,INPUT);
    pinMode(channel5,INPUT);
}
void loop() {
  if(!band3){
  point();
  }
  else{
  gpss();
  }
}

void RC(){
  ch5=pulseIn(channel5,HIGH);
    Serial.print("Killswitch: "); Serial.println(ch5); Serial.println();
    if(ch5>1900){
        ch1=pulseIn(channel1,HIGH);
        ch2=pulseIn(channel2,HIGH);    
        Serial.println(ch1);
        Serial.println(ch2); Serial.println();
        B.writeMicroseconds(ch1);
        if(ch2<1450||ch2>1550){
            aa=map(ch2,1000,2000,0,180);
            A.write(aa);
            //delay(50);
        }
        else{
            A.write(90);   
        }
    }
}


void GY(){
  Serial.print(compass.readNormalize());

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
}

void point(){
  static int i = 0;
  do  {
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    if (ch != '\n' && i < sentenceSize)
    {      sentence[i] = ch;
      i++;    }
    else
    {   float lat, lon;
  int dd;
  float ss;
  char field[20];
   int grados=0,minutos=0;
  double a=0,b=0,c=0,segundos=0;
  getField(field,0);
  if (strcmp(field, "$GPGGA") == 0)
  { 
 getField(field, 7);

sat=atof(field); 
     
 Serial.print("Numero de satelites: ");
 Serial.println(sat);
 
RC();
    
if(sat>=numero_sat){
  band2=1;
  band3=1;
}

if(band2==1){
  if(band==0)
    {band=1;     
       msj="Punto de origen ";
       msj+="Lat: ";
       getField(field,2);  
   a=atof(field)/100;
    grados=a;
    b=a-grados;
    minutos=(b/60);
    c=((b*60)-minutos)*100;
    segundos=(c/3600);
    total=grados+minutos+segundos;
    la=total;   
    msj+=la; 
       msj+=" ";
msj+="Long: ";
     
    getField(field,4);
    a=atof(field)/100;
    grados=a;
    b=a-grados;
    minutos=(b/60);
    c=((b*60)-minutos)*100;
    segundos=(c/3600);
    total=grados+minutos+segundos;

    lo=total; 
    msj+=lo;
     LoRa.beginPacket();
  Serial.print(msj);     
    LoRa.endPacket();
    msj="\0";
     adelante();delay(10000); 
     a1=1;    
    }
   
}}      
     sentence[i] = '\0';
     i = 0;    }  }
  
  }while(a1<1);//do
 a1=0;
  }
void gpss()
{
   static int i = 0;
  do  {
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    if (ch != '\n' && i < sentenceSize)
    {      sentence[i] = ch;
      i++;    }
    else
    {   
      displayGPS();
      
     sentence[i] = '\0';
     i = 0;    }  }

     //GY();
  }while(a1<1);//do
 a1=0;
}

void displayGPS()
{ String g;
  float lat, lon;
  int dd;
  float ss;
  char field[20];
   int grados=0,minutos=0;
  double a=0,b=0,c=0,segundos=0;
  getField(field,0);
  if (strcmp(field, "$GPGGA") == 0)
  { 
 getField(field, 7);

sat=atof(field);  

 


if(band2==1){
     
     //lat
       getField(field,2);  
   a=atof(field)/100;
    grados=a;
    b=a-grados;
    minutos=(b/60);
    c=((b*60)-minutos)*100;
    segundos=(c/3600);
    total=grados+minutos+segundos;
    la1=total;
    msj+=la1; 
     
//msj+="Long: ";
     
    getField(field,4);
    a=atof(field)/100;
    grados=a;
    b=a-grados;
    minutos=(b/60);
    c=((b*60)-minutos)*100;
    segundos=(c/3600);
    total=grados+minutos+segundos;
    lo1=total;

  
      cuadrante();
    }   
}
else{
  
  parar();
  
  }
}


void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
} 
void cuadrante()
{
 

  cad1=la1-la;
  cop1=lo1-lo;
  alp1=atan(cop1/cad1); 
  alp1=abs((alp1*180)/pi);
  if(la1>la && lo>lo1){     
   Serial.println("1er cuadrante");   
  alp0=(270-alp1);
  }
  if(la1> la && lo1>lo){     
   Serial.println("2do cuadrante"); 
  alp0=(alp1+90);
    }
  if(la>la1 && lo<lo1){     
   Serial.println("3er cuadrante"); 
  alp0=(-alp1+90);
 }
  if(la>la1 && lo>lo1){     
   Serial.println("4to cuadrante"); 
  alp0=(alp1+270);
   }   
   GY();
   RC();
   //movimiento();
 
}



  


void adelante()
{
analogWrite(PWM_A_Pin, 255);//izq
  digitalWrite(motor_izq1,0);
  digitalWrite(motor_izq2,1);
  analogWrite(PWM_B_Pin, 255);//derecho
  digitalWrite(motor_der1,1);
 digitalWrite(motor_der2,0);    
  
  
    
  }
  void atras()
  { 
       analogWrite(PWM_A_Pin, 255);//izq
  digitalWrite(motor_izq1,1);
  digitalWrite(motor_izq2,0);
  analogWrite(PWM_B_Pin, 235);//derecho
  digitalWrite(motor_der1,0);
 digitalWrite(motor_der2,1); 

    
    }
    void der()
    { analogWrite(PWM_A_Pin, 70);//izq
  digitalWrite(motor_izq1,1);
  digitalWrite(motor_izq2,0);
  analogWrite(PWM_B_Pin, 70);//derecho
  digitalWrite(motor_der1,1);
  digitalWrite(motor_der2,0); 
msj+=",izquierda<<";msj+=" ";
Serial.println("der");

      }
      void izq()
      {    analogWrite(PWM_A_Pin, 70);//izq
  digitalWrite(motor_izq1,0);
  digitalWrite(motor_izq2,1);
  analogWrite(PWM_B_Pin, 70);//derecho
  digitalWrite(motor_der1,0);
  digitalWrite(motor_der2,1);
 msj+=",derecha>>";msj+=" ";
 Serial.println("izq");
   }
    void parar()
    {     digitalWrite(motor_izq1,0);
  digitalWrite(motor_izq2,0);
  analogWrite(PWM_A_Pin, 0);//izq
  digitalWrite(motor_der1,0);
  digitalWrite(motor_der2,0);

 
    }
    
    
    void movimiento()
    {
      float ra,rb;
    cal_dis();
Serial.print(distancia0);
  ra=alp0-4;
  rb=alp0+4;
    if(la==la1 && lo!=lo1)
  {     
     Serial.println("Estas sobre el eje horizontal");   
  }
  else if(lo==lo1 && la!=la1)
  {      Serial.println("Estas sobre el eje vertical"); }
  else if(lo==lo1 && la==la1 || band4)
  {     Serial.println(" ");
  Serial.println("estas sobre el origen debes detenerte");   
  parar();delay(500);
 p3=1;
 delay(10000);
  }else
  p3=0;
      

    Serial.println("el angulo hacia el home:");
 Serial.println(alp0);

 Serial.println("el angulo mag:");
Serial.println(alp3);
  if(!p3){
if(alp3<rb && alp3 >ra){    
msj+="adelante-->>";  msj+=" ";
adelante();
delay(2000);
      } 
      
        if(alp3<ra)
      {
        
        parar();delay(400);
 der();
 delay(40);
 }
 
  if(alp3>rb)
  { parar();delay(400);
izq();
 delay(40);
    }
    msj+="Ang_home:";msj+=alp0;
    msj+="Ang_Mag:";msj+=alp3;
    if(!band)
     msj+="distancia:";msj+=distancia0;     
  LoRa.beginPacket();
  LoRa.print(msj);     
    LoRa.endPacket();
    msj="\0";
    
  //aqui termina el else
  
    }
      
      
      
      
      
      }




    void cal_dis()
      {int pr=20;
    distancia0=sqrt((pow((la1-la),2)+pow((lo1-lo),2))*11.319)*10000;
  pr=distancia0*10;
  if(pr<5)
  {band4=1;
   msj+="DIS<1m :";msj+=pr;
  }
  if(pr<=10)
  {
   msj+="DIS<1m :";msj+=pr;
  }
 
 
        }
