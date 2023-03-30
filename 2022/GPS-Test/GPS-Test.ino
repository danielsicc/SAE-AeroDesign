#include "SoftwareSerial.h"

SoftwareSerial GPS(7,-1); //Rx, Tx
const int tamano=80, anguloMag=180;
float lat, lon, latMeta=32.379589, lonMeta=115.266624;
int sat, cuadrantes;
char sentencia[tamano];

void setup()
{
  Serial.begin(9600);
  GPS.begin(9600);
}
/*
$GPGLL,,,,,002026.00,V,N*4C
$GPRMC,002027.00,V,,,,,,,220421,,,N*7D
$GPVTG,,,,,,,,,N*30
$GPGGA,002027.00,,,,,0,00,99.99,,,,,,*61
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,1,1,03,02,,,22,25,,,29,29,,,30*7C

 */

void loop()
{
  static int i=0;
  if(GPS.available())
  {
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
    Serial.print(sat);
    
    cuadrante();
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
   Serial.println("  1ER CUADRANTE"); 
   cuadrantes=1;  
   }
  if(latMeta<lat && lonMeta<lon){     
   Serial.println("  2DO CUADRANTE");
   cuadrantes=2;  
    }
  if(latMeta>lat && lon>lonMeta){     
   Serial.println("  3ER CUADRANTE"); 
   cuadrantes=3; 
 }
  if(lat<latMeta && lon<lonMeta){     
   Serial.println("  4TO CUADRANTE"); 
   cuadrantes=4; 
   }   
}

void angulo()
{
  //calculateHeading();
  //switch(cuadrantes){
    //case 1:
      
    //case 2:
      
    //case 3:
      
    //case 4:
      
  //}
}
