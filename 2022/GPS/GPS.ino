const int tamano_sentencia = 80;
char sentencia[tamano_sentencia];
float latitud, longitud, satelites;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  //GPS.begin(115200);
}

void loop()
{
  GPS();
}

void GPS(){
    static int i = 0;
  if (Serial1.available())
  {
    char dato = Serial1.read();
    if (dato != '\n' && i < tamano_sentencia)
    {
      sentencia[i] = dato;
      i++;
    }
    else
    {
     sentencia[i] = '\0';
     i = 0;
     displayGPS();
    }
  }
}

void displayGPS()
{
  char campo[20];
  obtener_campo(campo, 0);
  if (strcmp(campo, "$GPGGA") == 0)//AQUÍ PUEDES SELECCIONAR LA SENTENCIA
  {

    obtener_campo(campo, 2);  //AQUÍ SELECCIONAS EL CAMPO QUE QUIERES EXTRAER
    latitud=atof(campo);
    Serial.print(campo);
    Serial.print(", ");
    obtener_campo(campo, 4);  
    longitud=atof(campo);
    Serial.print(campo);
    Serial.print(", ");
    obtener_campo(campo, 7);  
    satelites=atof(campo);
    Serial.println(campo);
  }
}


void obtener_campo(char* buffer, int indice)
{
  int sentencia_pos = 0;
  int posicion_campo = 0;
  int contador_comas = 0;
  while (sentencia_pos < tamano_sentencia)
  {
    if (sentencia[sentencia_pos] == ',')
    {
      contador_comas ++;
      sentencia_pos ++;
    }
    if (contador_comas == indice)
    {
      buffer[posicion_campo] = sentencia[sentencia_pos];
      posicion_campo ++;
    }
    sentencia_pos ++;
  }
  buffer[posicion_campo] = '\0';
} 
