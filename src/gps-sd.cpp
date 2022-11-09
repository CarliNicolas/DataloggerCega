
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>

File myFile;
TinyGPS gps;
SoftwareSerial ss(0, 1);

struct GPSData{
    char id='g';
    int verificacion;
    float altitud;
    float latitud;
    float longitud;
    float velocidadennudos;
    char orientacion[5];
    char fechayhora[50];
};

GPSData gpsData;

void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  
  myFile = SD.open("gpsdata.txt", FILE_WRITE);  
}

void loop()
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, &gpsData.verificacion);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, &gpsData.latitud);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, &gpsData.longitud);
  print_date(gps, gpsData.fechayhora);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, &gpsData.altitud);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, &gpsData.velocidadennudos);
       
if(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE){sprintf(gpsData.orientacion,"0");}else{sprintf(gpsData.orientacion,"%s", TinyGPS::cardinal(gps.f_course()));}
 


  Serial.print("ID= ");Serial.println(gpsData.id);
  Serial.print("Verificacion (Cantidad de satelites)= ");Serial.println(gpsData.verificacion);
  Serial.print("Altitud= ");Serial.println(gpsData.altitud);
  Serial.print("Latitud= ");Serial.println(gpsData.latitud);
  Serial.print("Longitud= ");Serial.println(gpsData.longitud);
  Serial.print("Velocidad= ");Serial.println(gpsData.velocidadennudos);
  Serial.print("Orientacion= ");Serial.println(gpsData.orientacion);
  Serial.print("Fecha y hora= ");Serial.println(gpsData.fechayhora);
  

 myFile = SD.open("gpsdata.txt", FILE_WRITE);  

  if (myFile && gpsData.verificacion >= 1 ) {
    
  Serial.print("Writing to gpsdata.txt...");
  myFile.print("ID= ");myFile.println(gpsData.id);
  myFile.print("Esperando satelites...");myFile.println(gpsData.verificacion);
  
  myFile.print("Altitud= ");myFile.println(gpsData.altitud);
  myFile.print("Latitud= ");myFile.println(gpsData.latitud);
  myFile.print("Longitud= ");myFile.println(gpsData.longitud);
  myFile.print("Velocidad= ");myFile.println(gpsData.velocidadennudos);
  myFile.print("Orientacion= ");myFile.println(gpsData.orientacion);
  myFile.print("Fecha y hora= ");myFile.println(gpsData.fechayhora);

 
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("No hay satelites...");
    
  }
   // close the file:
   myFile.close();
     


  
  smartdelay(1000);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, float* gpsDataLocal)
{
  if (val == invalid)
  {
      gpsDataLocal=0;
  
  }
  else
  {
    gpsDataLocal=&val;
 
  }
  smartdelay(0);
}

static void print_int(int val, float invalid, int* gpsDataLocal)
{
  if(val == invalid)
    gpsDataLocal=0;
  else
  gpsDataLocal=&val;
  smartdelay(0);
}

static void print_date(TinyGPS &gps, char* gpsDataLocal)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    gpsDataLocal=0;
  else
  {
    char sz[32];
    gpsDataLocal=(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        year, month, day, hour, minute, second);
  }
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  smartdelay(0);
}