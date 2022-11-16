
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#endif

File myFile;
SoftwareSerial ss(0, 1);

MPU6050 accelgyro;

struct IMUData{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

IMUData imuData;

 int cont;
  


#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
bool blinkState = false;

void setup() {
   
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
    Serial.begin(38400);
{
  Serial.begin(115200);
  ss.begin(9600);
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  } 
  Serial.println("initialization done.");

  myFile = SD.open("imudata.txt", FILE_WRITE);
  
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    
    pinMode(LED_PIN, OUTPUT);
}
}

void loop() {

    
    accelgyro.getMotion6(&imuData.ax, &imuData.ay, &imuData.az, &imuData.gx, &imuData.gy, &imuData.gz);
    
  
     cont = cont + 1;
    Serial.println(cont);
    


    #ifdef OUTPUT_READABLE_ACCELGYRO
        
        Serial.print("a/g:\t ");
        Serial.print(imuData.ax); Serial.print(","); 
        Serial.print(imuData.ay); Serial.print(",");
        Serial.print(imuData.az); Serial.print(",");
        Serial.print(imuData.gx); Serial.print(",");
        Serial.print(imuData.gy); Serial.print(",");
        Serial.println( imuData.gz);
    #endif

 myFile = SD.open("imudata.txt", FILE_WRITE); 

 if (myFile){

 Serial.print("Writing to imudata.txt...");

 myFile.print("1,");
  myFile.print(imuData.ax);myFile.print(",");
  myFile.print(imuData.ay);myFile.print(",");
  myFile.print(imuData.az);myFile.print(",");
  myFile.print(imuData.gx);myFile.print(",");
  myFile.print(imuData.gy);myFile.print(",");
  myFile.println(imuData.gz);

  Serial.println("done.");
   } else {
    
    Serial.println("No hay SD...");
    
  }
   
   myFile.close();


    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(imuData.ax >> 8)); Serial.write((uint8_t)(imuData.ax & 0xFF));
        Serial.write((uint8_t)(imuData.ay >> 8)); Serial.write((uint8_t)(imuData.ay & 0xFF));
        Serial.write((uint8_t)(imuData.az >> 8)); Serial.write((uint8_t)(imuData.az & 0xFF));
        Serial.write((uint8_t)(imuData.gx >> 8)); Serial.write((uint8_t)(imuData.gx & 0xFF));
        Serial.write((uint8_t)(imuData.gy >> 8)); Serial.write((uint8_t)(imuData.gy & 0xFF));
        Serial.write((uint8_t)(imuData.gz >> 8)); Serial.write((uint8_t)(imuData.gz & 0xFF));
    #endif


    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}