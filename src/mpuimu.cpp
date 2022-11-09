
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

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

  
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    
    accelgyro.getMotion6(&imuData.ax, &imuData.ay, &imuData.az, &imuData.gx, &imuData.gy, &imuData.gz);


    #ifdef OUTPUT_READABLE_ACCELGYRO
        
        Serial.print("a/g:\t");
        Serial.print(imuData.ax); Serial.print("\t");
        Serial.print(imuData.ay); Serial.print("\t");
        Serial.print(imuData.az); Serial.print("\t");
        Serial.print(imuData.gx); Serial.print("\t");
        Serial.print(imuData.gy); Serial.print("\t");
        Serial.println(imuData.gz);
    #endif

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