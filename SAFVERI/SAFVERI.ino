
//  Kütüphane Tanımlamaları
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <lsm6dsm.h>
#include <deneyap.h>

//  Kütüphane Tanımlamaları

// Pin Tanımlamaları
#define LoraTX D9
#define GpsRX D13
#define MAXTX D0
#define MAXRX D1
#define BUZZER D4
#define PATLAMAK D12
#define SEALEVELPRESSURE_HPA 994  // hava durumuna bak // aksaray 1003
#define GRAVITY 9.80665
// Pin Tanımlamaları

// Class Tanımlamaları

Adafruit_BMP280 BMP;
LSM6DSM IMU;
TinyGPSPlus GPS;
HardwareSerial MAX3232Serial(1);
HardwareSerial ComboSerial(2);

// Class Tanımlamaları


// Değişkenler
double basinc;
float ivme_X;
float ivme_Y;
float ivme_Z;
float basinc_Irtifa;
// Değişkenler



// Setup Fonksiyonu
void setup(){
    Serial.begin(115200);
    pinMode(BUZZER,OUTPUT);
    pinMode(PATLAMAK,OUTPUT);
    digitalWrite(BUZZER,HIGH);
    BMP.begin(0x76);
    IMU.begin();
    MAX3232Serial.begin(115200,SERIAL_8N1,MAXRX,MAXTX);
    ComboSerial.begin(115200,SERIAL_8N1,GpsRX,LoraTX);
    delay(200);
}
void loop(){
basinc=BMP.readPressure() / 100;
basinc_Irtifa=BMP.readAltitude(SEALEVELPRESSURE_HPA);
ivme_X=IMU.readFloatAccelZ(); //Değiştirildi
ivme_Y=IMU.readFloatAccelY();
ivme_Z=IMU.readFloatAccelZ(); //Değiştirildi

Serial.print("Basinc: ");
Serial.print(basinc);
Serial.print(" hPa | Irtifa: ");
Serial.print(basinc_Irtifa);
Serial.print(" m | IvmeX: ");
Serial.print(ivme_X, 2);
Serial.print(" | IvmeY: ");
Serial.print(ivme_Y, 2);
Serial.print(" | IvmeZ: ");
Serial.println(ivme_Z, 2);
delay(1000);

}
