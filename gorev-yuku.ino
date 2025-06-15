
#include <DHT.h>
#include <DHT_U.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <lsm6dsm.h>
//Önİşlemci Tanımlamaları
TaskHandle_t Task1;
TaskHandle_t Task2;
#define GpsRX D8
#define GpsTX D9
#define buzzer D4
#define LoraRX D0
#define LoraTX D1
//Önİşlemci Tanımlamaları
// Class Tanımlaması
DHT dht(D12,DHT11);
Adafruit_BMP280 bmp;
LSM6DSM IMU;
HardwareSerial LoraSerial(2);
TinyGPSPlus gps;
HardwareSerial GpsSerial(1);
// Class Tanımlaması

//Değişken Tanımlamaları
struct variables{
float xAngle,yAngle,zAngle;
float sicaklik,nem;
float irtifaBasinc;
float basinc, roketivme_X, roketivme_Y, roketivme_Z ,gyroX,gyroY,gyroZ,new_irtifa;
double altitude;
String enlem, boylam;
}degiskenler;

//Değişken Tanımlamaları
void setup() {
  
  // LoRa başlatma
  LoraSerial.begin(9600, SERIAL_8N1, LoraRX, LoraTX);
  // GPS başlatma
  GpsSerial.begin(9600, SERIAL_8N1, GpsRX, GpsTX);
  bmp.begin(0x76);  //veya 77 denenecek
  IMU.begin();
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  delay(2000);
  digitalWrite(buzzer, LOW);
   xTaskCreatePinnedToCore(
    Degiskenler, /* Task function. */
    "Task1",  /* name of task. */
    10000,    /* Stack size of task */
    NULL,     /* parameter of the task */
    1,        /* priority of the task */
    &Task1,   /* Task handle to keep track of created task */
    0);       /* pin task to core 0 */
  delay(500);
   xTaskCreatePinnedToCore(
    Haberlesme, /* Task function. */
    "Task2",    /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &Task2,  /* Task handle to keep track of created task */
    1);         /* pin task to core 1 */
  delay(500);
}
void Degiskenler(void* pvParameters) {
  
  for (;;) {
     if (GpsSerial.available()) {
      if (gps.encode(GpsSerial.read())) {
        if (gps.location.isValid() && gps.altitude.isValid()) {
          degiskenler.enlem = String(gps.location.lat(),6);
          degiskenler.boylam =String(gps.location.lng(),6);


           degiskenler.altitude = gps.altitude.meters();

         
        }
      }
    } else {
      degiskenler.enlem = "0.000000";
      degiskenler.boylam = "0.000000";
      degiskenler.altitude = 00.0;
    }

     degiskenler.sicaklik = dht.readTemperature();
     degiskenler.nem = dht.readHumidity();
     degiskenler.basinc = bmp.readPressure();
     degiskenler.roketivme_Z = abs(IMU.readFloatAccelZ()) * 100;
     degiskenler.roketivme_Y = abs(IMU.readFloatAccelY()) * 100;
     degiskenler.roketivme_X = abs(IMU.readFloatAccelX()) * 100;
     degiskenler.gyroZ= abs(IMU.readFloatGyroX()) * 100;
     degiskenler.gyroY= abs(IMU.readFloatGyroY()) * 100;
     degiskenler.gyroX= abs(IMU.readFloatGyroZ()) * 100;
     degiskenler.irtifaBasinc = bmp.readAltitude(degiskenler.basinc);
     

  }
}

void Haberlesme(void* pvParameters) {
  


  for (;;) {
   LoraSerial.write((byte)0x00);//adresler değiştirilecek
   LoraSerial.write(0x15);//adresler değiştirilecek
   LoraSerial.write(0x12);//adresler değiştirilecek
  
   LoraSerial.print("#BOD_Gorev_Yuku,");
   LoraSerial.print("BOYLAM=");
   LoraSerial.print(degiskenler.boylam);
   LoraSerial.print(",");

   LoraSerial.print("ENLEM=");
   LoraSerial.print(degiskenler.enlem);
   LoraSerial.print(",");

  LoraSerial.print("GPS_IRTIFA=");
   LoraSerial.print(degiskenler.altitude);
   LoraSerial.print(",");


   LoraSerial.print("BASINC=");
   LoraSerial.print(degiskenler.basinc);
   LoraSerial.print(",");

  LoraSerial.print("BASINC_IRTIFA=");
   LoraSerial.print(degiskenler.irtifaBasinc);
   LoraSerial.print(",");

  LoraSerial.print("GYRO_X=");
   LoraSerial.print(degiskenler.gyroX);
   LoraSerial.print(",");

  LoraSerial.print("GYRO_Y=");
   LoraSerial.print(degiskenler.gyroY);
   LoraSerial.print(",");

   LoraSerial.print("GYRO_Z=");
   LoraSerial.print(degiskenler.gyroZ);
   LoraSerial.print(",");

  LoraSerial.print("ACCEL_X=");
   LoraSerial.print(degiskenler.roketivme_X);
   LoraSerial.print(",");

   LoraSerial.print("ACCEL_Y=");
   LoraSerial.print(degiskenler.roketivme_Y);
   LoraSerial.print(",");

   LoraSerial.print("ACCEL_Z=");
   LoraSerial.print(degiskenler.roketivme_Z);
   LoraSerial.print(",");

   LoraSerial.print("SICAKLIK=");
   LoraSerial.print(degiskenler.sicaklik);
   LoraSerial.print(",");

   LoraSerial.print("NEM=");
   LoraSerial.print(degiskenler.nem);
   LoraSerial.print(",");

   degiskenler.xAngle = atan2(degiskenler.roketivme_X, sqrt(degiskenler.roketivme_Y * degiskenler.roketivme_Y + degiskenler.roketivme_Z * degiskenler.roketivme_Z));
   degiskenler.yAngle = atan2(degiskenler.roketivme_Y, sqrt(degiskenler.roketivme_X * degiskenler.roketivme_X + degiskenler.roketivme_Z * degiskenler.roketivme_Z));
   degiskenler.zAngle = atan2(sqrt(degiskenler.roketivme_X * degiskenler.roketivme_X + degiskenler.roketivme_Y * degiskenler.roketivme_Y), degiskenler.roketivme_Z);

   LoraSerial.print("REALGYRO_X=");
   LoraSerial.print(degiskenler.xAngle);
   LoraSerial.print(",");

   LoraSerial.print("REALGYRO_Y=");
   LoraSerial.print(degiskenler.yAngle);
   LoraSerial.print(",");

   LoraSerial.print("REALGYRO_Z=");
   LoraSerial.print(degiskenler.zAngle);
   LoraSerial.print(",");

   LoraSerial.println("#EOD");
    
  delay(1800);

  }
}


void loop() { 
}
