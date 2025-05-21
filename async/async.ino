
#include <HardwareSerial.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "lsm6dsm.h"
TaskHandle_t Task1;
TaskHandle_t Task2;
// Kalman filtresi değişkenleri
float Q_Basinc= 0.001;  // Daha az sistem gürültüsü
float R_Basinc = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P_Basinc = 1.0;    // Başlangıç belirsizliği
float Basinc_Kalman = 0.0;    // İlk tahmin
float K_Basinc = 0.0;    // Kalman kazancı
float Q_XEksen = 0.001;  // Daha az sistem gürültüsü
float R_XEksen = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P_XEksen = 1.0;    // Başlangıç belirsizliği
float XEksen_Kalman = 0.0;    // İlk tahmin
float K_XEksen = 0.0;    // Kalman kazancı
float Q_YEksen = 0.001;  // Daha az sistem gürültüsü
float R_YEksen = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P_YEksen = 1.0;    // Başlangıç belirsizliği
float YEksen_Kalman = 0.0;    // İlk tahmin
float K_YEksen = 0.0;    // Kalman kazancı
// Kalman filtresi değişkenleri
//Önİşlemci Tanımlamaları
#define GpsRX D8
#define GpsTX D9
#define buzzer D4
#define LoraRX D0
#define LoraTX D1
//Önİşlemci Tanımlamaları

// Class Tanımlaması
Adafruit_BMP280 bmp;
LSM6DSM IMU;
HardwareSerial LoraSerial(2);
TinyGPSPlus gps;
HardwareSerial GpsSerial(1);
// Class Tanımlaması

//Değişken Tanımlamaları
String p_durum = "-";
float irtifaBasinc;
int irtifaKaybi = 0;
int roketYatma = 0;
int tetiklenmeSayisi=0;
float basinc, roketAci_X, roketAci_Y, roketAci_Z ,gyroX,gyroY,gyroZ,new_irtifa;
double enlem, boylam, altitude;
//Değişken Tanımlamaları
void setup() {
  #ifdef TESTMODU
  Serial.begin(115200);
  #endif
  
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
  //create a task that will be executed in the Kurtarma() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Kurtarma, /* Task function. */
    "Task1",  /* name of task. */
    10000,    /* Stack size of task */
    NULL,     /* parameter of the task */
    1,        /* priority of the task */
    &Task1,   /* Task handle to keep track of created task */
    0);       /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Haberlesme() function, with priority 1 and executed on core 1
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

void Kurtarma(void* pvParameters) {
  #ifdef TESTMODU
  Serial.print("Kurtarma running on core ");
  Serial.println(xPortGetCoreID());
  #endif
  for (;;) {
     basinc = bmp.readPressure();
     roketAci_X = abs(IMU.readFloatAccelX()) * 100;
     roketAci_Y = abs(IMU.readFloatAccelY()) * 100;
     roketAci_Z = abs(IMU.readFloatAccelZ()) * 100;
     gyroX= abs(IMU.readFloatGyroX()) * 100;
     gyroY= abs(IMU.readFloatGyroY()) * 100;
     gyroZ= abs(IMU.readFloatGyroZ()) * 100;
    //Basınç Kalman
    P_Basinc = P_Basinc + Q_Basinc;
    K_Basinc= P_Basinc / (P_Basinc + R_Basinc);
    Basinc_Kalman = Basinc_Kalman + K_Basinc * (basinc - Basinc_Kalman);
    P_Basinc= (1 - K_Basinc) * P_Basinc;
    //Basınç Kalman
    irtifaBasinc = bmp.readAltitude(Basinc_Kalman);
    //İvme_X Kalman
    P_XEksen = P_XEksen + Q_XEksen;
    K_XEksen = P_XEksen / (P_XEksen + R_XEksen);
    XEksen_Kalman = XEksen_Kalman + K_XEksen * (roketAci_X - XEksen_Kalman);
    P_XEksen = (1 - K_XEksen) * P_XEksen;
    //İvme_X Kalman

    //İvme_Y Kalman
    P_YEksen= P_YEksen + Q_YEksen;
    K_YEksen = P_YEksen / (P_YEksen + R_YEksen);
    YEksen_Kalman = YEksen_Kalman + K_YEksen * (roketAci_Y - YEksen_Kalman);
    P_YEksen = (1 - K_YEksen) * P_YEksen;
    //İvme_Y Kalman

    new_irtifa = bmp.readAltitude(Basinc_Kalman);
    if (new_irtifa < irtifaBasinc) {
      irtifaKaybi=1;
    }
    if(XEksen_Kalman <55 || YEksen_Kalman >75  )
    {
      roketYatma=1;
    }
    if (irtifaKaybi && roketYatma && tetiklenmeSayisi==0) {
      digitalWrite(buzzer, HIGH);
      tetiklenmeSayisi = 1;
      p_durum = "+";
      delay(600);

    } else {
      digitalWrite(buzzer, LOW);
    }
    #ifdef TESTMODU
    Serial.print("Patladı mı?");    Serial.println(c);

    #endif
    irtifaBasinc = new_irtifa;
  }
}



void Haberlesme(void* pvParameters) {
    #ifdef TESTMODU
  Serial.print("Haberlesme running on core ");
  Serial.println(xPortGetCoreID());
    #endif


  for (;;) {
    if (GpsSerial.available()) {
      if (gps.encode(GpsSerial.read())) {
        if (gps.location.isValid() && gps.altitude.isValid()) {
          enlem = gps.location.lat(),6;
          boylam =gps.location.lng(),6;


           altitude = gps.altitude.meters();

         
        }
      }
    } else {
      enlem = 0.000000;
      boylam = 0.000000;
      altitude = 00.0;
    }
    // LoRa'ya paket gönderimi
    LoraSerial.write((byte)0x00);
    LoraSerial.write(0x15);
    LoraSerial.write(0x12);
  
   LoraSerial.print("#BOD");
   LoraSerial.print("Boylam=");
   LoraSerial.print(boylam);
   LoraSerial.print(",");

   LoraSerial.print("Enlem=");
   LoraSerial.print(enlem);
   LoraSerial.print(",");

   LoraSerial.print("GPS_İRTİFA=");
   LoraSerial.print(altitude);
   LoraSerial.print(",");

  LoraSerial.print("Basinc_İRTİFA=");
   LoraSerial.print(irtifaBasinc);
   LoraSerial.print(",");

  LoraSerial.print("Gyro_X=");
   LoraSerial.print(gyroX);
   LoraSerial.print(",");

  LoraSerial.print("Gyro_Y=");
   LoraSerial.print(gyroY);
   LoraSerial.print(",");

   LoraSerial.print("Gyro_Z=");
   LoraSerial.print(gyroZ);
   LoraSerial.print(",");

  LoraSerial.print("Accel_X=");
   LoraSerial.print(roketAci_X);
   LoraSerial.print(",");

   LoraSerial.print("Accel_Y=");
   LoraSerial.print(roketAci_Y);
   LoraSerial.print(",");

   LoraSerial.print("Accel_Z=");
   LoraSerial.print(roketAci_Z);
   LoraSerial.print(",");

   LoraSerial.print("Parasut Durum=");
   LoraSerial.print(p_durum);
   LoraSerial.print(",");


float xAccel =roketAci_X; // X eksenindeki ivme değeri
float yAccel = roketAci_Y; // Y eksenindeki ivme değeri
float zAccel = roketAci_Z; // Z eksenindeki ivme değeri


float xAngle = atan2(xAccel, sqrt(yAccel * yAccel + zAccel * zAccel));
float yAngle = atan2(yAccel, sqrt(xAccel * xAccel + zAccel * zAccel));
float zAngle = atan2(sqrt(xAccel * xAccel + yAccel * yAccel), zAccel);

   LoraSerial.print("RealGyro_X=");
   LoraSerial.print(xAngle);
   LoraSerial.print(",");

   LoraSerial.print("RealGyro_Y=");
   LoraSerial.print(yAngle);
   LoraSerial.print(",");

   LoraSerial.print("RealGyro_Z=");
   LoraSerial.print(zAngle);
   LoraSerial.print(",");

   LoraSerial.println("#EOD");

  delay(700);
  }
}

void loop() {}

