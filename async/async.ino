//#define TESTMODU 1




#include <HardwareSerial.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "lsm6dsm.h"
TaskHandle_t Task1;
TaskHandle_t Task2;
// LED pins









// Kalman filtresi değişkenleri
float Q = 0.001;  // Daha az sistem gürültüsü
float R = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P = 1.0;    // Başlangıç belirsizliği
float X = 0.0;    // İlk tahmin
float K = 0.0;    // Kalman kazancı

float Q1 = 0.001;  // Daha az sistem gürültüsü
float R1 = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P1 = 1.0;    // Başlangıç belirsizliği
float X1 = 0.0;    // İlk tahmin
float K1 = 0.0;    // Kalman kazancı


float Q2 = 0.001;  // Daha az sistem gürültüsü
float R2 = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P2 = 1.0;    // Başlangıç belirsizliği
float X2 = 0.0;    // İlk tahmin
float K2 = 0.0;    // Kalman kazancı





#define buzzer D4
String p_durum = "-";
Adafruit_BMP280 bmp;
LSM6DSM IMU;


// Lora tanımlamaları
#define LoraRX D0
#define LoraTX D1

HardwareSerial LoraSerial(2);

float irtifaBasinc;
int irtifaKaybi = 0;
int roketYatma = 0;
int tetiklenmeSayisi=0;



// GPS tanımlamaları
#define GpsRX D8
#define GpsTX D9
TinyGPSPlus gps;
HardwareSerial GpsSerial(1);

String enlem, boylam, irtifa;

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
    float basinc = bmp.readPressure();
    float roketAci_X = abs(IMU.readFloatAccelX()) * 100;
    float roketAci_Y = abs(IMU.readFloatAccelY()) * 100;
    irtifaBasinc = bmp.readAltitude(X);

    P = P + Q;
    K = P / (P + R);
    X = X + K * (basinc - X);
    P = (1 - K) * P;

    P1 = P1 + Q1;
    K1 = P1 / (P1 + R1);
    X1 = X1 + K1 * (roketAci_X - X1);
    P1 = (1 - K1) * P1;

    P2 = P2 + Q2;
    K2 = P2 / (P2 + R2);
    X2 = X2 + K1 * (roketAci_Y - X2);
    P2 = (1 - K2) * P2;

    float new_irtifa = bmp.readAltitude(X);
    if (new_irtifa < irtifaBasinc) {
      irtifaKaybi=1;
    }
    if(X1>75 || X2>75 )
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
          enlem = String(gps.location.lat(), 6);
          boylam = String(gps.location.lng(), 6);


          double altitude = gps.altitude.meters();

          irtifa = String(altitude);
        }
      }
    } else {
      enlem = "0.000000";
      boylam = "0.000000";
      irtifa = "00.00";
    }

    //Açı verileri alınıyor
    String x = String((IMU.readFloatAccelX()) * 100);
    String y = String((IMU.readFloatAccelY()) * 100);
    String z = String((IMU.readFloatAccelZ()) * 100);

    // Veri paketi:
    String VeriPaketi = "Aviyonik," + enlem + "," + boylam + "," + X + "," + x + "," + y + "," + X1 + "," + irtifa + "," + p_durum;
      #ifdef TESTMODU
  Serial.print("Veri paket aktif: ");
  Serial.println(VeriPaketi);
    #endif

    // LoRa'ya paket gönderimi
    LoraSerial.write((byte)0x00);
    LoraSerial.write(0x17);
    LoraSerial.write(0x12);
    LoraSerial.println(VeriPaketi);
    delay(700);
  }
}

void loop() {}
