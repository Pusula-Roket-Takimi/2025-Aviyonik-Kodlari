

//#define TESTMODU 1
#define LoraRX D0
#define LoraTX D1
#define GpsRX D8
#define GpsTX D9
#define buzzer D4
#define PATLAMAK D12


#define PATLAMA_SURESI 1100
#define LORA_INTERVAL 1500;
#define SIT_INTERVAL = 100;


// Paket tanımı
const uint8_t HEADER = 0xAA;
const uint8_t FOOTER1 = 0x0D;
const uint8_t FOOTER2 = 0x0A;

// Command Bloğu
const uint8_t CMD_SIT_START = 0x20;  // Sit
const uint8_t CMD_SUT_START = 0x22;  // Sut
const uint8_t CMD_STOP = 0x24;       // Durdur




#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <lsm6dsm.h>
#include <deneyap.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;


// Kalman filtresi değişkenleri
float Q_Basinc = 0.001;     // Daha az sistem gürültüsü
float R_Basinc = 0.1;       // Ölçüme daha çok güven (daha düşük)
float P_Basinc = 1.0;       // Başlangıç belirsizliği
float Basinc_Kalman = 0.0;  // İlk tahmin
float K_Basinc = 0.0;       // Kalman kazancı

float Q_Z_Ivme = 0.001;     // Daha az sistem gürültüsü
float R_Z_Ivme = 0.1;       // Ölçüme daha çok güven (daha düşük)
float P_Z_Ivme = 1.0;       // Başlangıç belirsizliği
float Z_Ivme_Kalman = 0.0;  // İlk tahmin
float K_Z_Ivme = 0.0;       // Kalman kazancı

float Q_X_Ivme = 0.001;     // Daha az sistem gürültüsü
float R_X_Ivme = 0.1;       // Ölçüme daha çok güven (daha düşük)
float P_X_Ivme = 1.0;       // Başlangıç belirsizliği
float X_Ivme_Kalman = 0.0;  // İlk tahmin
float K_X_Ivme = 0.0;       // Kalman kazancı

float Q_Y_Ivme = 0.001;     // Daha az sistem gürültüsü
float R_Y_Ivme = 0.1;       // Ölçüme daha çok güven (daha düşük)
float P_Y_Ivme = 1.0;       // Başlangıç belirsizliği
float Y_Ivme_Kalman = 0.0;  // İlk tahmin
float K_Y_Ivme = 0.0;       // Kalman kazancı
// Kalman filtresi değişkenleri



// Class Tanımlaması
Adafruit_BMP280 BMP;
LSM6DSM IMU;
TinyGPSPlus GPS;

HardwareSerial GpsSerial(1);
HardwareSerial LoraSerial(2);
// Class Tanımlaması



//Değişken Tanımlamaları
bool p_durum;

// BMP
float BMP_irtifa;
double SEA_LEVEL;



// IMU
float gyroX, gyroY, gyroZ;

// GPS
String enlem, boylam;
double gps_irtifa;
//Değişken Tanımlamaları


















void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(PATLAMAK, OUTPUT);

  digitalWrite(buzzer, HIGH);


  // MAX3232
  Serial.begin(9600);



  // LoRa başlatma
  LoraSerial.begin(9600, SERIAL_8N1, LoraRX, LoraTX);
  // GPS başlatma
  GpsSerial.begin(9600, SERIAL_8N1, GpsRX, GpsTX);
  // BMP başlatma
  BMP.begin(0x76);  // veya 77 denenecek
  SEA_LEVEL = BMP.readPressure();

  // ATALETSEL OLCU BIRIMI BASLATMA
  IMU.begin();



  xTaskCreatePinnedToCore(
    Haberlesme, /* Task function. */
    "Task2",    /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &Task2,     /* Task handle to keep track of created task */
    1);         /* pin task to core 1 */


  delay(500);

  xTaskCreatePinnedToCore(
    ANA_ALGORITMA, /* Task function. */
    "Task1",       /* name of task. */
    10000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    1,             /* priority of the task */
    &Task1,        /* Task handle to keep track of created task */
    0);            /* pin task to core 0 */

  delay(500);

  xTaskCreatePinnedToCore(
    UKBTEST,   /* Task function. */
    "UKBTEST", /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task3,    /* Task handle to keep track of created task */
    0);        /* pin task to core 1 */




  delay(2000);
  digitalWrite(buzzer, LOW);
}

// ALGORITMADA
// SENSOR VERİLERİ VE KURTARMA FARKLI FONK OLACAK. BU KADAR





void sentetikUcusTesti(void* pvParameters) {
  for (;;) {
    byte packetSit[36];
    packetSit
    int index = 0;

    // Başlık
    packet[index++] = 0xAB;


    float floats[8] = { BMP_irtifa, 
    Basinc_Kalman, 
    X_Ivme_Kalman, 
    Y_Ivme_Kalman, 
    Z_Ivme_Kalman, 
    gyroX, 
    gyroY, 
    gyroZ };


    for (int i = 0; i < 8; i++) {
      byte* ptr = (byte*)(&floats[i]);
      for (int j = 0; j < 4; j++) {
        packetSit[index++] = ptr[j];
      }
    }

    packetSit[index++] = 0;  // checksum
    packetSit[index++] = 0x0D;// 35
    packetSit[index++] = 0x0A;// 36

    Serial.print(packetSit);

    vTaskDelay(SIT_INTERVAL / portTICK_PERIOD_MS); // 10 hazret

  }
}







void UKBTEST(void* pvParameters) {

  if (Serial.available()) {
    if (Serial.peek() != HEADER) {
      Serial.read();
      return;
    }

    uint8_t buf[5];                      // [HEADER, CMD, CHECKSUM, F1, F2]
    if (Serial.available() < 5) return;  // tam gelmemiş
    for (int i = 0; i < 5; i++) {
      buf[i] = Serial.read();
    }

    // 3) Basit footer kontrolü
    //  if (buf[3] != FOOTER1 || buf[4] != FOOTER2) {
    //   Serial.println(F("Footer hatalı, atlanıyor."));
    //  return;
    // }

    uint8_t cmd = buf[1];
    uint8_t receivedCk = buf[2];

    // 4) Yenilenmiş Checksum doğrulama:
    //    Önce header ve command toplanır, sonra 0xFF ile mod alınır
    uint8_t calcCk = (uint8_t)(((uint16_t)HEADER + cmd) % 0xFF);

    if (calcCk != receivedCk) {
      Serial.print(F("Checksum hatası! Hesaplanan=0x"));
      Serial.print(calcCk, HEX);
      Serial.print(F("  Gelen=0x"));
      Serial.println(receivedCk, HEX);
      return;
    }


    // 5) Komuta göre fonksiyon çağır
    switch (cmd) {
      case CMD_SIT_START:
        delay(1000);
         xTaskCreatePinnedToCore(
    ANA_ALGORITMA, /* Task function. */
    "Task1",       /* name of task. */
    10000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    1,             /* priority of the task */
    &Task1,        /* Task handle to keep track of created task */
    0);            /* pin task to core 0 */
        sensorIzlemeTesti();
        break;

      case CMD_SUT_START:
        delay(1000);
        sentetikUcusTesti();
        break;

      case CMD_STOP:
        durdur();
        break;

      default:

        break;
    }
  }
  //////////////////////////////////////////////////////////////////////// max3232 dinleme              loop son
}
















unsigned long patlama_millis;

void ANA_ALGORITMA(void* pvParameters,) {
  for (;;) {
    float basinc = BMP.readPressure();

    //Basınç Kalman
    P_Basinc = P_Basinc + Q_Basinc;
    K_Basinc = P_Basinc / (P_Basinc + R_Basinc);
    Basinc_Kalman = Basinc_Kalman + K_Basinc * (basinc - Basinc_Kalman);
    P_Basinc = (1 - K_Basinc) * P_Basinc;
    //Basınç Kalman



    // X ve Znin yerleri değiştirilmiştir///////////////////////
    float roketivme_X, roketivme_Y, roketivme_Z;
    roketivme_Z = abs(IMU.readFloatAccelX()) * 100;
    roketivme_Y = abs(IMU.readFloatAccelY()) * 100;
    roketivme_X = abs(IMU.readFloatAccelZ()) * 100;

    gyroZ = abs(IMU.readFloatGyroX()) * 100;
    gyroY = abs(IMU.readFloatGyroY()) * 100;
    gyroX = abs(IMU.readFloatGyroZ()) * 100;

    /////////////////////////////////////////////////////////////


    //İvme_X Kalman
    P_X_Ivme = P_X_Ivme + Q_X_Ivme;
    K_X_Ivme = P_X_Ivme / (P_X_Ivme + R_X_Ivme);
    X_Ivme_Kalman = X_Ivme_Kalman + K_X_Ivme * (roketivme_X - X_Ivme_Kalman);
    P_X_Ivme = (1 - K_X_Ivme) * P_X_Ivme;
    //İvme_X Kalman

    //İvme_Y Kalman
    P_Y_Ivme = P_Y_Ivme + Q_Y_Ivme;
    K_Y_Ivme = P_Y_Ivme / (P_Y_Ivme + R_Y_Ivme);
    Y_Ivme_Kalman = Y_Ivme_Kalman + K_Y_Ivme * (roketivme_Y - Y_Ivme_Kalman);
    P_Y_Ivme = (1 - K_Y_Ivme) * P_Y_Ivme;
    //İvme_Y Kalman

    //İvme Z Kalman
    P_Z_Ivme = P_Z_Ivme + Q_Z_Ivme;
    K_Z_Ivme = P_Z_Ivme / (P_Z_Ivme + R_Z_Ivme);
    Z_Ivme_Kalman = Z_Ivme_Kalman + K_Z_Ivme * (roketivme_Z - Z_Ivme_Kalman);
    P_Z_Ivme = (1 - K_Z_Ivme) * P_Z_Ivme;
    //İvme Z Kalman





    float new_irtifa = BMP.readAltitude(SEA_LEVEL);
    bool irtifaKaybi, roketYatma;

    if (new_irtifa < BMP_irtifa)
      irtifaKaybi = true;

    BMP_irtifa = new_irtifa;

    if (X_Ivme_Kalman > 75 || Y_Ivme_Kalman > 75)
      roketYatma = true;

    bool kosul = (irtifaKaybi && roketYatma);

    if (kosul && !p_durum) {
      p_durum = true;
      digitalWrite(PATLAMAK, HIGH);
    }

    unsigned long new_millis = millis();

    if (new_millis - patlama_millis >= PATLAMA_SURESI) {
      patlama_millis = new_millis;
      digitalWrite(PATLAMAK, LOW);
    }
  }
}










void Haberlesme(void* pvParameters) {
  for (;;) {

    if (GpsSerial.available()) {
      if (GPS.encode(GpsSerial.read())) {
        if (GPS.location.isValid() && GPS.altitude.isValid()) {
          enlem = String(GPS.location.lat(), 6);
          boylam = String(GPS.location.lng(), 6);


          gps_irtifa = GPS.altitude.meters();
        }
      }
    } else {
      enlem = "0.000000";
      boylam = "0.000000";
      gps_irtifa = 00.0;
    }

    // HEADERLER
    LoraSerial.write((byte)0x00);
    LoraSerial.write(0x16);  // adres 22=16 - 21=15 - 23=17
    LoraSerial.write(0x12);  // kanal 18 için
    LoraSerial.print("#BOD,");


    // GPS
    LoraSerial.print("B=");
    LoraSerial.print(boylam);
    LoraSerial.print(",");

    LoraSerial.print("E=");
    LoraSerial.print(enlem);
    LoraSerial.print(",");

    LoraSerial.print("GI=");
    LoraSerial.print(gps_irtifa);
    LoraSerial.print(",");

    // BMP
    LoraSerial.print("P=");
    LoraSerial.print(Basinc_Kalman);
    LoraSerial.print(",");

    LoraSerial.print("PI=");
    LoraSerial.print(BMP_irtifa);
    LoraSerial.print(",");

    // IMU
    LoraSerial.print("GX=");
    LoraSerial.print(gyroX);
    LoraSerial.print(",");

    LoraSerial.print("GY=");
    LoraSerial.print(gyroY);
    LoraSerial.print(",");

    LoraSerial.print("GZ=");
    LoraSerial.print(gyroZ);
    LoraSerial.print(",");

    LoraSerial.print("AX=");
    LoraSerial.print(X_Ivme_Kalman);
    LoraSerial.print(",");

    LoraSerial.print("AY=");
    LoraSerial.print(Y_Ivme_Kalman);
    LoraSerial.print(",");

    LoraSerial.print("AZ=");
    LoraSerial.print(Z_Ivme_Kalman);
    LoraSerial.print(",");

    LoraSerial.print("PD=");
    LoraSerial.print(p_durum);
    LoraSerial.print(",");

    // FOOTER
    LoraSerial.println("#EOD");

    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }
}





void loop(){};

// float f1 = 1.23;
// float f2 = 4.56;
// float f3 = 7.89;
// float f4 = 10.11;
// float f5 = 12.13;
// float f6 = 14.15;
// float f7 = 16.17;
// float f8 = 18.19;

// void setup() {
//   Serial.begin(9600);
// }

// void loop() {
//   byte packet[36];  // toplam paket uzunluğu

//   int index = 0;

//   // Başlık
//   packet[index++] = 0xAB;

//   // 8 float'ı sırayla ekle
//   float floats[8] = {f1, f2, f3, f4, f5, f6, f7, f8};
//   for (int i = 0; i < 8; i++) {
//     byte *ptr = (byte*)(&floats[i]);
//     for (int j = 0; j < 4; j++) {
//       packet[index++] = ptr[j];
//     }
//   }

//   // Paket sonu
//   packet[index++] = 0;
//   packet[index++] = 0x0D;
//   packet[index++] = 0x0A;

//   // Seri ekrana yaz
//   Serial.print("Packet (HEX): ");
//   for (int i = 0; i < 36; i++) {
//     if (packet[i] < 0x10) Serial.print("0");  // 0x0F gibi başa 0 koy
//     Serial.print(packet[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();

//   delay(1000);
// }
