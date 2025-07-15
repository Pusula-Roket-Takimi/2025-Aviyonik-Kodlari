#define TESTMODU 1
static bool aktif = true;   
static bool pasif = false;  

#define LoraTX D9
#define GpsRX D13
#define BUZZER D4
#define PATLAMAK D12

#define ALICI_ADRES 31
#define ALICI_KANAL 50

#define PATLAMA_SURESI 1100
#define LORA_INTERVAL 900
#define SIT_INTERVAL 100

#define SEALEVELPRESSURE_HPA (991)


#include <HardwareSerial.h>
#include <TinyGPS++.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <lsm6dsm.h>
#include <deneyap.h>



//////////////////////////////////////////////////////
///////////////////////KALMAN/////////////////////////
//////////////////////////////////////////////////////

// Basınç
float Q_Basinc = 0.001;
float R_Basinc = 0.1;
float P_Basinc = 1.0;
float Basinc_Kalman = 0.0;
float K_Basinc = 0.0;

// İvme X
float Q_X_Ivme = 0.001;
float R_X_Ivme = 0.1;
float P_X_Ivme = 1.0;
float X_Ivme_Kalman = 0.0;
float K_X_Ivme = 0.0;

// İvme Y
float Q_Y_Ivme = 0.001;
float R_Y_Ivme = 0.1;
float P_Y_Ivme = 1.0;
float Y_Ivme_Kalman = 0.0;
float K_Y_Ivme = 0.0;

// İvme Z
float Q_Z_Ivme = 0.001;
float R_Z_Ivme = 0.1;
float P_Z_Ivme = 1.0;
float Z_Ivme_Kalman = 0.0;
float K_Z_Ivme = 0.0;


enum {
  SENSOR_BASINC,
  SENSOR_IVME_Z,
  SENSOR_IVME_X,
  SENSOR_IVME_Y
};

float kalmanla(float olcum, int sensorID) {
  switch (sensorID) {
    case SENSOR_BASINC:
      P_Basinc += Q_Basinc;
      K_Basinc = P_Basinc / (P_Basinc + R_Basinc);
      Basinc_Kalman += K_Basinc * (olcum - Basinc_Kalman);
      P_Basinc *= (1.0f - K_Basinc);
      return Basinc_Kalman;

    case SENSOR_IVME_Z:
      P_Z_Ivme += Q_Z_Ivme;
      K_Z_Ivme = P_Z_Ivme / (P_Z_Ivme + R_Z_Ivme);
      Z_Ivme_Kalman += K_Z_Ivme * (olcum - Z_Ivme_Kalman);
      P_Z_Ivme *= (1.0f - K_Z_Ivme);
      return Z_Ivme_Kalman;

    case SENSOR_IVME_X:
      P_X_Ivme += Q_X_Ivme;
      K_X_Ivme = P_X_Ivme / (P_X_Ivme + R_X_Ivme);
      X_Ivme_Kalman += K_X_Ivme * (olcum - X_Ivme_Kalman);
      P_X_Ivme *= (1.0f - K_X_Ivme);
      return X_Ivme_Kalman;

    case SENSOR_IVME_Y:
      P_Y_Ivme += Q_Y_Ivme;
      K_Y_Ivme = P_Y_Ivme / (P_Y_Ivme + R_Y_Ivme);
      Y_Ivme_Kalman += K_Y_Ivme * (olcum - Y_Ivme_Kalman);
      P_Y_Ivme *= (1.0f - K_Y_Ivme);
      return Y_Ivme_Kalman;

    default:
      return 0.0f;  // Geçersiz sensör ID
  }
}





//////////////////////////////////////////////////////
///////////////////////KALMAN/////////////////////////
//////////////////////////////////////////////////////






// Class Tanımlaması
Adafruit_BMP280 BMP;
LSM6DSM IMU;
TinyGPSPlus GPS;
HardwareSerial LoraSerial(2);
// Class Tanımlaması







// GLOBAL Değişken Tanımlamaları

// RAW VERI SETI
float AX, AY, AZ, GX, GY, GZ, P, Pi;
bool p_durum = false;

// GPS
float enlem, boylam, gps_irtifa;


// GLOBAL Değişken Tanımlamaları












TaskHandle_t MAX3232_Handle;     // ALWAYS ON TASK
TaskHandle_t Haberlesme_Handle;  // Y
TaskHandle_t NORMAL_MOD_Handle;  // X
TaskHandle_t SIT_MOD_Handle;     // X
TaskHandle_t SUT_MOD_Handle;     // X


void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(PATLAMAK, OUTPUT);

  digitalWrite(BUZZER, HIGH);

  // MAX3232
  Serial.begin(115200);  // 192500

  // LoRa ve GPS birlikte başlatma
  LoraSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  // BMP başlatma
  BMP.begin(0x76);

  // IMU başlatma
  IMU.begin();

  //  xTaskCreatePinnedToCore(
  //  KURTARMA, Task function.
  //  "Task2",    name of task.
  //  10000,      Stack size of task
  //  NULL,       parameter of the task
  //  1,          priority of the task
  //  &Task2,     Task handle to keep track of created task
  //  1);         pin task to core 1


  MOD_SHIFT(false);
  delay(200);
  /*
  // ÇEKİRDEK 1
  xTaskCreatePinnedToCore(
    MAX3232_LISTEN,
    "MAX3232_LISTEN",
    10000,
    NULL,
    1,
    &MAX3232_Handle,
    1);

*/
  delay(2000);
  digitalWrite(BUZZER, LOW);
}


enum MOD {
  NORMAL_MOD,
  SUT_MOD,
  SIT_MOD
};

MOD roket_mod = NORMAL_MOD;

void MOD_SHIFT(bool kill) {
#ifdef TESTMODU
  Serial.print("MOD SHIFT  ");
  Serial.println(roket_mod);
  Serial.println(BMP.readTemperature());

#endif
  if (kill) {
    vTaskDelete(Haberlesme_Handle);
    Haberlesme_Handle = NULL;
    vTaskDelete(NORMAL_MOD_Handle);
    NORMAL_MOD_Handle = NULL;
    vTaskDelete(SIT_MOD_Handle);
    SIT_MOD_Handle = NULL;
    vTaskDelete(SUT_MOD_Handle);
    SUT_MOD_Handle = NULL;
  }


  // EN İYİ ÇEKİRDEKLERİ TERCIH ET
  if (roket_mod == NORMAL_MOD) {
    xTaskCreatePinnedToCore(
      NormalAlgoritma,
      "X_NORMAL_MOD",
      10000,
      &aktif,
      1,
      &NORMAL_MOD_Handle,
      1);
    delay(200);
    xTaskCreatePinnedToCore(
      Haberlesme,
      "Y_HABERLESME",
      10000,
      NULL,
      1,
      &Haberlesme_Handle,
      0);
  } else if (roket_mod == SIT_MOD) {
    xTaskCreatePinnedToCore(
      NormalAlgoritma,
      "X_NORMAL_MOD",
      10000,
      &aktif,
      1,
      &NORMAL_MOD_Handle,
      1);

    xTaskCreatePinnedToCore(
      SITAlgoritma,
      "SIT_MOD",
      10000,
      NULL,
      1,
      &SIT_MOD_Handle,
      0);

  } else if (roket_mod == SUT_MOD) {
    //  NormalAlgoritma(false)
  }
}




// Paket tanımı
const uint8_t HEADER = 0xAA;
const uint8_t FOOTER1 = 0x0D;
const uint8_t FOOTER2 = 0x0A;

// Command Bloğu
const uint8_t CMD_SIT_START = 0x20;  // Sit
const uint8_t CMD_SUT_START = 0x22;  // Sut
const uint8_t CMD_STOP = 0x24;       // Durdur


void MAX3232_LISTEN(void* pvParameters) {
  if (Serial.available()) {
    if (Serial.peek() != HEADER) {
      Serial.read();
      return;
    }

    uint8_t buf[5];                      // [HEADER, CMD, CHECKSUM, F1, F2]
    if (Serial.available() < 5) return;  // tam gelmemiş sg
    for (int i = 0; i < 5; i++) {
      buf[i] = Serial.read();
    }

    //3) Basit footer kontrolü
    if (buf[3] != FOOTER1 || buf[4] != FOOTER2) {
      Serial.println(F("Footer hatalı, atlanıyor."));
      return;
    }

    uint8_t cmd = buf[1];
    uint8_t receivedCk = buf[2];

    // 4) Yenilenmiş Checksum doğrulama:
    //    Önce header ve command toplanır, sonra 0xFF ile mod alınır
    uint8_t calcCk = (uint8_t)(((uint16_t)HEADER + cmd) % 0xFF);

    if (calcCk != receivedCk) {
      // Serial.print(F("Checksum hatası! Hesaplanan=0x"));  // WOW
      // Serial.print(calcCk, HEX);
      // Serial.print(F("  Gelen=0x"));
      // Serial.println(receivedCk, HEX);
      return;
    }

    // IF MOD FARKLI O HALDE MOD FARKLI İSE START-A-FUNCTION
    // 5) Komuta göre fonksiyon çağır
    MOD yeni_mod_talebi;
    switch (cmd) {
      case CMD_SIT_START:
        yeni_mod_talebi = SIT_MOD;
        break;

      case CMD_SUT_START:
        yeni_mod_talebi = SUT_MOD;
        break;

      case CMD_STOP:
        yeni_mod_talebi = NORMAL_MOD;
        break;

      default:

        break;
    }


    if (yeni_mod_talebi != roket_mod) {
      MOD_SHIFT(true);
    }  // else WOW
  }
}


void SITAlgoritma(void* pvParameters) {
  // NORMAL ALGORİTMA + AŞAĞISI

  for (;;) {

    byte packetSit[36];
    int index = 0;  // packetSit

    // Başlık
    packetSit[index++] = 0xAB;


    float floats[8] = { Pi,
                        Basinc_Kalman,
                        X_Ivme_Kalman,
                        Y_Ivme_Kalman,
                        Z_Ivme_Kalman,
                        GX,
                        GY,
                        GZ };


    for (int i = 0; i < 8; i++) {
      byte* ptr = (byte*)(&floats[i]);
      for (int j = 0; j < 4; j++) {
        packetSit[index++] = ptr[j];
      }
    }

    packetSit[index++] = 0;     // checksum
    packetSit[index++] = 0x0D;  // 35
    packetSit[index++] = 0x0A;  // 36

    // Serial.print(packetSit);

    vTaskDelay(SIT_INTERVAL / portTICK_PERIOD_MS);  // 10 hazret
  }
}

void SUTAlgoritma(void* pvParameters) {
  for (;;) {
    KALMAN_KUR();
    KURTARMA();
  }
}


void NormalAlgoritma(void* pvParameters) {
#ifdef TESTMODU
  Serial.print("ALGORITM running on core ");
  Serial.println(xPortGetCoreID());
#endif

  bool RAW = *((bool*)pvParameters);  // void* → bool*

  for (;;) {
    if (RAW)
      SensorVeriOku();

    KALMAN_KUR();
    KURTARMA();

    vTaskDelay(pdMS_TO_TICKS(1));  // TODO: Yield
  }
}



void SensorVeriOku() {
  P = BMP.readPressure();
  Pi = BMP.readAltitude(SEALEVELPRESSURE_HPA);
  // X ve Znin yerleri değiştirilmiştir

  AX = IMU.readFloatAccelZ();
  AY = IMU.readFloatAccelY();
  AZ = IMU.readFloatAccelX();

  GX = IMU.readFloatGyroZ();
  GY = IMU.readFloatGyroY();
  GZ = IMU.readFloatGyroX();
}

void KALMAN_KUR() {
  kalmanla(AX, SENSOR_IVME_X);
  kalmanla(AY, SENSOR_IVME_Y);
  kalmanla(AZ, SENSOR_IVME_Z);
  kalmanla(P, SENSOR_BASINC);
}

unsigned long patlama_millis, basinc_millis;
float eski_basinc;

void KURTARMA() {
  bool irtifaKaybi = false, roketYatma = false;

  float yeni_basinc = Basinc_Kalman;

  if (yeni_basinc > eski_basinc+50)
    irtifaKaybi = true;

  unsigned long new_millis = millis();

  if (new_millis - basinc_millis >= 100) {
    basinc_millis = new_millis;
    eski_basinc = yeni_basinc;
  }


  float roketivme_X = abs(X_Ivme_Kalman) * 100;
  float roketivme_Y = abs(Y_Ivme_Kalman) * 100;

  if (roketivme_X > 75 || roketivme_Y > 75)
    roketYatma = true;

  bool kosul = (irtifaKaybi && roketYatma);

  if (kosul && !p_durum) {
    p_durum = true;
    digitalWrite(PATLAMAK, HIGH);
  }


  if (new_millis - patlama_millis >= PATLAMA_SURESI) {
    patlama_millis = new_millis;
    digitalWrite(PATLAMAK, LOW);
  }
}









// TODO: better packing
void Haberlesme(void* pvParameters) {
#ifdef TESTMODU
  Serial.print("Haberlesme running on core ");
  Serial.println(xPortGetCoreID());
#endif
  for (;;) {

    // ASYNC
    while (LoraSerial.available())
      GPS.encode(LoraSerial.read());


    if (GPS.location.isValid()) {
      enlem = GPS.location.lat();
      boylam = GPS.location.lng();
    }

    if (GPS.altitude.isValid()) {
      gps_irtifa = GPS.altitude.meters();
    }


    // HEADERLER
    LoraSerial.write((byte)0x00);
    LoraSerial.write(ALICI_ADRES);
    LoraSerial.write(ALICI_KANAL);
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
    LoraSerial.print(Pi);
    LoraSerial.print(",");

    // IMU
    LoraSerial.print("GX=");
    LoraSerial.print(GX);
    LoraSerial.print(",");

    LoraSerial.print("GY=");
    LoraSerial.print(GY);
    LoraSerial.print(",");

    LoraSerial.print("GZ=");
    LoraSerial.print(GZ);
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
