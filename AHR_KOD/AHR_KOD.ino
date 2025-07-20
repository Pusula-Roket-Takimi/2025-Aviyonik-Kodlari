//#define TESTMODU 1

#define LoraTX D9
#define GpsRX D13
#define BUZZER D4
#define PATLAMAK D12

#define ALICI_ADRES 31
#define ALICI_KANAL 50

#define PATLAMA_SURESI 1100
#define LORA_INTERVAL 900
#define SIT_INTERVAL 100
#define GPS_INTERVAL 900

#define SEALEVELPRESSURE_HPA (991)

static bool aktif = true;
static bool pasif = false;

const uint8_t X_BOOTLOADER[] = {
  0x4D, 0x45, 0x48, 0x4D, 0x45, 0x54, 0x20,
  0x41, 0x4B, 0xC4, 0xB0, 0x46, 0x20,
  0x59, 0xC3, 0x9C, 0x43, 0x45, 0x00
};



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
//HardwareSerial MAX3232Serial(2);

HardwareSerial LoraSerial(2);
// Class Tanımlaması







// GLOBAL Değişken Tanımlamaları
double KALKIS_BASINC= 0.0;  // Başlangıç basıncı
// RAW VERI SETI
float AX, AY, AZ, GX, GY, GZ, P, Pi, ACI;
bool p_durum = false;

// GPS
float enlem, boylam, gps_irtifa;// DOUBLE
// or double takviye

// GLOBAL Değişken Tanımlamaları












TaskHandle_t MAX3232_Handle;     // ALWAYS ON TASK / CORE 1
TaskHandle_t Haberlesme_Handle;  // CORE 0
TaskHandle_t NORMAL_MOD_Handle;  // CORE 1
TaskHandle_t SIT_MOD_Handle;     // CORE 0
TaskHandle_t SUT_MOD_Handle;     // CORE 0

void NormalAlgoritma(void* pvParameters);
void SITAlgoritma(void* pvParameters);
void SUTAlgoritma(void* pvParameters);
void Haberlesme(void* pvParameters);


void setup() {

  pinMode(BUZZER, OUTPUT);
  pinMode(PATLAMAK, OUTPUT);

  digitalWrite(BUZZER, HIGH);

  // MAX3232
  Serial.begin(115200);  // 192500

#ifdef TESTMODU
  Serial.println();
  Serial.print("PUSULA ROKET TAKIMI TEST KIPI =  ");
  Serial.println((const char*)X_BOOTLOADER);
  Serial.println();
#endif


  // LoRa ve GPS birlikte başlatma
  LoraSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  // BMP başlatma
  BMP.begin(0x76);
  KALKIS_BASINC = BMP.readPressure();

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
  digitalWrite(BUZZER, LOW);
  // ÇEKİRDEK 1
  xTaskCreatePinnedToCore(
    MAX3232_LISTEN,
    "MAX3232_LISTEN",
    10000,
    NULL,
    1,
    &MAX3232_Handle,
    0);
}


enum MOD {
  NORMAL_MOD,
  SUT_MOD,
  SIT_MOD
};

MOD roket_mod = NORMAL_MOD;


void SafeDeleteTask(TaskHandle_t* handle) {
  if (*handle != NULL) {
    vTaskDelete(*handle);
    *handle = NULL;
  }
}


void MOD_SHIFT(bool kill) {
#ifdef TESTMODU
  Serial.println();
  Serial.print("MOD GECIS TALEBI ALINDI =  ");
  Serial.println(roket_mod);
  Serial.println();
#endif

  if (kill) {
    SafeDeleteTask(&Haberlesme_Handle);
    SafeDeleteTask(&NORMAL_MOD_Handle);
    SafeDeleteTask(&SIT_MOD_Handle);
    SafeDeleteTask(&SUT_MOD_Handle);
  }

#ifdef TESTMODU
  Serial.print("MOD GECIS TALEBI ONAYLANDI = Gorevler olduruldu");
  Serial.println();
#endif

  // EN İYİ ÇEKİRDEKLERİ TERCIH ET
  if (roket_mod == NORMAL_MOD) {


#ifdef TESTMODU
    Serial.println("MOD GECISI TALEBI SONLANDI = normal mod devrede ");
#endif


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
      "HABERLESME",
      10000,
      NULL,
      1,
      &Haberlesme_Handle,
      0);

  } else if (roket_mod == SIT_MOD) {
#ifdef TESTMODU
    Serial.println("MOD GECISI TALEBI SONLANDI = SIT mod devrede ");
#endif

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
  for (;;) {
    while (Serial.available()) {

      if (Serial.peek() != HEADER) {

#ifdef TESTMODU
        Serial.println("HEADER ERROR");
        Serial.println();
#endif

        Serial.read();
        break;
      }

      uint8_t buf[5];  // [HEADER, CMD, CHECKSUM, F1, F2]
      for (int i = 0; i < 5; i++)
        buf[i] = Serial.read();

        // TODO: CHECKSUM CHECK EDILECEK


      if (buf[3] != FOOTER1 || buf[4] != FOOTER2) {

#ifdef TESTMODU
        Serial.println("FOOTER ERROR");
        Serial.println();
#endif

        break;
      }


      uint8_t cmd = buf[1];
      uint8_t receivedCk = buf[2];

#ifdef TESTMODU
      Serial.print("GELEN KOMUT DUZENI = ");

      for (int i = 0; i < 5; i++)
        Serial.print(buf[i], HEX);

      Serial.println();
#endif

      MOD yeni_mod_talebi = roket_mod;


#ifdef TESTMODU
      Serial.print("Eski MOD  = ");
      Serial.println(yeni_mod_talebi);
#endif

      switch (cmd) {// CHECKSUM BURADA CHECK EDILSIN
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
#ifdef TESTMODU
      Serial.print("Yeni MOD = ");
      Serial.println(yeni_mod_talebi);
#endif

      if (yeni_mod_talebi != roket_mod) {
        roket_mod = yeni_mod_talebi;
#ifdef TESTMODU
        Serial.print("TALEP DEGISIMI YAPILIYOR ");
#endif
        MOD_SHIFT(true);
      }  // else WOW
    }
   //! yield(); TODO
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void SITAlgoritma(void* pvParameters) {
  // NORMAL ALGORİTMA + AŞAĞISI
#ifdef TESTMODU
  Serial.print("SITAlgoritma yuruyor:  ");
#endif
  for (;;) {

    byte packetSit[36];
    int index = 0;  // packetSit
    // HEADER
    packetSit[index++] = 0xAB;


    float floats[8] = { Pi,
                        Basinc_Kalman,
                        X_Ivme_Kalman,
                        Y_Ivme_Kalman,
                        Z_Ivme_Kalman,
                        GX,  // AÇI İSTER Kİ... TODO
                        GY,
                        GZ };


    for (int i = 0; i < 8; i++) {
      byte* ptr = (byte*)(&floats[i]);
      for (int j = 0; j < 4; j++) {
        packetSit[index++] = ptr[j];
      }
    }

    int sum = 0;
    for (int i = 0; i < 34; i++) {
      sum += packetSit[i];
    }

    int checksum = sum % 256;
#ifdef TESTMODU


     Serial.print("GIDECEK KOMUT SUM = ");
    Serial.println(sum);


    Serial.print("GIDECEK KOMUT CHECKSUM = ");
    Serial.println(checksum);

    Serial.println();
#endif

    packetSit[index++] = checksum;  // checksum

    packetSit[index++] = 0x0D;  // 35
    packetSit[index++] = 0x0A;  // 36

#ifdef TESTMODU


    Serial.print("GIDECEK KOMUT DUZENI = ");

    for (int i = 0; i < 36; i++) {
      if (packetSit[i] < 0x10) Serial.print("0");  // Tek basamaklı HEX için başına 0 koy
      Serial.print(packetSit[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
#endif


    Serial.write(packetSit, sizeof(packetSit));

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
  Serial.print("NormalAlgoritma yuruyor:  ");
  Serial.println(xPortGetCoreID());
#endif

  bool RAW = *((bool*)pvParameters);  // void* → bool*

  for (;;) {
    if (RAW)//if roket mod !=sut
      SensorVeriOku();
    KALMAN_KUR();

    KURTARMA();
    yield();// kodu yield ile dene TODO 5hz falan
  }
}



void SensorVeriOku() {
  P = BMP.readPressure();
  Pi = BMP.readAltitude(KALKIS_BASINC);//SEALEVELPRESSURE_HPA

  // X ve Z'nin yerleri değiştirilmiştir

  AX = IMU.readFloatAccelZ();
  AY = IMU.readFloatAccelY();
  AZ = IMU.readFloatAccelX();

  GX = IMU.readFloatGyroZ();
  GY = IMU.readFloatGyroY();
  GZ = IMU.readFloatGyroX();

  // X ve Z'nin yerleri değiştirilmiştir

  ACI = atan2(sqrt(AX * AX + AY * AY), AZ) * 180.0 / PI;

}

void KALMAN_KUR() {
  kalmanla(AX, SENSOR_IVME_X);
  kalmanla(AY, SENSOR_IVME_Y);
  kalmanla(AZ, SENSOR_IVME_Z);
  kalmanla(P, SENSOR_BASINC);
}

unsigned long patlama_millis, basinc_millis, gps_millis;
float eski_basinc;

void KURTARMA() {
  bool irtifaKaybi = false, roketYatma = false;

  float yeni_basinc = Basinc_Kalman;

  if (yeni_basinc > eski_basinc + 50)
    irtifaKaybi = true;

  unsigned long new_millis = millis();

  if (new_millis - basinc_millis >= 100) {
    basinc_millis = new_millis;
    eski_basinc = yeni_basinc;
  }


  int IvmeKosulX = abs(X_Ivme_Kalman) * 100 > 75;
  int IvmeKosulY = abs(Y_Ivme_Kalman) * 100 > 75;

  if (IvmeKosulX || IvmeKosulY)
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
  Serial.print("Haberlesme yuruyor:    ");
  Serial.println(xPortGetCoreID());
#endif

  while (true) {


  unsigned long new_millis = millis();

  if (new_millis - gps_millis >= GPS_INTERVAL) {
    gps_millis = new_millis;

    // GPS
    while (LoraSerial.available())
      GPS.encode(LoraSerial.read());

    if (GPS.location.isValid()) {
      enlem = GPS.location.lat();
      boylam = GPS.location.lng();
    }

    if (GPS.altitude.isValid()) {
      gps_irtifa = GPS.altitude.meters();
    }
  }
  
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

// BETA
       LoraSerial.print("D=");
    LoraSerial.print(ACI);
    LoraSerial.print(",");
//
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
