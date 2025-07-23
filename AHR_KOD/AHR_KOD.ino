//#define TESTMODU 1

#define LoraTX D9
#define GpsRX D13
//#define MAXTX D4
//#define MAXRX D12

#define BUZZER D4
#define PATLAMAK D12


#define PATLAMA_SURESI 1100    // ?
#define LORA_INTERVAL 900      // ?
#define SIT_INTERVAL 100       // 10HZ
#define GPS_INTERVAL 900       // 1HZ
#define MAX3232_INTERVAL 1000  // 1HZ

//#define ALICI_ADRES 21
//#define ALICI_KANAL 50
#define SEALEVELPRESSURE_HPA (991)
#define GRAVITY 9.80665

static bool aktif = true;
static bool pasif = false;

const uint8_t X_BOOTLOADER[] = {
  0x4D, 0x45, 0x48, 0x4D, 0x45, 0x54, 0x20,
  0x41, 0x4B, 0x49, 0x46, 0x20,
  0x59, 0x55, 0x43, 0x45, 0x00
};



#include <HardwareSerial.h>
#include <TinyGPS++.h>
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

//HardwareSerial MAX3232Serial(1);
HardwareSerial LoraSerial(2);
// Class Tanımlaması







// GLOBAL Değişken Tanımlamaları

double KALKIS_BASINC = 0.0;  // Başlangıç basıncı

// HYI RAW VERI SETI
float AX, AY, AZ, GX, GY, GZ, P, Pi, ACI;
bool p_durum = false;
bool irtifa_kosul = false;
bool aci_kosul = false;




// GPS
float enlem, boylam, gps_irtifa;  // DOUBLE

// GLOBAL Değişken Tanımlamaları






TaskHandle_t NORMAL_MOD_Handle;  // CORE 1
TaskHandle_t Haberlesme_Handle;  // CORE 0
TaskHandle_t OUKBTEST_Handle;    // CORE 0

void NormalAlgoritma(void* pvParameters);
void SITAlgoritma(void* pvParameters);
void SUTAlgoritma(void* pvParameters);
void Haberlesme(void* pvParameters);


void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(PATLAMAK, OUTPUT);
  digitalWrite(BUZZER, HIGH);


  // BMP başlatma
  BMP.begin(0x76);
  // IMU başlatma
  IMU.begin();

  // MAX3232 başlatma
  Serial.begin(115200);
  // MAX3232Serial.begin(115200, SERIAL_8N1, MAXRX, MAXTX);

  //LoRa ve GPS birlikte başlatma
  LoraSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);


  KALKIS_BASINC = BMP.readPressure() / 100;





#ifdef TESTMODU
  Serial.println();
  Serial.print("PUSULA ROKET TAKIMI TEST KIPI =  ");
  Serial.println((const char*)X_BOOTLOADER);
  Serial.print("KALKIS_BASINC:  ");
  Serial.println(KALKIS_BASINC);
  Serial.println();
#endif


  ParalelIslemYoneticisi(false);
  delay(500);
  digitalWrite(BUZZER, LOW);
  xTaskCreatePinnedToCore(
    NormalAlgoritma,
    "X_NORMAL_MOD",
    10000,
    NULL,
    1,
    &NORMAL_MOD_Handle,
    1);
}


enum MOD {
  NORMAL_MOD,
  SUT_MOD,
  SIT_MOD
};

MOD roket_mod = NORMAL_MOD;


void TaskKill(TaskHandle_t* handle) {
  if (*handle != NULL) {
    vTaskDelete(*handle);
    *handle = NULL;
  }
}


void ParalelIslemYoneticisi(bool kill) {
#ifdef TESTMODU
  Serial.println();
  Serial.print("MOD GECIS TALEBI ALINDI = ");
  Serial.println(roket_mod);
  Serial.println();
#endif

  if (kill) {
    TaskKill(&Haberlesme_Handle);
    TaskKill(&OUKBTEST_Handle);
#ifdef TESTMODU
    Serial.print("MOD GECIS TALEBI = gorevler olduruldu");
    Serial.println();
#endif
  }



  if (roket_mod == NORMAL_MOD) {
#ifdef TESTMODU
    Serial.println("MOD GECISI TALEBI = haberlesme mod devrede ");
#endif
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
    Serial.println("MOD GECISI TALEBI = SIT mod devrede ");
#endif
    xTaskCreatePinnedToCore(
      SITAlgoritma,
      "OUKBTEST_Handle",
      10000,
      NULL,
      1,
      &OUKBTEST_Handle,
      0);

  } else if (roket_mod == SUT_MOD) {
#ifdef TESTMODU
    Serial.println("MOD GECISI TALEBI = SUT mod devrede ");
#endif
    xTaskCreatePinnedToCore(
      SUTAlgoritma,
      "OUKBTEST_Handle",
      10000,
      NULL,
      1,
      &OUKBTEST_Handle,
      0);
    //  NormalAlgoritma(false)
  }
}


typedef union {
  float f;
  uint8_t b[4];
} FLOAT32_UINT8_DONUSTURUCU;

// LITTLE ENDIAN -> BIG ENDIAN
void FtoU(float* sayi, uint8_t* hedef) {
  FLOAT32_UINT8_DONUSTURUCU d;
  d.f = *sayi;

  for (int i = 0; i < 4; i++)
    hedef[i] = d.b[3 - i];
}

// BIG ENDIAN -> LITTLE ENDIAN
void UtoF(uint8_t* kaynak, float* sayi) {
  FLOAT32_UINT8_DONUSTURUCU d;

  for (int i = 0; i < 4; i++)
    d.b[3 - i] = kaynak[i];

  *sayi = d.f;
}


void SITAlgoritma(void* pvParameters) {
#ifdef TESTMODU
  Serial.println("SITAlgoritma yuruyor: ");
#endif

  for (;;) {
    // SIT'e Özel Veriler
    float RAKIM = BMP.readAltitude(SEALEVELPRESSURE_HPA);

    float xms2 = X_Ivme_Kalman * GRAVITY;
    float yms2 = Y_Ivme_Kalman * GRAVITY;
    float zms2 = Z_Ivme_Kalman * GRAVITY;

    float xAngle = atan2(X_Ivme_Kalman, sqrt(Y_Ivme_Kalman * Y_Ivme_Kalman + Z_Ivme_Kalman * Z_Ivme_Kalman));
    float yAngle = atan2(Y_Ivme_Kalman, sqrt(X_Ivme_Kalman * X_Ivme_Kalman + Z_Ivme_Kalman * Z_Ivme_Kalman));
    float zAngle = ACI;
    // SIT'e Özel Veriler


    uint8_t packet[36];  // 1 header + 8*4 veri + 1 checksum + 2 footer
    int i = 0;

    // Header
    packet[i++] = 0xAB;

    FtoU(&RAKIM, &packet[i]);
    i += 4;
    FtoU(&Basinc_Kalman, &packet[i]);
    i += 4;
    FtoU(&xms2, &packet[i]);
    i += 4;
    FtoU(&yms2, &packet[i]);
    i += 4;
    FtoU(&zms2, &packet[i]);
    i += 4;
    FtoU(&xAngle, &packet[i]);
    i += 4;  // MANYETOMETER LAZIM
    FtoU(&yAngle, &packet[i]);
    i += 4;  // MANYETOMETER LAZIM
    FtoU(&zAngle, &packet[i]);
    i += 4;  // MANYETOMETER LAZIM TODO

    // Checksum (ilk 34 byte)
    uint16_t sum = 0;
    for (int j = 0; j < 34; j++) {
      sum += packet[j];
    }
    packet[i++] = sum % 256;

    // Footer
    packet[i++] = 0x0D;
    packet[i++] = 0x0A;

#ifdef TESTMODU
    Serial.print("GIDECEK KOMUT SUM = ");
    Serial.println(sum);
    Serial.print("GIDECEK KOMUT CHECKSUM = ");
    Serial.println(packet[34]);

    Serial.print("GIDECEK KOMUT DUZENI = ");
    for (int j = 0; j < 36; j++) {
      if (packet[j] < 0x10) Serial.print("0");
      Serial.print(packet[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
#endif

    // Gönder
    Serial.write(packet, sizeof(packet));

    vTaskDelay(SIT_INTERVAL / portTICK_PERIOD_MS);
  }
}


void SUTAlgoritma(void* pvParameters) {
  for (;;) {

    uint8_t packet[6];
    packet[0] = 0xAA;

    uint8_t sonuc =
      (p_durum << 7) |       // Bit 7 – Ana paraşüt açma emri (ALGORITMA GLOBAL KOSUL)
      (0 << 6) |             // Bit 6 – Roket belirlenen irtifanın altına indi (şimdilik pasif)
      (0 << 5) |             // Bit 5 – Sürüklenme paraşütü açma emri (şimdilik pasif)
      (irtifa_kosul << 4) |  // Bit 4 – Roket irtifası alçalmaya başladı (ALGORITMA GLOBAL KOSUL)
      (aci_kosul << 3) |     // Bit 3 – Gövde açısı / ivme eşiği aşıldı (ALGORITMA GLOBAL KOSUL)
      (1 << 2) |             // Bit 2 – Minimum irtifa eşiği aşıldı (sabit 1)
      (0 << 1) |             // Bit 1 – Motor önlem süresi doldu (şimdilik pasif)
      (1 << 0);              // Bit 0 – Roket kalkışı algılandı (sabit 1)


    packet[1] = sonuc;  ///data 1
    packet[2] = 0x00;
    packet[3] = (0xAA + sonuc + 0x00) % 256;  // check
    packet[4] = 0x0D;
    packet[5] = 0x0A;
    Serial.write(packet, sizeof(packet));

    vTaskDelay(SIT_INTERVAL / portTICK_PERIOD_MS);
  }
}

unsigned long patlama_millis, basinc_millis, gps_millis, MAX_3232_millis;

void NormalAlgoritma(void* pvParameters) {
#ifdef TESTMODU
  Serial.print("NormalAlgoritma cekirdekte:  ");
  Serial.println(xPortGetCoreID());
#endif

  for (;;) {

    unsigned long new_millis = millis();

    if (new_millis - MAX_3232_millis >= MAX3232_INTERVAL) {
      MAX_3232_millis = new_millis;
      MAX3232_Dinle();
    }


    if (roket_mod != SUT_MOD)  //SUT Harici SYNC sensör okuması
      SensorVeriOku();

    KALMAN_KUR();
    KURTARMA();
    // yield();  // kodu yield ile dene TODO 5hz falan
  }
}




// Paket tanımı
const uint8_t HEADER = 0xAA;
const uint8_t FOOTER1 = 0x0D;
const uint8_t FOOTER2 = 0x0A;

// Command Bloğu
const uint8_t CMD_SIT_START = 0x20;  // Sit // 0xAA 0x20 0x8C 0x0D 0x0A
const uint8_t CMD_SUT_START = 0x22;  // Sut // 0xAA 0x22 0x8E 0x0D 0x0A
const uint8_t CMD_STOP = 0x24;       // Durdur // 0xAA 0x24 0x90 0x0D 0x0A
//#include "driver/uart.h"


void MAX3232_Dinle() {
  while (Serial.available()) {
    if (Serial.peek() != HEADER) {

#ifdef TESTMODU
      Serial.println("HEADER ERROR");
      Serial.println();
#endif

      // uart_flush_input(UART_NUM_0);
      break;
    }

    uint8_t buf[5];  // [HEADER, CMD, CHECKSUM, F1, F2]
    for (int i = 0; i < 5; i++)
      buf[i] = Serial.read();



    uint8_t cmd = buf[1];
    uint8_t receivedCk = buf[2];
    uint8_t calcCk = (HEADER + cmd) % 256;
    if (calcCk != receivedCk) {
#ifdef TESTMODU
      Serial.println("CHECKSUM ERROR");
      Serial.println();
#endif
      break;
    }




    if (buf[3] != FOOTER1 || buf[4] != FOOTER2) {

#ifdef TESTMODU
      Serial.println("FOOTER ERROR");
      Serial.println();
#endif

      break;
    }

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

    switch (cmd) {  // CHECKSUM BURADA CHECK EDILSIN
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
      ParalelIslemYoneticisi(true);
    }  // else WOW
  }
}


// RAW Veri Okuma. Kart pozisyonuna göre düzeltme yapar.
void SensorVeriOku() {
  P = BMP.readPressure() / 100;          // hpa
  Pi = BMP.readAltitude(KALKIS_BASINC);  //SEALEVELPRESSURE_HPA & metre

  // X ve Z'nin yerleri değiştirilmiştir

  AX = IMU.readFloatAccelZ();
  AY = IMU.readFloatAccelY();
  AZ = IMU.readFloatAccelX();

  GX = IMU.readFloatGyroZ();
  GY = IMU.readFloatGyroY();
  GZ = IMU.readFloatGyroX();

  // X ve Z'nin yerleri değiştirilmiştir

  ACI = atan2(sqrt(AX * AX + AY * AY), AZ) * 180.0 / PI;

  // AZ = AZ * -1; ///////////////
  ////////////////// onemlı
  //////////////////kart ters ise çarp
}

void KALMAN_KUR() {
  kalmanla(AX, SENSOR_IVME_X);
  kalmanla(AY, SENSOR_IVME_Y);
  kalmanla(AZ, SENSOR_IVME_Z);
  kalmanla(P, SENSOR_BASINC);
}

float eski_basinc;

void KURTARMA() {
  unsigned long new_millis = millis();
  float yeni_basinc = Basinc_Kalman;

  irtifa_kosul = (yeni_basinc > eski_basinc);

  if (new_millis - basinc_millis >= 100) {
    basinc_millis = new_millis;
    eski_basinc = yeni_basinc;
  }

  //TODO: directly from aci. aci. aci. aci
  int IvmeKosulX = abs(X_Ivme_Kalman) * 100 > 75;
  int IvmeKosulY = abs(Y_Ivme_Kalman) * 100 > 75;

  aci_kosul = (IvmeKosulX || IvmeKosulY);

  bool kosul = (aci_kosul && aci_kosul);

  if (kosul && !p_durum) {
    p_durum = true;
    digitalWrite(PATLAMAK, HIGH);
  }


  if (new_millis - patlama_millis >= PATLAMA_SURESI) {
    patlama_millis = new_millis;
    digitalWrite(PATLAMAK, LOW);
  }
}


#define HEADER_BYTE 0xAB
#define FOOTER_BYTE 0x56

typedef struct __attribute__((packed)) {
  uint8_t header;  // 1 byte

  float enlem;          // 4 byte
  float boylam;         // 4 byte
  float gps_irtifa;     // 4 byte
  float basinc;         // 4 byte
  float basinc_irtifa;  // 4 byte
  float ivmex;          // 4 byte
  float ivmey;          // 4 byte
  float ivmez;          // 4 byte
  float gyrox;          // 4 byte
  float gyroy;          // 4 byte
  float gyroz;          // 4 byte
  float aci;            // 4 byte

  uint8_t parasut_durum;  // 1 byte
  uint8_t checksum;       // 1 byte
  uint8_t footer;         // 1 byte

} VeriPaketi;


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

    VeriPaketi paket;
    // Header
    paket.header = HEADER_BYTE;

    // GPS
    paket.enlem = enlem;
    paket.boylam = boylam;
    paket.gps_irtifa = gps_irtifa;

    // BMP
    paket.basinc = Basinc_Kalman;
    paket.basinc_irtifa = Pi;

    // IMU
    paket.ivmex = X_Ivme_Kalman;
    paket.ivmey = Y_Ivme_Kalman;
    paket.ivmez = Z_Ivme_Kalman;
    paket.gyrox = GX;
    paket.gyroy = GY;
    paket.gyroz = GZ;
    paket.aci = ACI;

    paket.parasut_durum = p_durum ? 1 : 0;  //

    uint8_t* gonderilecekPaket = (uint8_t*)&paket;
    
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(VeriPaketi) - 2; ++i)
      sum += gonderilecekPaket[i];

    paket.checksum = sum % 256;

    paket.footer = FOOTER_BYTE;

    LoraSerial.write(gonderilecekPaket, sizeof(paket));
    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }// TODOvTaskDelay convert to diğer şey
}

void loop(){};