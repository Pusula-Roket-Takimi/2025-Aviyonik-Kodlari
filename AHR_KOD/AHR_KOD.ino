#define TESTMODU 1

//#define ALICI_ADRES 21
//#define ALICI_KANAL 50
//#define NET_ID 46


#define LoraTX D9
#define GpsRX D13
#define MAXTX D0
#define MAXRX D1

#define BUZZER D4
#define PATLAMAK D12


#define PATLAMA_SURESI 1500    // SGU EYLEM SÜRESİ
#define LORA_INTERVAL 1000     // 2.5HZ. 2HZ yeterli aslında. Ama 1 hz kullanalım
#define KURTARMA_INTERVAL 100  // 10HZ
#define UKBTEST_INTERVAL 100   // 10HZ
#define GPS_INTERVAL 900       // 1HZ
#define MAX3232_INTERVAL 1000  // 1HZ

#define SEALEVELPRESSURE_HPA (1013.25)
#define GRAVITY 9.80665

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
// Basınç
float Q_Irtifa = 0.001;
float R_Irtifa = 0.1;
float P_Irtifa = 1.0;
float Irtifa_Kalman = 0.0;
float K_Irtifa = 0.0;


enum {
  SENSOR_BASINC,
  SENSOR_IVME_Z,
  SENSOR_IVME_X,
  SENSOR_IVME_Y,
  SENSOR_IRTIFA
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
    case SENSOR_IRTIFA:
      P_Irtifa += Q_Irtifa;
      K_Irtifa = P_Irtifa / (P_Irtifa + R_Irtifa);
      Irtifa_Kalman += K_Irtifa * (olcum - Irtifa_Kalman);
      P_Irtifa *= (1.0f - K_Irtifa);
      return Irtifa_Kalman;

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

HardwareSerial MAX3232Serial(1);
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

// Millis seti
unsigned long patlama_millis, basinc_millis, gps_millis, MAX_3232_millis;

enum MOD {
  NORMAL_MOD,
  SUT_MOD,
  SIT_MOD
};

MOD roket_mod = NORMAL_MOD;



TaskHandle_t Haberlesme_Handle;
TaskHandle_t OUKBTEST_Handle;

// GLOBAL Değişken Tanımlamaları

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(PATLAMAK, OUTPUT);
  digitalWrite(BUZZER, HIGH);


  // BMP başlatma
  BMP.begin(0x76);

  // IMU başlatma
  IMU.begin();

  // MAX3232 başlatma
  MAX3232Serial.begin(115200, SERIAL_8N1, MAXRX, MAXTX);

  //LoRa ve GPS birlikte başlatma
  LoraSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);


  KALKIS_BASINC = BMP.readPressure() / 100;


#ifdef TESTMODU
  Serial.begin(115200);
  Serial.println();
  Serial.print("PUSULA ROKET TAKIMI TEST KIPI =  ");
  Serial.println((const char*)X_BOOTLOADER);
  Serial.print("KALKIS_BASINC:  ");
  Serial.println(KALKIS_BASINC);
  Serial.println();
#endif


  ParalelIslemYoneticisi();
  delay(500);
  digitalWrite(BUZZER, LOW);
}

void TaskKill(TaskHandle_t* handle) {
  if (*handle != NULL) {
    vTaskDelete(*handle);
    *handle = NULL;
  }
}


void ParalelIslemYoneticisi() {
#ifdef TESTMODU
  Serial.print("MOD GECIS TALEBI ALINDI = ");
  Serial.println(roket_mod);
#endif

  if (roket_mod == NORMAL_MOD) {
    TaskKill(&OUKBTEST_Handle);
    xTaskCreatePinnedToCore(
      Haberlesme,
      "HABERLESME",
      10000,
      NULL,
      1,
      &Haberlesme_Handle,
      0);

  } else if (roket_mod == SIT_MOD) {
    TaskKill(&Haberlesme_Handle);
    xTaskCreatePinnedToCore(
      SITAlgoritma,
      "OUKBTEST_Handle",
      10000,
      NULL,
      1,
      &OUKBTEST_Handle,
      0);

  } else if (roket_mod == SUT_MOD) {
    TaskKill(&Haberlesme_Handle);
    delay(200);//TODO sıfırlama başarısız devreye giriyor - high
    p_durum = false; 
    Pi=0;
    ACI=0;

    xTaskCreatePinnedToCore(
      SUTAlgoritma,
      "OUKBTEST_Handle",
      10000,
      NULL,
      1,
      &OUKBTEST_Handle,
      0);
  }
}


const uint8_t TELEMETRI_SIZE = 36;
const uint8_t HEADER = 0xAA;
const uint8_t TELEMETRI_HEADER = 0xAB;
const uint8_t FOOTER1 = 0x0D;
const uint8_t FOOTER2 = 0x0A;

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
  Serial.print("SITAlgoritma ONCORE: ");
  Serial.println(xPortGetCoreID());
#endif

  while (true) {

    //////////////////////////////////////////////////////////////////////////////// SIT'e Özel Veriler
    float RAKIM = BMP.readAltitude(SEALEVELPRESSURE_HPA);

    float xms2 = X_Ivme_Kalman * GRAVITY;
    float yms2 = Y_Ivme_Kalman * GRAVITY;
    float zms2 = Z_Ivme_Kalman * GRAVITY;

    float xAngle = atan2(X_Ivme_Kalman, sqrt(Y_Ivme_Kalman * Y_Ivme_Kalman + Z_Ivme_Kalman * Z_Ivme_Kalman));
    float yAngle = atan2(Y_Ivme_Kalman, sqrt(X_Ivme_Kalman * X_Ivme_Kalman + Z_Ivme_Kalman * Z_Ivme_Kalman));
    float zAngle = ACI;
    //////////////////////////////////////////////////////////////////////////////// SIT'e Özel Veriler


    uint8_t packet[TELEMETRI_SIZE];
    int i = 0;

    // Header
    packet[i++] = TELEMETRI_HEADER;


    // Veriler
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
    i += 4;
    FtoU(&yAngle, &packet[i]);
    i += 4;
    FtoU(&zAngle, &packet[i]);
    i += 4;  // TODO: MANYETOMETER LAZIM - mid

    int sum = 0;
    for (int j = 0; j < TELEMETRI_HEADER - 2; j++) sum += packet[j];

    // Checksum
    packet[i++] = sum % 256;

    // Footer
    packet[i++] = FOOTER1;
    packet[i++] = FOOTER2;

    // Gönder
    MAX3232Serial.write(packet, sizeof(packet));
    vTaskDelay(UKBTEST_INTERVAL / portTICK_PERIOD_MS);  // TODO UNTIL TASK DELAY - low
  }
}


void SUTAlgoritma(void* pvParameters) {
#ifdef TESTMODU
  Serial.print("SUTAlgoritma ONCORE: ");
  Serial.println(xPortGetCoreID());
#endif

  while (true) {
    uint8_t packet[6];
    packet[0] = HEADER;

    uint8_t sonuc =
      (p_durum << 7) |       // Bit 7 – Ana paraşüt açma emri (ALGORITMA GLOBAL KOSUL)
      (0 << 6) |             // Bit 6 – Roket belirlenen irtifanın altına indi (şimdilik pasif)
      (0 << 5) |             // Bit 5 – Sürüklenme paraşütü açma emri (0)
      (irtifa_kosul << 4) |  // Bit 4 – Roket irtifası alçalmaya başladı (ALGORITMA GLOBAL KOSUL)
      (aci_kosul << 3) |     // Bit 3 – Gövde açısı / ivme eşiği aşıldı (ALGORITMA GLOBAL KOSUL)
      (0 << 2) |             // Bit 2 – Minimum irtifa eşiği aşıldı (sabit 0)
      (0 << 1) |             // Bit 1 – Motor önlem süresi doldu (şimdilik pasif)
      (0 << 0);              // Bit 0 – Roket kalkışı algılandı (sabit 0)


    packet[1] = sonuc;  // data 1
    packet[2] = 0x00;   // data 2


    int sum = 0;
    for (int i = 0; i < 3; i++)
      sum += packet[i];

    packet[3] = sum % 256;  // checksum
    packet[4] = FOOTER1;    //
    packet[5] = FOOTER2;
    MAX3232Serial.write(packet, sizeof(packet));

    vTaskDelay(UKBTEST_INTERVAL / portTICK_PERIOD_MS);
  }
}





// KOMUT SETI
const uint8_t CMD_SIT_START = 0x20;  // Sit // 0xAA 0x20 0x8C 0x0D 0x0A
const uint8_t CMD_SUT_START = 0x22;  // Sut // 0xAA 0x22 0x8E 0x0D 0x0A
const uint8_t CMD_STOP = 0x24;       // Durdur // 0xAA 0x24 0x90 0x0D 0x0A

struct {  //TODO sil veya kullan bunu.. yA DA gyroyu sadece haberleşmeye falan yaz.. yani gyroyu global saklama ve kurtarmaları sync olarak parametre olarak ver fonksiyonlara:! - high
  float irtifa;
  float basinc;
  float ivmeX;
  float ivmeY;
  float ivmeZ;
  float aciX;
  float aciY;
  float aciZ;
  uint8_t checksum;
} sonuc;


void MAX3232_Dinle() {
  while (MAX3232Serial.available()) {
    if (roket_mod == SUT_MOD) {
      if (MAX3232Serial.peek() == TELEMETRI_HEADER) {

        uint8_t paket[TELEMETRI_SIZE];
        MAX3232Serial.readBytes(paket, TELEMETRI_SIZE); 

        // Paket bitiş baytları
        if (paket[TELEMETRI_SIZE - 2] != FOOTER1 || paket[TELEMETRI_SIZE - 1] != FOOTER2)
          break;

        // Checksum kontrolü
        uint8_t hesaplanan = 0;
        for (int i = 0; i < TELEMETRI_SIZE - 3; i++)
          hesaplanan += paket[i];

        if (hesaplanan != paket[TELEMETRI_SIZE - 3]) break;

//float AX, AY, AZ, GX, GY, GZ, P, Pi, ACI;

        // Float verileri dönüştür
        int offset = 1;
        UtoF(&paket[offset], &sonuc.irtifa);
        offset += 4;
        UtoF(&paket[offset], &sonuc.basinc);
        offset += 4;
        UtoF(&paket[offset], &sonuc.ivmeX);
        offset += 4;
        UtoF(&paket[offset], &sonuc.ivmeY);
        offset += 4;
        UtoF(&paket[offset], &sonuc.ivmeZ);
        offset += 4;
        UtoF(&paket[offset], &sonuc.aciX);
        offset += 4;
        UtoF(&paket[offset], &sonuc.aciY);
        offset += 4;
        UtoF(&paket[offset], &sonuc.aciZ);
        offset += 4;


        Pi=sonuc.irtifa;
        ACI=sonuc.aciY;

#ifdef TESTMODU
        Serial.print("Gelen ornek irtifa  ");
        Serial.println(sonuc.irtifa);
        Serial.print("Gelen ornek basinc  ");
        Serial.println(sonuc.basinc);
         Serial.print("Gelen ornek aciY  ");
        Serial.println(sonuc.aciY);


#endif

        break;
      }
    }


    uint8_t kafa = MAX3232Serial.read();
    if (kafa != HEADER) 
      break;
    
    uint8_t buf[4];
    for (int i = 0; i < 4; i++)
      buf[i] = MAX3232Serial.read();

    uint8_t cmd = buf[0];
    uint8_t receivedCk = buf[1];
    uint8_t calcCk = (kafa + cmd) % 256;
    if (calcCk != receivedCk || buf[2] != FOOTER1 || buf[3] != FOOTER2)
      break;

    MOD yeni_mod_talebi = roket_mod;

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
      roket_mod = yeni_mod_talebi;
      ParalelIslemYoneticisi();
    }
  }
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
float readAltitude(float pressure, float seaLevelhPa) {
 return 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
}

// RAW Veri Okuma. Kart pozisyonuna göre düzeltme yapar.
void SensorVeriOku() {
  P = BMP.readPressure() / 100;          // hpa
  Pi = readAltitude(P, KALKIS_BASINC);  //SEALEVELPRESSURE_HPA & metre


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

float eski_irtifa;

void KURTARMA() {
  unsigned long new_millis = millis();
  float yeni_irtifa = Pi;

  irtifa_kosul = (yeni_irtifa < eski_irtifa);

  if (new_millis - basinc_millis >= 100) {
    basinc_millis = new_millis;
    eski_irtifa = yeni_irtifa;
  }

  aci_kosul = ACI > 60;

  bool kosul = (aci_kosul && irtifa_kosul);

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

    paket.parasut_durum = p_durum ? 2 : 1;  //

    uint8_t* gonderilecekPaket = (uint8_t*)&paket;

    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(VeriPaketi) - 2; ++i)
      sum += gonderilecekPaket[i];

    paket.checksum = sum % 256;

    paket.footer = FOOTER_BYTE;

    LoraSerial.write(gonderilecekPaket, sizeof(paket));
    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }
}

void loop() {
  if (roket_mod == SUT_MOD) {
    MAX3232_Dinle();
  } else {
    SensorVeriOku();
    unsigned long new_millis = millis();
    if (new_millis - MAX_3232_millis >= MAX3232_INTERVAL) {
      MAX_3232_millis = new_millis;
      MAX3232_Dinle();
    }
  }

  KALMAN_KUR();
  KURTARMA();
  vTaskDelay(KURTARMA_INTERVAL / portTICK_PERIOD_MS);  // CPU HAFIFLET
};