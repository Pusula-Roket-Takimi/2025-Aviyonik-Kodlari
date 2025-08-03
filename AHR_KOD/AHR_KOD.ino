//#define TESTMODU 1

//#define ALICI_ADRES 97
//#define ALICI_KANAL 18
//#define NET_ID 46


#define LoraTX D9
#define GpsRX D13
#define MAXTX D0
#define MAXRX D1

#define BUZZER D4
#define PATLAMAK D12




#define MAIN_INTERVAL 100     // Ana Döngü: 10HZ
#define PATLAMA_SURESI 10000   // SGU EYLEM SÜRESİ
#define LORA_INTERVAL 900     // must= < GPS 1 hz kullanalım
#define UKBTEST_INTERVAL 100  // 10HZ
#define GPS_INTERVAL 900      // 1HZ

#define SEALEVELPRESSURE_HPA 994  // hava durumuna bak // aksaray 1003
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

class Kalman {
private:
  const float Q;  // Süreç gürültüsü (Düşük. ESP32 veri kaybetmez. Çok az kaybedebilir Floatları sadece.)
  const float R;  // Ölçüm gürültüsü (Değişken)
  float P;        // hata kovaryansı (Q ama sadece başlangıç. Q'nun başlangıçta yumuşak olmasını sağlar)

public:
  float X;  // tahmin edilen değer (public)

  Kalman(float q, float r, float p, float initial)
    : Q(q), R(r), P(p), X(initial) {}

  float update(float olcum) {
    P += Q;
    float K = P / (P + R);
    X += K * (olcum - X);
    P *= (1 - K);
    return X;
  }
  void reset(float initialX = 0.0, float initialP = 1.0) {
    X = initialX;
    P = initialP;
  }
};


Kalman Basinc_Kalman(0.001, 1 * 1 /* tolerans ** 2 */, 1.0 /* arastir */, 0.0);
Kalman XIvme_Kalman(0.001, 0.1, 1.0, 0.0);
Kalman YIvme_Kalman(0.001, 0.1, 1.0, 0.0);
Kalman ZIvme_Kalman(0.001, 0.1, 1.0, 0.0);
Kalman Irtifa_Kalman(0.001, 1 * 1, 1.0, 0.0);
Kalman Aci_Kalman(0.001, 0.1, 1.0, 0.0);




//////////////////////////////////////////////////////
///////////////////////KALMAN/////////////////////////
//////////////////////////////////////////////////////






// Class Tanımlaması
Adafruit_BMP280 BMP;
LSM6DSM IMU;
TinyGPSPlus GPS;

HardwareSerial MAX3232Serial(1);
HardwareSerial ComboSerial(2);
// Class Tanımlaması







// GLOBAL Değişken Tanımlamaları

// HYI/SUT RAW BIT SETI
bool ana_parasut = false;
bool irtifa_kosul = false;
bool aci_kosul = false;

// BMP
double KALKIS_BASINC;  // Başlangıç basıncı

// RAW EK-6 DATA
struct {
  float RAMPA_IRTIFA;
  float BASINC;
  float ACI;
  float AX, AY, AZ;
} RawSensorData;

// Millis seti
unsigned long patlama_millis, irtifa_millis, gps_millis;  //, MAX_3232_millis;

enum MOD { NORMAL_MOD,
           SUT_MOD,
           SIT_MOD };

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
  ComboSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  delay(100);
  KALKIS_BASINC = BMP.readPressure() / 100;
  delay(100);

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
  digitalWrite(BUZZER, LOW);
}


void loop() {
  if (roket_mod != SUT_MOD)
    SensorVeriOku();

  MAX3232_Dinle();

  Basinc_Kalman.update(RawSensorData.BASINC);
  XIvme_Kalman.update(RawSensorData.AX);
  YIvme_Kalman.update(RawSensorData.AY);
  ZIvme_Kalman.update(RawSensorData.AZ);
  Irtifa_Kalman.update(RawSensorData.RAMPA_IRTIFA);
  Aci_Kalman.update(RawSensorData.ACI);

  KURTARMA(Aci_Kalman.X, Irtifa_Kalman.X);

  vTaskDelay(MAIN_INTERVAL / portTICK_PERIOD_MS);
};



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
  RawSensorData.BASINC = BMP.readPressure() / 100;
  RawSensorData.RAMPA_IRTIFA = readAltitude(RawSensorData.BASINC, KALKIS_BASINC);

  // X ve Z'nin yerleri değiştirilmiştir
  RawSensorData.AX = IMU.readFloatAccelZ();
  RawSensorData.AY = IMU.readFloatAccelY();
  RawSensorData.AZ = IMU.readFloatAccelX();  // kart tersse ters yap bunu mesela
  // X ve Z'nin yerleri değiştirilmiştir


  RawSensorData.ACI = fabs(atan2(sqrt(RawSensorData.AX * RawSensorData.AX + RawSensorData.AY * RawSensorData.AY), -RawSensorData.AZ) * 180.0 / PI);
#ifdef TESTMODU

  Serial.println("=== SENSOR VERISI ===");
  Serial.print("BASINC (hPa): ");
  Serial.println(RawSensorData.BASINC, 2);
  Serial.print("RAMPA_IRTIFA (m): ");
  Serial.println(RawSensorData.RAMPA_IRTIFA, 2);

  Serial.print("Ivme X (AX): ");
  Serial.println(RawSensorData.AX, 4);
  Serial.print("Ivme Y (AY): ");
  Serial.println(RawSensorData.AY, 4);
  Serial.print("Ivme Z (AZ): ");
  Serial.println(RawSensorData.AZ, 4);

  Serial.print("ACI (°): ");
  Serial.println(RawSensorData.ACI, 2);
  Serial.println("=====================");
#endif
}


float eski_irtifa;

void KURTARMA(float aci, float irtifa) {
  unsigned long new_millis = millis();

  irtifa_kosul = (irtifa < eski_irtifa);
  if (new_millis - irtifa_millis >= 100) {
    irtifa_millis = new_millis;
    eski_irtifa = irtifa;
  }

  aci_kosul = aci > 45;

#ifdef TESTMODU
  Serial.println(aci);
#endif

  bool kosul = (aci_kosul && irtifa_kosul);

  if (kosul && !ana_parasut) {
    ana_parasut = true;
    digitalWrite(PATLAMAK, HIGH);
    patlama_millis = new_millis;
  }

  if (digitalRead(PATLAMAK))
    if (new_millis - patlama_millis >= PATLAMA_SURESI) {
      patlama_millis = new_millis;
      digitalWrite(PATLAMAK, LOW);
    }
}




// KOMUT SETI
const uint8_t CMD_SIT_START = 0x20;  // Sit // 0xAA 0x20 0x8C 0x0D 0x0A
const uint8_t CMD_SUT_START = 0x22;  // Sut // 0xAA 0x22 0x8E 0x0D 0x0A
const uint8_t CMD_STOP = 0x24;       // Durdur // 0xAA 0x24 0x90 0x0D 0x0A

const uint8_t TELEMETRI_SIZE = 36;
const uint8_t TELEMETRI_HEADER = 0xAB;

const uint8_t HEADER = 0xAA;
const uint8_t FOOTER1 = 0x0D;
const uint8_t FOOTER2 = 0x0A;

typedef union {
  float f;
  uint8_t b[4];
} FLOAT32_UINT8_DONUSTURUCU;

// FLOAT -> BIG ENDIAN UINT ARRAY
void FtoU(float* sayi, uint8_t* hedef) {
  FLOAT32_UINT8_DONUSTURUCU d;
  d.f = *sayi;

  for (int i = 0; i < 4; i++)
    hedef[i] = d.b[3 - i];
}

// BIG ENDIAN UINT ARRAY -> FLOAT
void UtoF(uint8_t* kaynak, float* sayi) {
  FLOAT32_UINT8_DONUSTURUCU d;

  for (int i = 0; i < 4; i++)
    d.b[3 - i] = kaynak[i];

  *sayi = d.f;
}


void MAX3232_Dinle() {
  while (MAX3232Serial.available()) {
    if (roket_mod == SUT_MOD) {
      if (MAX3232Serial.peek() == TELEMETRI_HEADER) {

        uint8_t paket[TELEMETRI_SIZE];
        MAX3232Serial.readBytes(paket, TELEMETRI_SIZE);

        // Paket bitiş baytları
        if (paket[TELEMETRI_SIZE - 2] != FOOTER1 || paket[TELEMETRI_SIZE - 1] != FOOTER2) break;

        // Checksum kontrolü
        uint8_t hesaplanan = 0;
        for (int i = 0; i < TELEMETRI_SIZE - 3; i++) hesaplanan += paket[i];

        if (hesaplanan != paket[TELEMETRI_SIZE - 3]) break;

        // Float verileri dönüştür
        int offset = 1;
        UtoF(&paket[offset], &RawSensorData.RAMPA_IRTIFA);
        offset += 4;
        //  UtoF(&paket[offset], &sonuc.basinc); teknofest yanlıs veri veriyor
        offset += 4;
        UtoF(&paket[offset], &RawSensorData.AX);
        offset += 4;
        UtoF(&paket[offset], &RawSensorData.AY);
        offset += 4;
        UtoF(&paket[offset], &RawSensorData.AZ);
        offset += 4;
        //  UtoF(&paket[offset], &sonuc.aciX);
        offset += 4;
        UtoF(&paket[offset], &RawSensorData.ACI);  // teknofest dogru veri veriyor
        offset += 4;
        //  UtoF(&paket[offset], &sonuc.aciZ);
        offset += 4;
        break;
      }
    }


    uint8_t kafa = MAX3232Serial.read();  // i++ gidebilir tek kafa kullanımı olabilir
    if (kafa != HEADER)
      break;

    uint8_t buf[4];
    for (int i = 0; i < 4; i++)
      buf[i] = MAX3232Serial.read();

    uint8_t cmd = buf[0];
    uint8_t receivedCk = buf[1];
    uint8_t calcCk = (kafa + cmd) % 256;
    if (calcCk != receivedCk || buf[2] != FOOTER1 || buf[3] != FOOTER2) break;

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


void ParalelIslemOldur(TaskHandle_t* handle) {
  if (*handle == NULL) return;
  vTaskDelete(*handle);
  *handle = NULL;
}

void ParalelIslemYoneticisi() {
  switch (roket_mod) {
    case NORMAL_MOD:
      ParalelIslemOldur(&OUKBTEST_Handle);
      xTaskCreatePinnedToCore(
        Haberlesme,
        "HABERLESME",
        10000,
        NULL,
        1,
        &Haberlesme_Handle,
        0);
      break;

    case SIT_MOD:
      ParalelIslemOldur(&Haberlesme_Handle);
      xTaskCreatePinnedToCore(
        SITAlgoritma,
        "OUKBTEST_Handle",
        10000,
        NULL,
        1,
        &OUKBTEST_Handle,
        0);
      break;

    case SUT_MOD:
      ParalelIslemOldur(&Haberlesme_Handle);
      xTaskCreatePinnedToCore(
        SUTAlgoritma,
        "OUKBTEST_Handle",
        10000,
        NULL,
        1,
        &OUKBTEST_Handle,
        0);
      break;
  }
}




void SITAlgoritma(void* pvParameters) {
#ifdef TESTMODU
  Serial.print("SITAlgoritma ONCORE: ");
  Serial.println(xPortGetCoreID());
#endif

  while (true) {

    //////////////////////////////////////////////////////////////////////////////// SIT'e Özel Veriler
    float RAKIM = readAltitude(Basinc_Kalman.X, SEALEVELPRESSURE_HPA);

    float xms2 = XIvme_Kalman.X * GRAVITY;
    float yms2 = YIvme_Kalman.X * GRAVITY;
    float zms2 = ZIvme_Kalman.X * GRAVITY;

    float yAngle = atan2(YIvme_Kalman.X, -ZIvme_Kalman.X) * 180 / PI;                                                           // roll
    float xAngle = atan2(-XIvme_Kalman.X, sqrt(YIvme_Kalman.X * YIvme_Kalman.X + ZIvme_Kalman.X * ZIvme_Kalman.X)) * 180 / PI;  // pitch
    float zAngle = 0;                                                                                                           // yaw degistirme dusuncem yok teknofest
    //////////////////////////////////////////////////////////////////////////////// SIT'e Özel Veriler


    uint8_t packet[TELEMETRI_SIZE];
    int i = 0;

    // Header
    packet[i++] = TELEMETRI_HEADER;


    // Veriler
    FtoU(&RAKIM, &packet[i]);
    i += 4;
    FtoU(&Basinc_Kalman.X, &packet[i]);
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
    i += 4;

    int sum = 0;
    for (int j = 0; j < TELEMETRI_HEADER - 2; j++) sum += packet[j];

    // Checksum
    packet[i++] = sum % 256;

    // Footer
    packet[i++] = FOOTER1;
    packet[i++] = FOOTER2;

    // Gönder
    MAX3232Serial.write(packet, sizeof(packet));
    vTaskDelay(UKBTEST_INTERVAL / portTICK_PERIOD_MS);
  }
}


void SUTAlgoritma(void* pvParameters) {
#ifdef TESTMODU
  Serial.print("SUTAlgoritma ONCORE: ");
  Serial.println(xPortGetCoreID());
#endif


  delay(1000);  // MAX3232 Sentetik Veri Dinlemesinden Emin Ol

  ana_parasut = false;
  //RawSensorData.RAMPA_IRTIFA = 0;
  //RawSensorData.ACI = 0;
  Irtifa_Kalman.reset();
  Aci_Kalman.reset();

  while (true) {
    uint8_t packet[6];
    packet[0] = HEADER;

    uint8_t sonuc =
      (ana_parasut << 7) |   // Bit 7 – Ana paraşüt açma emri (ALGORITMA GLOBAL KOSUL)
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
    for (int i = 0; i < 3; i++) sum += packet[i];

    packet[3] = sum % 256;
    packet[4] = FOOTER1;
    packet[5] = FOOTER2;
    MAX3232Serial.write(packet, sizeof(packet));

    vTaskDelay(UKBTEST_INTERVAL / portTICK_PERIOD_MS);
  }
}






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

} LoraPaket;


// GPS
float enlem, boylam, gps_irtifa;


void Haberlesme(void* pvParameters) {

#ifdef TESTMODU
  Serial.print("Haberlesme ONCORE:    ");
  Serial.println(xPortGetCoreID());
#endif

  while (true) {
    unsigned long new_millis = millis();

    if (new_millis - gps_millis >= GPS_INTERVAL) {
      gps_millis = new_millis;

      // GPS
      while (ComboSerial.available())
        GPS.encode(ComboSerial.read());

      if (GPS.location.isValid()) {
        enlem = GPS.location.lat();
        boylam = GPS.location.lng();
      }

      if (GPS.altitude.isValid()) {
        gps_irtifa = GPS.altitude.meters();
      }
    }

    float GyroX = IMU.readFloatGyroZ();  // DEGISTIRILDI
    float GyroY = IMU.readFloatGyroY();  // HYI
    float GyroZ = IMU.readFloatGyroX();  // DEGISTIRILDI



    LoraPaket paket;

    // Header
    paket.header = 0xAB;

    // GPS
    paket.enlem = enlem;
    paket.boylam = boylam;
    paket.gps_irtifa = gps_irtifa;

    // BMP
    paket.basinc = Basinc_Kalman.X;
    paket.basinc_irtifa = Irtifa_Kalman.X;

    // IMU //

    // ACC
    paket.ivmex = XIvme_Kalman.X;
    paket.ivmey = YIvme_Kalman.X;
    paket.ivmez = ZIvme_Kalman.X;

    // GYRO
    paket.gyrox = GyroX;
    paket.gyroy = GyroY;
    paket.gyroz = GyroZ;

    paket.aci = Aci_Kalman.X;

    paket.parasut_durum = ana_parasut ? 2 : 1;

    uint8_t* gonderilecekPaket = (uint8_t*)&paket;

    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(LoraPaket) - 2; ++i)
      sum += gonderilecekPaket[i];

    paket.checksum = sum % 256;

    paket.footer = 0x56;

    ComboSerial.write(gonderilecekPaket, sizeof(paket));
    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }
}
