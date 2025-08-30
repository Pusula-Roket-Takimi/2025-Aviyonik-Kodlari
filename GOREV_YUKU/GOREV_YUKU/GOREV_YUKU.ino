#define BUZZER D0
#define GpsRX D9
#define LoraTX D12

#define HZ_5 200

#define HEADER_BYTE 0xAA
#define FOOTER_BYTE 0x55


#include <deneyap.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <SPIFFS.h>


Adafruit_BMP280 BMP;
TinyGPSPlus GPS;
HardwareSerial ComboSerial(2);
File LOG;

void setup() {
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);

  // SPIFFS başlat
  if (!SPIFFS.begin(true)) {
    while (1)
      ;
  }

  // Dosyayı sürekli açık bırak
  LOG = SPIFFS.open("/veri.csv", FILE_APPEND);
  if (!LOG) {
    while (1)
      ;
  }

  // LoRa ve GPS birlikte başlatma
  ComboSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  // BMP başlatma
  BMP.begin(0x76);  //veya 77 denenecek


  xTaskCreatePinnedToCore(
    Haberlesme,
    "Haberlesme",
    10000,
    NULL,
    1,
    NULL,
    1);
  delay(100);
  xTaskCreatePinnedToCore(
    Depolama,
    "Depolama",
    10000,
    NULL,
    1,
    NULL,
    0);

  delay(400);

  digitalWrite(BUZZER, LOW);
}



const float R = 287.05;  // Havanın gaz sabiti (J/kg.K)

// Degiskenler
float enlem, boylam, irtifa;
float sicaklik, yogunluk, basinc;



typedef struct __attribute__((packed)) {
  uint8_t header;  // 1 byte

  float enlem;     // 4 byte
  float boylam;    // 4 byte
  float irtifa;    // 4 byte
  float basinc;    // 4 byte (Pa)
  float yogunluk;  // 4 byte (kg/m3)
  float sicaklik;  // 4 byte (°C)

  uint8_t checksum;  // 1 byte
  uint8_t footer;    // 1 byte

} VeriPaketi;


void Depolama(void* pvParameters) {
  for (;;) {
    if (LOG) {
      LOG.printf("%.6f,%.6f,%.2f,%.2f,%.2f,%.2f\n",
                     enlem, boylam, irtifa, basinc, yogunluk, sicaklik);
      LOG.flush();
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // 5Hz
  }
}

void Haberlesme(void* pvParameters) {
  for (;;) {
    while (ComboSerial.available())
      GPS.encode(ComboSerial.read());

    if (GPS.location.isValid()) {
      enlem = GPS.location.lat();
      boylam = GPS.location.lng();
    }
    if (GPS.altitude.isValid()) {
      irtifa = GPS.altitude.meters();
    }

    sicaklik = BMP.readTemperature();
    basinc = BMP.readPressure();
    yogunluk = basinc / (R * (sicaklik + 273.15));

    VeriPaketi paket;
    paket.header = HEADER_BYTE;
    paket.enlem = enlem;
    paket.boylam = boylam;
    paket.irtifa = irtifa;
    paket.basinc = basinc;
    paket.yogunluk = yogunluk;
    paket.sicaklik = sicaklik;

    uint8_t* gonderilecekPaket = (uint8_t*)&paket;
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(VeriPaketi) - 2; ++i)
      sum += gonderilecekPaket[i];

    paket.checksum = sum % 256;

    paket.footer = FOOTER_BYTE;

    ComboSerial.write(gonderilecekPaket, sizeof(VeriPaketi));
    vTaskDelay(HZ_5 / portTICK_PERIOD_MS);
  }
}


void loop() {}