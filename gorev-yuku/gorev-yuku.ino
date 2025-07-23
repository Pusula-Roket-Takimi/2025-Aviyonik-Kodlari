/*
#define BUZZER D0
#define GpsRX D9
#define LoraTX D12
*/

#define LoraTX D9
#define GpsRX D13
#define BUZZER D4

#define LORA_INTERVAL 400

//#define ALICI_ADRES 31
//#define ALICI_KANAL 50

#define HEADER_BYTE 0xAA
#define FOOTER_BYTE 0x55


#include <deneyap.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>


Adafruit_BMP280 BMP;
TinyGPSPlus GPS;
HardwareSerial LoraSerial(2);


TaskHandle_t AnaAlgoritma;
TaskHandle_t Communication;


void setup() {
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);

  // LoRa ve GPS birlikte başlatma
  LoraSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  // BMP başlatma
  BMP.begin(0x76);  //veya 77 denenecek

  xTaskCreatePinnedToCore(
    Degiskenler,
    "AnaAlgoritma",
    10000,
    NULL,
    1,
    &AnaAlgoritma,
    0);
  delay(100);

  xTaskCreatePinnedToCore(
    Haberlesme,
    "Communication",
    10000,
    NULL,
    1,
    &Communication,
    1);

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

  uint8_t checksum;// 1 byte
  uint8_t footer;  // 1 byte

} VeriPaketi;


void Degiskenler(void* pvParameters) {
  while (1) {
    while (LoraSerial.available())
      GPS.encode(LoraSerial.read());

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

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Haberlesme(void* pvParameters) {
  for (;;) {
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

    LoraSerial.write(gonderilecekPaket, sizeof(VeriPaketi));
    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }
}


void loop() {}