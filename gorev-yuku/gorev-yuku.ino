
#define BUZZER D0
#define NEMPIN D4
#define GpsRX D9
#define LoraTX D12

#define LORA_INTERVAL 900

#define ALICI_ADRES 32
#define ALICI_KANAL 50


#include <deneyap.h>
#include <HardwareSerial.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>


//Önİşlemci Tanımlamaları
// Class Tanımlaması
DHT dht(NEMPIN, DHT11);
Adafruit_BMP280 BMP;
LSM6DSM IMU;
HardwareSerial LoraSerial(2);
TinyGPSPlus gps;
// Class Tanımlaması


#include <lsm6dsm.h>







//Önİşlemci Tanımlamaları
TaskHandle_t AnaAlgoritma;
TaskHandle_t Communication;


//Değişken Tanımlamaları
void setup() {
  pinMode(BUZZER, OUTPUT);

  //nem kaldı
  dht.begin();

  // LoRa ve GPS birlikte başlatma
  LoraSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  // BMP başlatma
  BMP.begin(0x76);  //veya 77 denenecek

  // IMU başlatma
  IMU.begin();


  digitalWrite(BUZZER, HIGH);

  xTaskCreatePinnedToCore(
    Degiskenler,    /* Task function. */
    "AnaAlgoritma", /* name of task. */
    10000,          /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &AnaAlgoritma,  /* Task handle to keep track of created task */
    0);             /* pin task to core 0 */
  delay(100);

  xTaskCreatePinnedToCore(
    Haberlesme,      /* Task function. */
    "Communication", /* name of task. */
    10000,           /* Stack size of task */
    NULL,            /* parameter of the task */
    1,               /* priority of the task */
    &Communication,  /* Task handle to keep track of created task */
    1);              /* pin task to core 1 */

  delay(400);

  digitalWrite(BUZZER, LOW);
}




// GPS,S
float enlem, boylam, gps_irtifa;
float sicaklik, nem, basinc.

                     void
                     Degiskenler(void*pvParameters) {
  for (;;) {
    while (LoraSerial.available())
      GPS.encode(LoraSerial.read());

    if (GPS.location.isValid()) {
      enlem = GPS.location.lat();
      boylam = GPS.location.lng();
    }

    if (GPS.altitude.isValid()) {
      gps_irtifa = GPS.altitude.meters();
    }

    sicaklik = IMU.readTemperature();
    nem = dht.readHumidity();
    basinc = BMP.readPressure();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}





void Haberlesme(void* pvParameters) {
  for (;;) {
    // ASYNC

    LoraSerial.write((byte)0x00);
    LoraSerial.write(ALICI_ADRES);
    LoraSerial.write(ALICI_KANAL);

    LoraSerial.print("#BOD,");
    LoraSerial.print("B=");
    LoraSerial.print(degiskenler.boylam);
    LoraSerial.print(",");

    LoraSerial.print("E=");
    LoraSerial.print(degiskenler.enlem);
    LoraSerial.print(",");

    LoraSerial.print("GI=");
    LoraSerial.print(degiskenler.altitude);
    LoraSerial.print(",");

    LoraSerial.print("P=");
    LoraSerial.print(degiskenler.basinc);
    LoraSerial.print(",");

    LoraSerial.print("T=");
    LoraSerial.print(degiskenler.sicaklik);
    LoraSerial.print(",");

    LoraSerial.print("H=");
    LoraSerial.print(degiskenler.nem);
    LoraSerial.print(",");

    LoraSerial.println("#EOD_Gorev");

    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }
}


void loop() {}
