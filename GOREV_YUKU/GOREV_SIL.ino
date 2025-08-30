#include <SPIFFS.h>

void setup() {
  Serial.begin(921600);
  if (!SPIFFS.begin()) {
    Serial.print("Hata");
    return;
  }


  if (SPIFFS.remove("/veri.csv")) {
    Serial.print("CSV Dosyası Silindi");
  }else{
        Serial.print("CSV Dosyası Bulunamadı");

  }

}

void loop(){}