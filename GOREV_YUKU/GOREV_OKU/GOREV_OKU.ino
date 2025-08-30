#include <SPIFFS.h>

void setup() {
  Serial.begin(921600);
  if (!SPIFFS.begin()) {
    Serial.print("Hata");
    return;
  }


  File file = SPIFFS.open("/veri.csv", "r");
  if (!file) {
    Serial.print("CSV Dosyası Bulunamadı");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
}

void loop(){}