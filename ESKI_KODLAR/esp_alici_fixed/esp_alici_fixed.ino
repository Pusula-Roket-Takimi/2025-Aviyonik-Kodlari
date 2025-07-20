
#include <HardwareSerial.h>
#include <deneyap.h>
#include <Arduino.h>

#define RXD2 D0 
#define TXD2 D1  

HardwareSerial mySerial(2); // UART2

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 Alıcı Başladı!");
}

void loop() {
  if (mySerial.available()) {
    String gelenMesaj = mySerial.readStringUntil('\n'); 
    Serial.println("Gelen mesaj: "+ gelenMesaj);
  }

  
  //hakem yer istasyonu eklenecek
}

