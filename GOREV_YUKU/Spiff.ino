#include <FS.h>
#include <SPIFFS.h>
bool kontrol = false;
void setup(){
Serial.begin(115200);
if(!SPIFFS.begin()){
  Serial.print("Hata");
 return;
}

}
void OKUMA(){
  File file = SPIFFS.open("/veri.csv","r");
  if(!file){
    Serial.print("CSV Dosyası Bulunamadı");
    return;
  }
  while(file.available()){
    Serial.write(file.read());

  }
}
void loop(){
  if(!kontrol){
    OKUMA();
    kontrol = true;
    return;
  }
}