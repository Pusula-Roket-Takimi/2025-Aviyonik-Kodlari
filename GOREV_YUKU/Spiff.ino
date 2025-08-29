#include <FS.h>
#include <SPIFFS.h>
void setup(){
Serial.begin(115200);
if(!SPIFFS.begin()){
  Serial.print("Hata");
  return;
}
File file = SPIFFS.open("/veri.csv","r");
if(!file){
  Serial.print("Bulunamadı CSV Dosyası");
  return;
}
while(file.available()){
  Serial.write(file.read());
}

}
void loop(){

}
