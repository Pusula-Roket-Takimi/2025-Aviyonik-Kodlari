// Kalman filtresi değişkenleri
float Q = 0.001;  // Daha az sistem gürültüsü
float R = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P = 1.0;    // Başlangıç belirsizliği
float X = 0.0;    // İlk tahmin
float K = 0.0;    // Kalman kazancı


float Q1 = 0.001;  // Daha az sistem gürültüsü
float R1 = 0.1;    // Ölçüme daha çok güven (daha düşük)
float P1 = 1.0;    // Başlangıç belirsizliği
float X1 = 0.0;    // İlk tahmin
float K1 = 0.0;    // Kalman kazancı
#include <Adafruit_BMP280.h>
#include "lsm6dsm.h"
#define buzzer D4
String durum_parasut = "-";
Adafruit_BMP280 bmp; 
int c = 0;
int a = 0;
float new_irtifa;
LSM6DSM IMU;
void setup() {


  // put your setup code here, to run once:
   Serial.begin(115200);
    bmp.begin(0x76);//veya 77 denenecek
    IMU.begin();
    pinMode(buzzer,OUTPUT);
}
void loop() {
  float basinc = bmp.readPressure();
   float roketAci = abs(IMU.readFloatAccelZ()) * 100;

   P = P + Q;

  // 2. Kalman kazancı: (K = P / (P + R))
  K = P / (P + R);

  // 3. Ölçüm düzeltme: (X = X + K * (Z - X))
  X = X + K * (basinc - X);

  // 4. Hata kovaryansı güncelle: (P = (1 - K) * P)
  P = (1 - K) * P;

     P1 = P1 + Q1;

  // 2. Kalman kazancı: (K = P / (P + R))
  K1 = P1 / (P1 + R1);

  // 3. Ölçüm düzeltme: (X = X + K * (Z - X))
  X1 = X1 + K1 * (roketAci - X1);

  // 4. Hata kovaryansı güncelle: (P = (1 - K) * P)
  P1 = (1 - K1) * P1;

  // Sonuç: X (filtrelenmiş basınç)
  Serial.print("Ham: ");
  Serial.print(basinc);
  Serial.print("  Kalman: ");
  Serial.println(X);

   Serial.print("Ham Z: ");
  Serial.print(roketAci);
  Serial.print("  Kalman: ");
  Serial.println(X1);
 float irtifa = bmp.readAltitude(X);
 if(new_irtifa < irtifa && X1 > 50 ){
  c = 1;
 }
 if(c==1 && a==0){
  a=1;
  digitalWrite(buzzer,HIGH);
  Serial.println("Patladi");
  durum_parasut = "