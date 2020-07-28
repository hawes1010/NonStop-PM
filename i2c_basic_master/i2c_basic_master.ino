#include <Wire.h>

void setup() {
  Wire.begin();
}
String data = "";
void loop() {
  delay(500);// put your main code here, to run repeatedly:
  //Wire.requestFrom(8,11);
  //Serial.println("req");
  
  /*while(Wire.available()) {
    data += Wire.read();
    Serial.println("Im trying");
  }
  */
  Wire.beginTransmission(8);
  Wire.write(1);  // address high byte
  Wire.endTransmission();
  Serial.println("ooo");
  delay(500);
}
