#include <i2c_t3.h>
//#include <Wire.h>

void setup() {
  //Wire.setSDA(18);
  //Wire.setSCL(19);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms
}
String data = "";
int target = 0x08;
void loop() {
  delay(5000);// put your main code here, to run repeatedly:
  Wire.requestFrom(8,2);
  
  
  if(Wire.available()>0) {
    data = Wire.read();
    Serial.print("Im trying, see: ");
     Serial.println("data is " + data);
  }
 /* else{
    Serial.println("send");
  Wire.beginTransmission(8);
  Wire.write("b");    // address high byte
  Wire.endTransmission();
  }
 */
  
 
}
