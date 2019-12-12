#include <Wire.h>

#include <AllSensors_DLHR.h>

AllSensors_DLHR_L05D_8 gagePressure(&Wire);
/*
class AllSensors_DLHR_L05G_8 : public AllSensors_DLHR {
public:
  AllSensors_DLHR_L05G_8(TwoWire *bus) : AllSensors_DLHR(bus, AllSensors_DLHR::SensorType::GAGE, AllSensors_DLHR::SensorResolution::RESOLUTION_18_BITS, 5.0) {}
};

*/
int EOC = 21;  
int request;
  enum MeasurementType {
    SINGLE    = 0xAA,
    AVERAGE2  = 0xAC,
    AVERAGE4  = 0xAD,
    AVERAGE8  = 0xAE,
    AVERAGE16 = 0xAF,
  };
void setup() {
  Serial.begin(115200);
  
  digitalWrite(0, HIGH);
  Wire.begin();
  //digitalRead(pin)    
  Serial.println("H");
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);
  request = 0;
}
void loop() {
    
    delay(1000);
    int x = gagePressure.readStatus();
    Serial.print("Status ");
    Serial.println(x);
    
    if (request == 0){
    gagePressure.startMeasurement();
    request = 1;
    Serial.println("Request Sent");
    }
    delay (5);
    x = gagePressure.readStatus();
   Serial.print("Error is ");
   Serial.println(gagePressure.isError(x));
   int y = x & 32 ;
   Serial.print("Status 2 is ");
   Serial.println(y);
   Serial.print("EOC is ");
   Serial.println(digitalRead(21));   
   if (y == 0 && request == 1){
    gagePressure.readData(true);
  //Serial.println(gagePressure.isBusy());
  Serial.print("Pressure: ");
  Serial.print(gagePressure.pressure);
  Serial.print(" Temperature: ");
  Serial.println(gagePressure.temperature);
  request = 0;
   }
}
