
#include <Wire.h>
// pin 18 - SDA pin for I2C communication
// pin 19 - SCL pin for I2C communication


String inByte = "";
int i=0;
int Bpress;
int Epress;
int SetPre;
int mspeed;
int pumpspeed = 0;

void setup() {
    Wire.begin(8); //use for PM pump*
    Wire.onRequest(requestEvent); // register event for I2C with mainboard
    Wire.onReceive(receiveEvent);
    Serial.begin(9600);    //open Serial port
    delay(100);

    
}

void loop() {
  // put your main code here, to run repeatedly:

}

void requestEvent()  // function that executes whenever data is requested by main
{
  //Serial.println("H");
  //Serial.println("Request");
  char PresArray[12];
  String str;
  str = '1' + ',' + '1234' + ',' + '123';
  str.toCharArray(PresArray,12);
  Wire.print(PresArray);
  Serial.println(PresArray);
  
 //  Serial.println(PresArray);
}

void receiveEvent(int howMany) {  //set pump control, bp pressures, and motor speed
  //Serial.println("H");
  while (Wire.available() > 0) {
    char c = Wire.read();
    inByte += c;
    i = 1;   
  } 
  if (i == 1 && inByte.startsWith("P")) {     //if first chars = P set pressure
   //initialize();   // reset PID
   inByte = inByte.substring(1);
   int Epress = inByte.toInt();
   Serial.println(Epress);
   //EEPROM.put(100, Epress);
   delay(50);
   SetPre = Epress;
   //nopid = 0;
   i = 0;
   inByte = "";
  }
  else if (i == 1 && inByte.startsWith("B")) {               //if first chars = B set bar pressure
   inByte = inByte.substring(1);
   int Bpress = inByte.toInt();
   Serial.println(Bpress);
   //EEPROM.put(200, Bpress);
   delay(50);
   i = 0;
   //nopid = 0;
   inByte = "";
  } 
  else if (i == 1 && inByte.startsWith("S")) {               //if first chars = S set motor speed
   inByte = inByte.substring(1);
   mspeed = inByte.toInt();
   Serial.println(mspeed);
   //digitalWrite(StandbyPin, HIGH);  
   //analogWrite(PWMPin,mspeed);
   pumpspeed = mspeed;   
   //nopid = 1;
   i = 0;
   inByte = "";
  }     
}
