#include <Wire.h>



void setup() {
  // put your setup code here, to run once:
 Wire.begin(8);
 Wire.onRequest(requestEvent); // register event for I2C with mainboard
 Wire.onReceive(receiveEvent);
}

void loop() {
  //Serial.println("I am alive");
  //delay(5000);
  delay(100);

}

void requestEvent()  // function that executes whenever data is requested by main
{
 
  Serial.println("H2");
  //Serial.println("Request");
  //char PresArray[] = "1,1000,127";
  int i = 0;
  //String str;
  //str = "1,1000,127";
  //str.toCharArray(PresArray,12);
  
  //for (i =0;i <10;i++)
  //Wire.print(PresArray[i]);
  Wire.print("eeeeeeee");
  
 Serial.println("MUDA!");
}



void receiveEvent() {  //set pump control, bp pressures, and motor speed
  while(Wire.available() > 0) {  // loop through all but the last
    char c = Wire.read();        // receive byte as a character
    Serial.print(c);             // print the character
  }
}
