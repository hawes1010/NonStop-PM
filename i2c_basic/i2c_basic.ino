#include <Wire.h>



void setup() {
  // put your setup code here, to run once:
 Wire.begin(8);
 Wire.onRequest(requestEvent); // register event for I2C with mainboard
 Wire.onReceive(receiveEvent);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void requestEvent()  // function that executes whenever data is requested by main
{
 
  Serial.println("H2");
  //Serial.println("Request");
  char PresArray[] = "1,1000,127";
  int i = 0;
  //String str;
  //str = "1,1000,127";
  //str.toCharArray(PresArray,12);
  for (i =0;i <13;i++)
  Wire.print(PresArray[i]);
 //  Serial.println(PresArray);
}



void receiveEvent(int howMany) {  //set pump control, bp pressures, and motor speed
  Serial.println(Wire.read()+"a");
}
