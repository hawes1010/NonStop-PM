// I2C
//pin D18 - SDA for barometric pressure, temperature, and pumps
//pin D19 - SCL for barometric pressure, temperature, and pumps

#include <Wire.h>               // IC2 library

String inByte = "";   
int i = 0;
String P1Speed;
String P1Press;
char P1Char;
String P1Byte;
int PMP1Stat = 0; 
int P1Pres = 0;
int Pmp1Speed = 0;

void setup() {
  Wire.begin();
  
  Serial.begin(9600);   // computer serial I/O port

}

void loop() {
  inByte = "";
 
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    inByte += inChar;
    i = 1;   
  }
  
  if (i == 1 && inByte.startsWith(">USA")) {    // SET PUMP1 SPEED
      char P1Array[5];
      inByte = inByte.substring(4);
      //P1Speed = 'S' + inByte.toInt();
      P1Speed = "S" + inByte;
      P1Speed.toCharArray(P1Array,5);
      Wire.beginTransmission(8); // transmit to i2c device #8 pump1
      Wire.write(P1Array);        // sends five bytes
      Wire.endTransmission(8);    // stop transmitting
      //XbeeSerial.println(">ASA1");  // send ack
      //PMP1Cont = 1;
      //PMP1Trig = 0;
      i = 0;
       inByte = "";    
    } 
         
     else if (i == 1 && inByte.startsWith(">UFA")) {    // SET PUMP1 PRESSURE
      char PM1Array[6];
      inByte = inByte.substring(4);
      P1Press = "P" + inByte;
      P1Press.toCharArray(PM1Array,6);
      Wire.beginTransmission(8); // transmit to i2c device #8 pump1
      Wire.write(PM1Array);        // sends five bytes
      Wire.endTransmission(8);    // stop transmitting
      //XbeeSerial.println(">AFA1");  // send ack
      //PMP1Cont = 0;
      i = 0;
       inByte = "";    
    }
    
   else if (i == 1){
    i = 0;
    inByte = "";
    //XbeeSerial.println(">A0");  // send ack error
   }          
 delay(50);

   Wire.requestFrom(8,12);    // request 12 bytes from pump1
    while (Wire.available() > 0){   // slave may send less than requested      
       P1Char = Wire.read();
       //Serial.println(p);
       P1Byte = P1Byte + P1Char;         
    }            
     //Serial.print(P1Byte);
      int P1commaIndex = P1Byte.indexOf(','); 
      int P1secondCommaIndex = P1Byte.indexOf(',', P1commaIndex+1);
      String P1firstValue = P1Byte.substring(0, P1commaIndex);
      String P1secondValue = P1Byte.substring(P1commaIndex+1, P1secondCommaIndex);
      String P1thirdValue = P1Byte.substring(P1secondCommaIndex+1);
           
      PMP1Stat = P1firstValue.toInt();
      P1Pres = P1secondValue.toInt();
      Pmp1Speed = P1thirdValue.toInt();
      Serial.print(P1Byte);
      P1Byte="";
      
  delay(950);
            
}
