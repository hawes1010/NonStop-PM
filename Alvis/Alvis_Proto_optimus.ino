//#include <Adafruit_Sensor.h>

// This code is used for the Kolibri onboard Teensy 3.2
// The main purpose is to log and transfer sensor data
// Version 5: Rewritten for the Teensy 3.1 chip, BMP180 sensor, and HiVol blower
// V6_1 fixes pressure/temp sensor
// V8-1 fixes logging clock
// V8-2 adds Adafruit BME 680 sensor and data
// V8-3 adds pump state to data log
// V9-0 works with Kolibri base station V2 software and V3 PCB. Switch to TinyGPS++ library.

// ***************
// PIN Assignmets:
// ***************

// SD Card (SPI)
// pin D6 - CS
// pin D11 - MOSI (DOUT)
// pin D12 - MISO (DIN)
// pin D13 - SCK (CLK)

//Serial Ports
// GPS
// pin D10(TX2) - RX
// pin D9(RX2) - TX
// Xbee
// pin D8(TX3) - RX
// pin D7(RX3) - TX
// K30 FS CO2
// pin D1(TX1) - RX
// pin D0(RX1) - TX

//pin A1(D15) - HiVol analog pressure input
//pin A2(D16) - HiVol analog temperature input

// Gases voltage inputs
// pin A0(D14) - CO sensor
// pin A3(D17) - Gas sensor 1
// pin A6(D20) - Gas sensor 2
// pin A7(D21) - Gas sensor 3
// pin A14(DAC) - Gas sensor 4

// I2C
//pin D18 - SDA for barometric pressure, temperature, and pumps
//pin D19 - SCL for barometric pressure, temperature, and pumps

// Pumps
//D3 - digital on/off signal for pump1
//D4 - digital on/off signal for pump2
//D5 - digital on/off signal for pump3
//D2 - digital on/off signal for HiVol

// Monitor battery voltage
// pin A8(D22) - voltpin

// **********************
// End of pin assignments
// **********************

// Xbee I/O
// Xbee command codes (ground to Kolibri):
// AA = Arm Pump 1
// AB = Arm Pump 2
// AC = Arm Pump 3
// AH = Arm HiVol
// VA = pump1 trigger value update
// VB = pump2 trigger value update
// VC = pump3 trigger value update
// VH = HiVol trigger value update
// SA = set pump1 speed
// SB = set pump2 speed
// SC = set pump3 speed
// FA = set flow pressure pump1
// FB = set flow pressure pump2
// FC = set flow pressure pump3
// RA = Reset ambient pump1
// RB = Reset ambient pump2
// RC = Reset ambient pump3
// ST = Sync Time

// Xbee data codes(Kolibri to ground):
// C2 = CO2
// CO = CO
// GC = Gas1
// GD = Gas2
// GE = Gas3
// GF = Gas4
// BV = BATTERY VOLTS
// AL = ALTITUDE
// AT = Ambient Temp
// AP = PUMP1 PRESSURE
// BP = PUMP2 PRESSURE
// CP = PUMP3 PRESSURE
// HP = HiVol PRESSURE
// AS = PUMP1 STATUS
// BS = PUMP2 STATUS
// CS = PUMP3 STATUS
// HS = HiVol STATUS
// AL = Pump1 TRIGGER LEVEL
// BL = Pump2 TRIGGER LEVEL
// CL = Pump3 TRIGGER LEVEL
// HL = HiVol TRIGGER LEVEL
// MA = Pump1 Motor Speed
// MB = Pump2 Motor Speed
// MC = Pump3 Motor Speed

// Libraries ---------------------------------------------------------------------------------
#include <SD.h>                 // SD card library   
#include <Wire.h>               // IC2 library
#include <math.h>               // Math library
#include <TimeLib.h>               // Used for rtc
#include <SPI.h>                // Used for rtc
#include <Adafruit_Sensor.h>    // BP&T Sensor
//#include <Adafruit_GPS.h>       // GPS library
#include <TinyGPS++.h>
#include <Adafruit_BMP085_U.h>  // BP&T Sensor
//#include <Adafruit_BME680.h>

TinyGPSPlus gps;

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define CO2Serial Serial1
#define GPSSerial Serial2
#define XbeeSerial Serial3

//Adafruit_GPS GPS(&GPSSerial);   // GPS library
//Adafruit_BME680 bme; // I2C
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// Declare pins ----------------------------------------------------------------------------------------
const int SDPin = 6;       // CS pin for SD card
const int COinput = A0;    // CO analog input pin
const int PrePin = A1;     // HiVol Pressure sensor pin
const int TempPin = A2;    // HiVol Temperature
const int Gas1 = A3;       // Gas1 sensor pin
const int Gas2 = 59;       // Gas2 sensor pin A6 for new board
const int Gas3 = A7;       // Gas3 sensor pin
const int Gas4 = A14;      // Gas4 sensor pin
const int VoltPin = A8;    // Monitor package voltage
const int HVonoff = 2;     // Control HiVol pump on/off
const int PMP1onoff = 20;   // Control pump1 on/off set to 3 for ne board
const int PMP2onoff = 4;   // Control pump2 on/off
const int PMP3onoff = 5;   // Control pump3 on/off
const int SDAPin = 18;     // SDA pin for I2C
const int SCLPin = 19;     // SCL pin for I2C


//int HVonoff = 21;          // Control HV pump on/off

// Define variables -----------------------------------------------------------------------------------
int oldsec;
int newsec;

byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response

//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;

int PMP1Cont = 0;          // pump1 control command
int PMP2Cont = 0;          // pump2 control command
int PMP3Cont = 0;          // pump2 control command
int HVCont = 0;            // HV pump control command
int PMP1Stat = 0;          // pump1 on/off status
int PMP2Stat = 0;          // pump2 on/off status
int PMP3Stat = 0;          // pump3 on/off status
int HVStat = 0;            // HV pump on/off status
int PMP1Trig = 9999;       // pump1 CO2 trigger value (ppm)
int PMP2Trig = 9999;       // pump2 CO2 trigger value (ppm)
int PMP3Trig = 9999;       // pump3 CO2 trigger value (ppm)
int HVTrig = 9999;         // HV CO2 trigger value (ppm)
String P1Speed;            // pump1 speed setpoint
String P2Speed;            // pump2 speed setpoint
String P3Speed;            // pump3 speed setpoint
String P1Press;            // pump1 pressure setpoint
String P2Press;            // pump2 pressure setpoint
String P3Press;            // pump3 pressure setpoint
String P1APress;           // Pump1 ambient pressure
String P2APress;           // Pump2 ambient pressure
String P3APress;           // Pump3 ambient pressure
int CO2Hyst = 25;          // Hyterisis for HiVO to turn off (ppm)

//int Trigger;               // Trigger value from Base station
int pressure = 0;          // Pressure from BMP680 (hPa)
float temperature = 0.0;   // Temperature from BMP680 (C)
float humidity = 0;        // Humidity reading from BMP680 (%)
int GsOne = 0;             // Gas1 reading
int GsTwo = 0;             // Gas2 reading
int GsThree = 0;           // Gas3 reading
int GsFour = 0;            // Gas4 reading
float CO = 0;              // CO concentration (ppm)
int Volt = 0.0;            // Package voltage (Volt)
int Altitude = 0;          // Altitude (m)
int P1Pres = 0;            // Read Pump1 pressure
int P2Pres = 0;            // Read Pump2 pressure
int P3Pres = 0;            // Read Pump3 pressure
int Pmp1Speed = 0;         // Read Pump1 speed
int Pmp2Speed = 0;         // Read pump2 speed
int Pmp3Speed = 0;         // Read pump3 speed
int HVPres = 0;            // Pressure from HiVol
int nobmp = 0;             // BMP sensor error flag
int PCO = 0;               // CO variable
String P1Byte;             // Pump1 read 
char P1Char;               // Pump1 read 
String P2Byte;             // Pump2 read
char P2Char;               // Pump2 read
String P3Byte;             // Pump3 read
char P3Char;               // Pump3 read

unsigned long valCO2 = 0;          // CO2 concentration in ppm
const int CO2Width = 2;            // Length of CO2 running average (seconds)
int readings[CO2Width];            // the readings from the analog input
int total = 0;                     // the running total
int myindex = 0;                   // the myindex of the current reading
int k = 0;
int CO2Avg = 0;                    // Running average CO2 concentration (ppm)
int CO2Past = 0;                   // The last second CO2 data

String inByte = "";                // Reading Xbee commands
String payload = "";               // Array of sensor data
int PayLen = 0;
int ptemp = 0;
int lock = 0;                      // gps sat lock
int i = 0;
float HVTemp = 0.0;                // HiVol blower temperature 

//String NMEA1; //Variable for first NMEA sentence
//String NMEA2; //Variable for second NMEA sentence
char c; //to read characters coming from the GPS

boolean pumpone = true;  //!!!!!!!!!!!!!!!make true to enable pump1 !!!!!!!!
boolean pumptwo = false;  //!!!!!!!!!!!!!!!make true to enable pump2 !!!!!!
boolean pumpthree = false;  //!!!!!!!!!!!!!!!make true to enable pump3 !!!!!!

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
int sd_succ = 0;
void setup() {
  Wire.begin();
  setSyncProvider(getTeensy3Time); // Call Teensy RTC subroutine
  analogReadRes(13); // Set Teensy analog read resolution to 13 bits
  
  // Open serial communications and wait for port to open:  --------------------------------------
  Serial.begin(9600);   // computer serial I/O port
  XbeeSerial.begin(38400);  // Xbee
  GPSSerial.begin(9600);  // GPS
  CO2Serial.begin(9600);  // CO2
  
  setSyncProvider(getTeensy3Time); // Call Teensy RTC subroutine
  
  pinMode(PMP1onoff, OUTPUT);    //define pump1 digital on/off
  pinMode(PMP2onoff, OUTPUT);    //define pump2 digital on/off
  pinMode(PMP3onoff, OUTPUT);    //define pump3 digital on/off
  pinMode(HVonoff, OUTPUT);      //define HiVol digital on/off
  //pinMode(PrePin, INPUT);        // HiVol Pressure sensor
 // pinMode(TempPin, INPUT);       // HiVol Temperature sensor
 // pinMode(Gas1, INPUT);          // Gas1 sensor
  //pinMode(COinput, INPUT);       // CO analog input
  //pinMode(VoltPin, INPUT);       // Package voltage

  digitalWrite(PMP1onoff, LOW); //pump1 off
  digitalWrite(PMP2onoff, LOW); //pump2 off
  digitalWrite(PMP3onoff, LOW); //pump3 off
  digitalWrite(HVonoff, LOW);  //HiVol off
    
  SPI.begin();
  
   // GPS start ------------------------------------------------------------------------------------
   // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  //GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rat
   
  //delay(1000); 
  
  // Init pressure sensor ------------------------------------------------------------------------
  if (!bmp.begin())
  {
    Serial.println("Ooops, no BME680 detected");
    nobmp = 1;
    //while (1);
  }

 // bme.setTemperatureOversampling(BME680_OS_8X);//8
 // bme.setHumidityOversampling(BME680_OS_2X);//2
  //bme.setPressureOversampling(BME680_OS_4X);
  //bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  //bme.setGasHeater(0, 0);
  
 //init SD card ---------------------------------------------------------------------------------
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SDPin, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin()) {
    Serial.println("Card failed, or not present");
    Serial.println("Card failed, ree");
    // don't do anything more:
    return;
  }
  else{
  Serial.println("card initialized.");
  sd_succ = 1;
  }
  //end SD card init

  // initialize all the readings to zero --------------------------------------------------------
  for (int thisReading = 0; thisReading < CO2Width; thisReading++)
    readings[thisReading] = 0;

  //startrate = millis(); // start timer
  oldsec = second();
   valCO2 = 0; 
}

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
void loop() {
   Serial.print("start_loop: ");
   Serial.println(sd_succ++);
   // GPS time, coordinate, and altitude
    while(GPSSerial.available() > 0)  //Loop until you have a good NMEA sentence
     if (gps.encode(Serial2.read()))

  inByte = "";
 
  while (XbeeSerial.available() > 0) {
    char inChar = XbeeSerial.read();
    inByte += inChar;
    i = 1;   
  } 
//Serial.print(inByte);    
   if (i == 1 && inByte.startsWith(">UST")) {               //if first chars = >ST set clock
    inByte = inByte.substring(4);
    time_t t = processSyncMessage(); //call processSyncMessage subroutine & return "t"
     if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
      Serial.print("CLOCK SET TO: ");
      Serial.print(month());
      Serial.print("/");
      Serial.print(day());
      Serial.print("/");
      Serial.print(year());
      Serial.print(" ");
      Serial.print(hour());
      Serial.print(":");
      Serial.print(minute());
      Serial.print(":");
      Serial.println(second());
      Serial.println();
     }
     XbeeSerial.println(">AST1");  // send ack
     i = 0;
     inByte = "";
    }
    
    else if (i == 1 && inByte.startsWith(">URD")) {    // data request
     XbeeSerial.println(">ARD1");  // send ack
     XbeeSerial.println(payload);
     //payload = "";
     //XbeeSerial.println("ARD1");  // send ack
     i = 0;
     inByte = "";    
    }

    else if (i == 1 && inByte.startsWith(">UAA")) {    // arm pump1 control
     inByte = inByte.substring(4);
     PMP1Cont = inByte.toInt();
      if (PMP1Cont == 0){
       digitalWrite(PMP1onoff, LOW);
       PMP1Stat = 0;
      }
      
     XbeeSerial.println(">AAA1");  // send ack
     i = 0;
     inByte = "";    
    }
    
    else if (i == 1 && inByte.startsWith(">UAB")) {    // arm pump2 control
     inByte = inByte.substring(4);
     PMP2Cont = inByte.toInt();
      if (PMP2Cont == 0){
       digitalWrite(PMP2onoff, LOW);
       PMP2Stat = 0;
      }
      
     XbeeSerial.println(">AAB1");  // send ack
     i = 0;
     inByte = "";    
    }

    else if (i == 1 && inByte.startsWith(">UAC")) {    // arm pump3 control
     inByte = inByte.substring(4);
     PMP3Cont = inByte.toInt();
      if (PMP3Cont == 0){
       digitalWrite(PMP3onoff, LOW);
       PMP3Stat = 0;
      }
      
     XbeeSerial.println(">AAC1");  // send ack
     i = 0;
     inByte = "";    
    }    
    
    else if (i == 1 && inByte.startsWith(">UAH")) {    // Hivol pump control
     inByte = inByte.substring(4);
     HVCont = inByte.toInt();
      if (HVCont == 0){
       digitalWrite(HVonoff, LOW);
       HVStat = 0;
      }      
     XbeeSerial.println(">AAH1");  // send ack
     i = 0;
     inByte = "";    
    }
    
    else if (i == 1 && inByte.startsWith(">UVA")) {    // Pump1 trigger set value
     inByte = inByte.substring(4);
     PMP1Trig = inByte.toInt();     
     XbeeSerial.println(">AVA1");  // send ack
     i = 0;
     inByte = "";    
    }

    else if (i == 1 && inByte.startsWith(">UVB")) {    // Pump2 trigger set value
     inByte = inByte.substring(4);
     PMP2Trig = inByte.toInt();
     XbeeSerial.println(">AVB1");  // send ack
     i = 0;
     inByte = "";    
    }

    else if (i == 1 && inByte.startsWith(">UVC")) {    // Pump3 trigger set value
     inByte = inByte.substring(4);
     PMP3Trig = inByte.toInt();
     XbeeSerial.println(">AVC1");  // send ack
     i = 0;
     inByte = "";    
    }    
    
    else if (i == 1 && inByte.startsWith(">UVH")) {    // HV trigger set value
     inByte = inByte.substring(4);
     HVTrig = inByte.toInt();     
     XbeeSerial.println(">AVH1");  // send ack
     i = 0;
     inByte = "";    
    }    
        
    else if (i == 1 && inByte.startsWith(">USA")) {    // SET PUMP1 SPEED
      char P1Array[5];
      inByte = inByte.substring(4);
      //P1Speed = 'S' + inByte.toInt();
      P1Speed = "S" + inByte;
      P1Speed.toCharArray(P1Array,5);
      Wire.beginTransmission(8); // transmit to i2c device #8 pump1
      Wire.write(P1Array);        // sends five bytes
      Wire.endTransmission(8);    // stop transmitting
      XbeeSerial.println(">ASA1");  // send ack
      PMP1Cont = 1;
      PMP1Trig = 0;
      i = 0;
       inByte = "";    
    }      

    else if (i == 1 && inByte.startsWith(">USB")) {    // SET PUMP2 SPEED
      char P2Array[5];
      inByte = inByte.substring(4);
      P2Speed = "S" + inByte;
      P2Speed.toCharArray(P2Array,5);
      //Serial.print(VOCArray);
      Wire.beginTransmission(4); // transmit to i2c device #4 pump2
      Wire.write(P2Array);        // sends five bytes
      Wire.endTransmission(4);    // stop transmitting
      XbeeSerial.println(">ASB1");  // send ack
      PMP2Cont = 1;
      PMP2Trig=0;
      i = 0;
       inByte = "";    
    }

    else if (i == 1 && inByte.startsWith(">USC")) {    // SET PUMP3 SPEED
      char P3Array[5];
      inByte = inByte.substring(4);
      P3Speed = "S" + inByte;
      P3Speed.toCharArray(P3Array,5);
      //Serial.print(VOCArray);
      Wire.beginTransmission(2); // transmit to i2c device #2 pump3
      Wire.write(P3Array);        // sends five bytes
      Wire.endTransmission(2);    // stop transmitting
      XbeeSerial.println(">ASC1");  // send ack
      PMP3Cont = 1;
      PMP3Trig=0;
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
      XbeeSerial.println(">AFA1");  // send ack
      //PMP1Cont = 0;
      i = 0;
       inByte = "";    
    }      

    else if (i == 1 && inByte.startsWith(">UFB")) {    // SET PUMP2 PRESSURE
      char PM2Array[6];
      inByte = inByte.substring(4);
      P2Press = "P" + inByte;
      P2Press.toCharArray(PM2Array,6);
      Wire.beginTransmission(4); // transmit to i2c device #4 pump2
      Wire.write(PM2Array);        // sends five bytes
      Wire.endTransmission(4);    // stop transmitting
      XbeeSerial.println(">AFB1");  // send ack
      //PMP2Cont = 0;
      i = 0;
       inByte = "";    
    } 

    else if (i == 1 && inByte.startsWith(">UFC")) {    // SET PUMP3 PRESSURE
      char PM3Array[6];
      inByte = inByte.substring(4);
      P3Press = "P" + inByte;
      P3Press.toCharArray(PM3Array,6);
      Wire.beginTransmission(2); // transmit to i2c device #2 pump3
      Wire.write(PM3Array);        // sends five bytes
      Wire.endTransmission(2);    // stop transmitting
      XbeeSerial.println(">AFC1");  // send ack
      //PMP2Cont = 0;
      i = 0;
       inByte = "";    
    }         
    
    else if (i == 1 && inByte.startsWith(">URA")) {    // RESET AMBIENT PUMP1 PRESSURE
      char AP1Array[6];
      inByte = inByte.substring(4);
      P1APress = "P" + inByte;
      P1APress.toCharArray(AP1Array,6);
      Wire.beginTransmission(8); // transmit to i2c device #8 pm pump1
      Wire.write(AP1Array);        // sends five bytes
      Wire.endTransmission(8);    // stop transmitting
      XbeeSerial.println(">ARA1");  // send ack
      i = 0;
       inByte = "";    
    }          
 
    else if (i == 1 && inByte.startsWith(">URB")) {    // RESET AMBIENT PUMP2 PRESSURE
      char AP2Array[6];
      inByte = inByte.substring(4);
      P2APress = "P" + inByte;
      P2APress.toCharArray(AP2Array,6);
      Wire.beginTransmission(4); // transmit to i2c device #4 pump2
      Wire.write(AP2Array);        // sends five bytes
      Wire.endTransmission(4);    // stop transmitting
      XbeeSerial.println(">ARB1");  // send ack
      i = 0;
       inByte = "";    
    }

    else if (i == 1 && inByte.startsWith(">URC")) {    // RESET AMBIENT PUMP3 PRESSURE
      char AP3Array[6];
      inByte = inByte.substring(4);
      P3APress = "P" + inByte;
      P3APress.toCharArray(AP3Array,6);
      Wire.beginTransmission(2); // transmit to i2c device #2 pump3
      Wire.write(AP3Array);        // sends five bytes
      Wire.endTransmission(2);    // stop transmitting
      XbeeSerial.println(">ARC1");  // send ack
      i = 0;
       inByte = "";    
    }                          
    
   else if (i == 1){
    i = 0;
    inByte = "";
    XbeeSerial.println(">A0");  // send ack error
   }          
 
   // i = i + 1;
    delay(50);
  
//PUMP ON OFF CONTROL BEGIN**********************************************
  // Turn on pump when moving average CO2 is higher
  // than set trigger and ground control allows it to do so.
  
  // Turn on/off pump1
  if (PMP1Cont == 1) {
    if (valCO2 >= PMP1Trig) {
      digitalWrite(PMP1onoff, HIGH);
      PMP1Stat = 1;
    } 
    else if (valCO2 < PMP1Trig) {
      digitalWrite(PMP1onoff, LOW);
      PMP1Stat = 0;
    } 
  }

 //  Turn on/off pump2
    if (PMP2Cont == 1) {
     if (valCO2 >= PMP2Trig){
      digitalWrite(PMP2onoff, HIGH);
      PMP2Stat = 1; //
     }  
     else if (valCO2 < PMP2Trig){ 
      digitalWrite(PMP2onoff, LOW);
      PMP2Stat = 0;
     } 
   }

 //  Turn on/off pump3
    if (PMP3Cont == 1) {
     if (valCO2 >= PMP3Trig){
      digitalWrite(PMP3onoff, HIGH);
      PMP3Stat = 1; //
     }  
     else if (valCO2 < PMP3Trig){ 
      digitalWrite(PMP3onoff, LOW);
      PMP3Stat = 0;
     } 
   }
   
  // Turn on/off HV blower
  if (HVCont == 1) {
    //Serial.println(valCO2);
    if (valCO2 >= HVTrig) {
      digitalWrite(HVonoff, HIGH);
      HVStat = 1;  // Hi-VO blower status    
    }
    else if (valCO2 - CO2Hyst < HVTrig) {
      digitalWrite(HVonoff, LOW);      
      HVStat = 0;  // Hi-VO blower status    
    }
  }  
//PUMP ON OFF CONTROL END************************************************
  

  if (oldsec - second() != 0)     //LOGGING TIMER
  {

    // read from the sensor and add into array
       
    sendRequest(readCO2);         //CALL CO2 SUB
    valCO2 = getValue(response);
    CO = analogRead(COinput) * .0677;
    PCO = CO * 10;

    // Voltage read
    //Volt = analogRead(VoltPin)  * 0.024539;  //use for Kolibri 1      
     //Volt = analogRead(VoltPin) * 0.021975;  //use for Kolibri 2 Baldor     
     Volt = (int)round(analogRead(VoltPin) * 0.024039);  //use for HiVol 1    
     //Volt = (int)round(analogRead(VoltPin) * 0.024374);  //use for HiVol 2 Loke
     //Serial.println(Volt);
     
    // GPS Altitude & Lock
    Altitude = (gps.altitude.meters());

    if (gps.location.isValid())
    {
      lock = 1;
    }
    else
    {
     lock = 0;
    }
  
// BEGIN READING PUMPS *************************************************

  if (pumpone == true){
    Serial.print("P1Char: ");
   Wire.requestFrom(8,12);    // request 12 bytes from pump1
   delay(50);
    while (Wire.available() > 0){   // slave may send less than requested      
       P1Char = Wire.read();
       Serial.println(P1Char);
       P1Byte = P1Byte + P1Char;         
    }
    Serial.print("P1Byte: ");           
     Serial.println(P1Byte);
      int P1commaIndex = P1Byte.indexOf(','); 
      Serial.println("Not dead_01");   
      int P1secondCommaIndex = P1Byte.indexOf(',', P1commaIndex+1);
           Serial.println("Not dead_02");   
      Serial.println(P1Byte.substring(0, P1commaIndex));
      String P1firstValue = P1Byte.substring(0, P1commaIndex);
      Serial.println("Not dead_1");   
      String P1secondValue = P1Byte.substring(P1commaIndex+1, P1secondCommaIndex);
      Serial.println("Not dead_2");    
      String P1thirdValue = P1Byte.substring(P1secondCommaIndex+1);
       Serial.println("Not dead_3");    
      PMP1Stat = P1firstValue.toInt();
      P1Pres = P1secondValue.toInt();
      Pmp1Speed = P1thirdValue.toInt();
      
 //     Serial.print(P1Byte);
      P1Byte="";
        
  }
    
  if (pumptwo == true){         
   Wire.requestFrom(4,12);    // request 12 bytes from pump2
    while (Wire.available() > 0){   // slave may send less than requested      
      P2Char = Wire.read();

       P2Byte = P2Byte + P2Char;
    }            
     //Serial.print(P2Byte);
      int P2commaIndex = P2Byte.indexOf(',');
      int P2secondCommaIndex = P2Byte.indexOf(',',P2commaIndex+1);
      String P2firstValue = P2Byte.substring(0, P2commaIndex);
      String P2secondValue = P2Byte.substring(P2commaIndex+1, P2secondCommaIndex);
      String P2thirdValue = P2Byte.substring(P2secondCommaIndex+1);
      PMP2Stat = P2firstValue.toInt();
      P2Pres = P2secondValue.toInt();
      Pmp2Speed = P2thirdValue.toInt();
      P2Byte="";
  }
    Serial.println("Not dead_3");

  if (pumpthree == true){  
   Wire.requestFrom(2,12);    // request 12 bytes from pump3
    while (Wire.available() > 0){   // slave may send less than requested      
      P3Char = Wire.read();

       P3Byte = P3Byte + P3Char;
    }            

      int P3commaIndex = P3Byte.indexOf(',');
      int P3secondCommaIndex = P3Byte.indexOf(',',P3commaIndex+1);
      String P3firstValue = P3Byte.substring(0, P3commaIndex);
      String P3secondValue = P3Byte.substring(P3commaIndex+1, P3secondCommaIndex);
      String P3thirdValue = P3Byte.substring(P3secondCommaIndex+1);
      PMP3Stat = P3firstValue.toInt();
      P3Pres = P3secondValue.toInt();
      Pmp3Speed = P3thirdValue.toInt();
      P3Byte="";
  }                      
// END READING PUMPS***********************************************************************      

    // pressure in analog
    HVPres = analogRead(PrePin);
    HVPres = HVPres * 2.98;
    HVTemp = ((analogRead(TempPin) * .000615)- 1.25) / .005;    
    GsOne = analogRead(Gas1);    //Gas1 reading
    GsTwo = analogRead(Gas2);    //Gas2 reading 
    GsThree = analogRead(Gas3);  //Gas3 reading
    GsFour = analogRead(Gas4);  //Gas4 reading    

    // begin pressure and temp**************************************
    if (nobmp == 0) {
      sensors_event_t event;
      bmp.getEvent(&event);
      pressure = (event.pressure);
      bmp.getTemperature(&temperature);
      
      //bme.performReading();
      //pressure = (bme.pressure / 100.0);
      //temperature = (bme.temperature);
      //humidity = (bme.humidity);
     }
     
    //Serial.println(pressure);
     temperature = temperature + 20;
    
    // End pressure and temp****************************************    
             
    // Xbee transfer data ------------------------------------------------------------------------
    payload = "";
    payload += "CC";
    payload += valCO2;
    payload += ",";
    payload += "CO";
    payload += PCO;
    payload += ",";
    payload += "GC";
    payload += GsOne;
    payload += ",";
    payload += "GD";
    payload += GsTwo;
    payload += ",";
    payload += "GE";
    payload += GsThree;
    payload += ",";
    payload += "GF";
    payload += GsFour;                
    payload += ",";
    payload += "BV";
    payload += Volt;
    payload += ",";
    payload += "AX";
    payload += Altitude;
    payload += ",";    
    payload += "AT";
    payload += temperature;    
    payload += ",";
    payload += "AP";
    payload += P1Pres;
    payload += ",";
    payload += "BP";
    payload += P2Pres;
    payload += ",";
    payload += "CP";    
    payload += P3Pres;
    payload += ",";        
    payload += "HP";
    payload += HVPres;
    payload += ",";
    payload += "AS";
    payload += PMP1Stat;
    payload += ",";
    payload += "BS";
    payload += PMP2Stat;
    payload += ",";
    payload += "CS";
    payload += PMP3Stat;
    payload += ",";    
    payload += "HS";
    payload += HVStat;
    payload += ",";    
    payload += "AL";
    payload += PMP1Trig;
    payload += ",";
    payload += "BL";
    payload += PMP2Trig;
    payload += ",";
    payload += "CL";
    payload += PMP3Trig;
    payload += ",";    
    payload += "HL";
    payload += HVTrig;
    payload += ",";
    payload += "AM";
    payload += Pmp1Speed;
    payload += ",";
    payload += "BM";
    payload += Pmp2Speed;
    payload += ",";
    payload += "CM";
    payload += Pmp3Speed;    
    
    
   //string length calc here
    PayLen = payload.length();
    payload = PayLen + payload;
    payload = ">D" + payload;    
    //Serial.println(payload);

    oldsec = second(); // restart timer

    // Datalogging
      Serial.println("Not dead_5");
    logger();

  }
}

// begin K30 CO2 for log: **************************
void sendRequest(byte packet[])
{
  while(!Serial1.available())  //keep sending request until we start to get a response
  {
    Serial1.write(readCO2,7);
    delay(50);
  }
  
  int timeout=0;  //set a timeoute counter
  while(Serial1.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;  
    if(timeout > 10)    //if it takes to long there was probably an error
      {
        while(Serial1.available())  //flush whatever we have
          Serial1.read();         
          break;                        //exit and try again
      }
      delay(50);
  }
  
  for (int i=0; i < 7; i++)
  {
    response[i] = Serial1.read();
  }  
}

unsigned long getValue(byte packet[])
{
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];                         //low byte for value is 5th byte in the packet  
    unsigned long val = high*256 + low;          //Combine high byte and low byte with this formula to get value
    return val* valMultiplier;
}
// end K30 CO2 input ***********************************************

// logging data
void logger()
{
  Serial.println("WRITING");
  //Begin Write data to file *************************************
  // open file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
    char filename[] = "00000000.CSV"; //default file name
    getFilename(filename); //daily file name
    File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(month());
    dataFile.print("/");
    dataFile.print(day());
    dataFile.print("/");
    dataFile.print(year());
    dataFile.print(" ");
    dataFile.print(hour());
    dataFile.print(":");
    dataFile.print(minute());
    dataFile.print(":");
    dataFile.print(second());
    dataFile.print(", ");
    
    // GPS time, coordinate, and altitude
      dataFile.print(lock); dataFile.print(',');
      dataFile.print(gps.time.hour()); dataFile.print(':');
      dataFile.print(gps.time.minute()); dataFile.print(':');
      dataFile.print(gps.time.second()); dataFile.print(", ");
      dataFile.print(gps.location.lat(), 6); //dataFile.print(GPS.lat);
      dataFile.print(", ");
      dataFile.print(gps.location.lng(), 6); //dataFile.println(GPS.lon);
      dataFile.print(", ");
      dataFile.print(gps.altitude.meters());
      dataFile.print(", ");
      
    // Sensor data
    dataFile.print(CO); dataFile.print(", ");
    dataFile.print(valCO2); dataFile.print(", ");
    dataFile.print(CO2Avg); dataFile.print(", ");
    dataFile.print(GsOne); dataFile.print(", ");
    dataFile.print(GsTwo); dataFile.print(", ");
    dataFile.print(GsThree); dataFile.print(", ");
    dataFile.print(GsFour); dataFile.print(", ");                
    dataFile.print(P1Pres); dataFile.print(", ");
    dataFile.print(P2Pres); dataFile.print(", ");
    dataFile.print(P3Pres); dataFile.print(", ");    
    dataFile.print(HVPres); dataFile.print(", ");
    dataFile.print(temperature - 20); dataFile.print(", ");
    dataFile.print(humidity); dataFile.print(", ");
    dataFile.print(pressure); dataFile.print(", ");
    dataFile.print(PMP1Stat); dataFile.print(", ");
    dataFile.print(PMP2Stat); dataFile.print(", ");
    dataFile.print(PMP3Stat); dataFile.print(", ");    
    dataFile.println(HVStat);    
    dataFile.close(); //close file
    
    //Serial.print(GsOne); Serial.print(", ");
    //Serial.print(valCO2); Serial.print(", ");
    //Serial.println(GsTwo);    
    
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog file");
  }
}

//RTC subroutine
time_t getTeensy3Time() 
{
  return Teensy3Clock.get();
}
/*  code to process time messages from the serial port   */
//#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  
     pctime = inByte.toInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
    return pctime; 
  }
  
    //file date / name generator:
void getFilename(char *filename) {
    filename[0] = '2';
    filename[1] = '0';
    filename[2] = int(year()/10)%10 + '0';
    filename[3] = year()%10 + '0';
    filename[4] = month()/10 + '0';
    filename[5] = month()%10 + '0';
    filename[6] = day()/10 + '0';
    filename[7] = day()%10 + '0';
    filename[8] = '.';
    filename[9] = 'C';
    filename[10] = 'S';
    filename[11] = 'V';
}
