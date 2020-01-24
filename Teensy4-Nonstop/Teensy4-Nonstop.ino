/* Version 2.0
  -fixed pid input output scaling error
  -fixed pid output range error
  -increased program loop rate thus increasing pid correction rate
  -tunid pid to p=3,i=2,d=.3 

  Version 2.1 
  -added I2C functions for the DLHR Sensor

  Version 2.2 Beta 
  -fixing PID that rises upwards infinitely
*/

#include <math.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Wire.h>
#include "RunningAverage.h"

//double Setpoint, Input, Output;
double Input = 0.0;
double Output = 0.0;
double Setpoint = 0.0;
String inByte = "";
int i=0;
int Bpress;
int Epress;
int nopid = 0;

//Int

//int PumpCode = 1;    

// Parameter needs calibration
int MaxPower = 255;  // Max allowed power (out of 255)
int SetPre;      // Pressure corresponds to 10 l/min
int LowPreLim = 0; // Pressure corresponds to 9.5 l/min
int PMax = 0;          // powerlevel for pump
  
//Specify the links and initial PID tuning parameters, choose one
//PID myPID(&Input, &Output, &Setpoint,3,3,.2, DIRECT); // ODEN #1
// PID myPID(&Input, &Output, &Setpoint,3,3,.2, DIRECT); // BALDOR #2
//PID myPID(&Input, &Output, &Setpoint,2,1.5,.2, DIRECT); // NORSE DEITY #3
//PID myPID(&Input, &Output, &Setpoint,2,1.5,.2, DIRECT); // LOKE #4


// motor A connected between A01 and A02
// pin A7 - read pressure sensor analog output
// pin A9 - adjust potentiometer
// pin 13 - Stanby
// pin 17 - read command from mainboard to turn on pump
// pin 18 - SDA pin for I2C communication
// pin 19 - SCL pin for I2C communication
// pin 20 - PWM output


int mspeed;
float vout;
float pressure;


// Pin assignments
const int PrePin = A7;          // Pressure sensor pin
const int AdjustPin = A9;       // Potentiometer pin
const int StandbyPin = 13;      // Motor board standby pin
const int inPin = 17;           // Read command from mainboard
const int EOC = 21;
//const int SDAPin = 18;          // SDA pin for I2C
//const int SCLPin = 19;          // SCL pin for I2C
const int PWMPin = 20;          // PWM output to motor control board


const int PreReadings = 1000;    // 1000 Average 5s pressure sensor for PID
const int OutReadings = 5;      // 5 Average 5s PID output to control pump
const int threeSAvg = 3;
const int numReadings = 20;     // Average 20 pressure readings within 1s
int PumpStatus = 0;            // Pump on/off (off = 0, on = 1)
byte PumpUnload = 1;            // Unloaded = 1, fully load = 0
byte ShutDownCount = 0;         // Shut down pump slowly after command
int myindex = 0;                // the index of the current reading

RunningAverage Pre(PreReadings);// the readings from the analog input
RunningAverage Out(OutReadings);// the readings from the analog input
RunningAverage TEMP(10);
RunningAverage PRES(1000);
float average_P = 0;            // the average of pressure
float average_O = 0;            // the average of PID output
int pumpspeed = 0;

unsigned long TimeCount = 0;    // counting time step

//I2C Commands
byte single_start = 0xAA;
byte start_avg2 = 0xAC;
byte start_avg4 = 0xAD;
byte start_avg8 = 0xAE;
byte start_avg16 = 0xAF;

//I2C address and sending info
int PM_slave_addr = 0x29;
boolean first_time = true;

//I2C 
unsigned long currentMillis = 0;
long previousMillis_send = 0;       // will store last time i2c data was sent to PS sensor
long previousMillis_receive = 0;
long interval_read = 65; 
long interval_send = 65; 
boolean send_flag = false;
long send_ms = 0;
//I2C values read from Sensors
 uint32_t pressure_read = 0;
 uint32_t temperature = 0;
int status_byte = 0;

//Sensor References 
uint32_t REFERENCE =  1 << 16;
uint32_t FULL_RES = 1 << 23;
uint32_t D_REFERENCE = (uint32_t)(REFERENCE * 0.5);
uint32_t RESOLUTION = 24;
uint32_t SENSOR_RES = 18;
uint32_t TEMP_RES = 16;
uint32_t FSS = 10;
// Math Masks
uint32_t pressure_resolution_mask    = ~(((uint32_t) 1 << (RESOLUTION - SENSOR_RES)) - 1);
uint32_t temperature_resolution_mask = ~(((uint32_t) 1 << (RESOLUTION - TEMP_RES)) - 1);
// Timing Variables
int period = 3000;
int intervals = 3;

double average_pres = 0;
double average_temp = 0;
double output_power = 0;

PID myPID(&average_pres, &output_power, &Setpoint,3,3,.2, DIRECT); // BALDOR #2
 
void setup(){
    pinMode(EOC, INPUT);    // sets the digital pin 7 as input
  analogWriteFrequency(20, 488); // pwm frequenc. default = 488.28
  analogWriteResolution(12);     // default = 8 (255)
  delay(50);
  Wire1.begin(); //use for PM pump********************************************************************************
  
 // Wire.begin(8); // use foe VOC pump
 // Wire.onRequest(requestEvent); // register event for I2C with mainboard
  //Wire.onReceive(receiveEvent);
  
  analogReadRes(13); // Set Teensy analog resolution to 13 bits
  pinMode(PWMPin, OUTPUT);    
  pinMode(PrePin, INPUT);
  pinMode(StandbyPin, OUTPUT);
  pinMode(inPin, INPUT);
  digitalWrite(inPin,LOW);


 //Clean up Average
//Out.clear();
 TEMP.clear();
 PRES.clear();
 
  //digitalWrite(StandbyPin, LOW);  // turn off pump 
  delay(50);
  Serial.begin(115200);    //open Serial port
  delay(100);
  
 

    
  //turn the PID on
  myPID.SetOutputLimits(-16384, 16384); // This doesn't work unless this somehow 
  myPID.SetMode(AUTOMATIC);
  //EEPROM.put(200, 1002);  // !!!ONLY USE THE VERY FIRST TIME LOADING TEENSY!!!
  //EEPROM.put(100, 5000);  // !!!ONLY USE THE VERY FIRST TIME LOADING TEENSY!!!
}

void loop(){
//Send is currently here as a test for I2C comms
Request_PS();
read_PS();


 
  // read from the sensor and add into array
    //Pre.addValue(analogRead(PrePin));
  // advance to the next position in the array:
  average_P = PRES.getAverage();  
  
  if (nopid == 0){
      if (digitalRead(inPin) == HIGH){
      // Receive command to turn on pump
        digitalWrite(StandbyPin, HIGH); 
     //if (ShutDownCount == 0){Output = 5000; }
        PIDMain();
        ShutDownCount = 1; // Count shutdown
     }
   else{
     // Turn off pump
     if (ShutDownCount == 1){
       PumpSlowShutDown();
       ShutDownCount = 0;
       myindex = 0;
       initialize();   // reset PID
     }
   }
  } 

}

void initialize() {
  myindex = 0;
  average_P =0;
  average_O = 0;
  pumpspeed = 0;
  
  TimeCount = 0;
  PumpStatus = 0;
  PumpUnload = 1;
         
  Pre.clear();
  Out.clear();
  TEMP.clear();
  PRES.clear();
  Input = 0.0;
  Output = 0.0;
  Setpoint = 0.0;
 // myPID.Initialize();
}


void avgPIDvalues(){

  
}
void PumpSlowShutDown(){
// Slowly shutdown the pump within 1 second

  int countdown = 50;
  while (countdown > 0){
    analogWrite(PWMPin,countdown);
    countdown = countdown-2;
    delay(20);
  }
  digitalWrite(StandbyPin, LOW); // turn off pump
}

void PIDMain(){                   // PID main function
  //Serial.println("H");
  // Read the potentiometer reading as calibration
    EEPROM.get(100,SetPre);       //get stored pump control pressure
    PumpStatus = 1;
    PRES.getAverage();
    TEMP.getAverage();
    //PID myPID(&average_pres, &output_power, &Setpoint,3,3,.2, DIRECT); // BALDOR #2
      // advance to the next position in the array:  
      myindex = myindex + 1;                    
    
      // Update control after 1s     
        myindex = 0;         
        // Get the difference of sensor reading and set point
        //Old values==Input = average_P - SetPre;
        average_pres = PRES.getAverage();
        // Calculate PID output
        myPID.Compute();
        // Obtain PID output:  
        Out.addValue(output_power);
        // Calculate output average
        average_O = Out.getAverage();
        // Control pump power
        analogWrite(PWMPin,min(Output/32,MaxPower)); 
        pumpspeed = PMax + (Output/32);   
        // Print out to the screen
        Serial.print(Output/32);
        Serial.print(',');
        Serial.print(Input);
        Serial.print(',');
        Serial.print(PMax);
        Serial.print(',');
        Serial.print(average_P);
        Serial.print(',');
        Serial.print(average_O/32);
        Serial.print(',');
        Serial.print(pumpspeed);
        Serial.print(',');        
        Serial.println(myindex);
       
  delay(10);
}

void requestEvent()  // function that executes whenever data is requested by main
{

  Serial.println("Request");
  char PresArray[12];
  String str;
  str = String(PumpStatus) + ',' + int(average_P) + ',' + int(pumpspeed);
  str.toCharArray(PresArray,12);
  Wire.print(PresArray);
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
   initialize();   // reset PID
   inByte = inByte.substring(1);
   int Epress = inByte.toInt();
   EEPROM.put(100, Epress);
   delay(50);
   SetPre = Epress;
   nopid = 0;
   i = 0;
   inByte = "";
  }
  else if (i == 1 && inByte.startsWith("B")) {               //if first chars = B set bar pressure
   inByte = inByte.substring(1);
   int Bpress = inByte.toInt();
   EEPROM.put(200, Bpress);
   delay(50);
   i = 0;
   nopid = 0;
   inByte = "";
  } 
  else if (i == 1 && inByte.startsWith("S")) {               //if first chars = S set motor speed
   inByte = inByte.substring(1);
   mspeed = inByte.toInt();
   Serial.println(mspeed);
   digitalWrite(StandbyPin, HIGH);  
   analogWrite(PWMPin,mspeed);
   pumpspeed = mspeed;   
   nopid = 1;
   i = 0;
   inByte = "";
  }     
}

void read_PS(){
  //Serial.println("read");
currentMillis = millis(); 
//array to hold bytes from PS sensor
boolean Data_done = false;
//static int count = 0;
uint32_t data[7];
uint32_t pressure0 = 0;
uint32_t pressure1 = 0;
uint32_t pressure2= 0;
uint32_t temp0= 0;
uint32_t temp1= 0;
uint32_t temp2= 0;

//uint8_t status0;
//int status1;
//int status2;
//int status3;
//int status4;
//int status5;
//int status6;
//int status7;


byte ready_byte = 0;
if(send_flag){

if(digitalRead(EOC)==1){
 //Serial.print("Wait bit is ");
 //Serial.println(bitRead(ready_byte,5));
 Serial.print("Hex of Ready byte is  0");
 //Serial.println(ready_byte,HEX);
 Data_done = true;
}

if(/*(currentMillis - send_ms >= interval_read) && */(send_flag) & (Data_done)){
  Data_done = false;
 Serial.println("Reading");
  Wire1.requestFrom(41,7);
  //Serial.println("H");
  int i = 0;
  for (i = 0; i < 7; i++){
    data[i] = Wire1.receive();
    i++;
  }
  i = 0;
   //Serial.println("H2");
 status_byte = data[0];
 pressure2 =   data[1];  //<< 16;
 pressure1  =  data[2];  //<< 8;
 pressure0= data[3] ;
 temp2= data[4]; //<< 16;
 temp1= data[5]; //<< 8;
 temp0= data[6];

 /*
uint32_t REFERENCE = (uint32_t) 1 << 24;
uint32_t D_REFERENCE = (REFERENCE/2);
uint32_t FSS = 20;
  * 
  pressure_resolution_mask    = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - pressure_resolution)) - 1);
  temperature_resolution_mask = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - TEMPERATURE_RESOLUTION)) - 1);
  * 
  */
 uint32_t mp = 0;
 pressure_read = pressure2 << 16 | (pressure1 & 0xffff ) << 8 | (pressure0 & 0xff);

 mp += (uint32_t)pressure0;
 mp += (uint32_t)(pressure1 << 8);
 mp += (uint32_t)(pressure2 << 16) ;
//Serial.print("Raw Pressure 1: "); // raw value is 10000000 << 8 which is 2^15 which is 32768
//Serial.println(pressure1, BIN);
Serial.print("Raw Pressure: ");
Serial.println(mp);

 float mt = 0;
 temp0 = 0;
 mt += (float)temp0;
 mt += (float)(temp1 << 8);
 mt += (float)(temp2 << 16) ;
 Serial.print("Raw Temp: ");
Serial.println(mt);
 uint32_t ref = pow(2,24);
//pressure_read = pressure_read & pressure_resolution_mask;
//mp = mp & pressure_resolution_mask;
// pressure_read = (1.25*FSS*((pressure_read -D_REFERENCE)))*(1/REFERENCE);
  mp &= 0x00ffffff;
  //mp = (float)12.5*((mp/REFERENCE)-.5);
 // mp = ((1.25*FSS*((mp-D_REFERENCE)))/(REFERENCE));
 temperature = temp2 + temp1 + temp0;
// temperature = temperature & temperature_resolution_mask;
 
// temperature = ((temperature*125)/REFERENCE)-40;
 //mt &= ~0x000000ff;
 mt = ((mt*125)/FULL_RES)-5.7;
 mt = (int)round(mt);
 send_flag = false;
 //Serial.println("R");

Serial.print("Pressure: ");
Serial.println(mp);
//Serial.println(pressure_read);
Serial.print("Temperature: ");
Serial.println(mt);
Input = mp;
//add PID values;
PRES.addValue(mp);
TEMP.addValue(mt);
/*
 * uint32_t pressure_read = 0;
 * uint32_t temperature = 0;
 */


for (i = 0; i < 7; i++){
    data[i] = 0;
    i++;
    }
    i = 0;
 }

    }

  }

void Request_PS(){
 //Serial.println("Request");

if (first_time == true){
 // delay(1000);
  first_time = false;
}

 // delay(1000);

currentMillis = millis();
if((currentMillis - previousMillis_send > interval_send)  && (!send_flag)) {
  Serial.println("Send");
    // save the last time sent data
    
    previousMillis_send = currentMillis;   
// start transmission to address 0x29 and ask for a average of 16 samples
    Wire1.beginTransmission(41);
    Wire1.write(start_avg16);
    Wire1.endTransmission();
     send_flag = true;
     send_ms = currentMillis;
     Serial.println("Send");
  }
  
}







//Past here thar be code that hath been laid to rest


/* 
  if (i == 1 && inByte.startsWith("P")) {     //if first chars = P set pressure
   initialize();   // reset PID
   inByte = inByte.substring(1);
   int Epress = inByte.toInt();
   EEPROM.put(100, Epress);
   delay(50);
   SetPre = Epress;
   nopid = 0;
   i = 0;
   inByte = "";
  }
  else if (i == 1 && inByte.startsWith("B")) {               //if first chars = B set bar pressure
   inByte = inByte.substring(1);
   int Bpress = inByte.toInt();
   EEPROM.put(200, Bpress);
   delay(50);
   i = 0;
   nopid = 0;
   inByte = "";
  } 
  else if (i == 1 && inByte.startsWith("S")) {               //if first chars = S set motor speed
   inByte = inByte.substring(1);
   mspeed = inByte.toInt();
   Serial.println(mspeed);
   digitalWrite(StandbyPin, HIGH);  
   analogWrite(PWMPin,mspeed);
   pumpspeed = mspeed;   
   nopid = 1;
   i = 0;
   inByte = "";
  }     
  */
