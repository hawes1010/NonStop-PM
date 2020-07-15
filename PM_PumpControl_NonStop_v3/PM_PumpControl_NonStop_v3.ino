/* Version 2.0
  -fixed pid input output scaling error
  -fixed pid output range error
  -increased program loop rate thus increasing pid correction rate
  -tunid pid to p=3,i=2,d=.3 
*/


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



//int PumpCode = 1;    

// Parameter needs calibration
int MaxPower = 255;  // Max allowed power (out of 255)
int SetPre;      // Pressure corresponds to 10 l/min
int LowPreLim = 0; // Pressure corresponds to 9.5 l/min
int PMax = 0;          // powerlevel for pump
  
//Specify the links and initial PID tuning parameters, choose one
//PID myPID(&Input, &Output, &Setpoint,3,3,.2, DIRECT); // ODEN #1
PID myPID(&Input, &Output, &Setpoint,3,3,.2, DIRECT); // BALDOR #2
//PID myPID(&Input, &Output, &Setpoint,2,1.5,.2, DIRECT); // #3
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
//const int SDAPin = 18;          // SDA pin for I2C
//const int SCLPin = 19;          // SCL pin for I2C
const int PWMPin = 20;          // PWM output to motor control board

const int PreReadings = 1000;    // 1000 Average 5s pressure sensor for PID
const int OutReadings = 5;      // 5 Average 5s PID output to control pump
const int numReadings = 20;     // Average 20 pressure readings within 1s
int PumpStatus = 0;            // Pump on/off (off = 0, on = 1)
byte PumpUnload = 1;            // Unloaded = 1, fully load = 0
byte ShutDownCount = 0;         // Shut down pump slowly after command
int myindex = 0;                // the index of the current reading


RunningAverage Pre(PreReadings);// the readings from the analog input
RunningAverage Out(OutReadings);// the readings from the analog input
float average_P = 0;            // the average of pressure
float average_O = 0;            // the average of PID output
int pumpspeed = 0;

unsigned long TimeCount = 0;    // counting time step

void setup(){
  analogWriteFrequency(20, 488); // pwm frequenc. default = 488.28
  //analogWriteResolution(12);     // default = 8 (255)

  Wire.begin(8); //use for PM pump********************************************************************************
 // Wire.begin(4); // use foe VOC pump
  Wire.onRequest(requestEvent); // register event for I2C with mainboard
  Wire.onReceive(receiveEvent);
  analogReadRes(13); // Set Teensy analog resolution to 13 bits
  pinMode(PWMPin, OUTPUT);    
  pinMode(PrePin, INPUT);
  pinMode(StandbyPin, OUTPUT);
  pinMode(inPin, INPUT);
  digitalWrite(inPin,LOW);
  
  digitalWrite(StandbyPin, LOW);  // turn off pump 
  delay(50);
  Serial.begin(9600);    //open Serial port
  delay(100);
  
 

    
  //turn the PID on
  myPID.SetOutputLimits(-8191, 8191);
  myPID.SetMode(AUTOMATIC);
  //EEPROM.put(200, 1002);  // !!!ONLY USE THE VERY FIRST TIME LOADING TEENSY!!!
 // EEPROM.put(100, 5000);  // !!!ONLY USE THE VERY FIRST TIME LOADING TEENSY!!!
}

void loop(){
 // Serial.println("H");
        // read from the sensor and add into array
      Pre.addValue(analogRead(PrePin));
      // advance to the next position in the array:
        average_P = Pre.getAverage();  
  
  if (nopid == 0){
    //Serial.println("H");
   if (digitalRead(inPin) == HIGH){
     // Serial.println("InCommand");
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
  
  Input = 0.0;
  Output = 0.0;
  Setpoint = 0.0;
  myPID.Initialize();
}

void PumpSlowShutDown(){
// Slowly shutdown the pump within 1 second

  int countdown = 175;
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
  
      // advance to the next position in the array:  
      myindex = myindex + 1;                    
    
      // Update control after 1s     
        myindex = 0;         
        // Get the difference of sensor reading and set point
        Input = average_P - SetPre;
        // Calculate PID output
        myPID.Compute();
        // Obtain PID output:  
        Out.addValue(Output);
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
  //Serial.println("H");
  //Serial.println("Request");
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
