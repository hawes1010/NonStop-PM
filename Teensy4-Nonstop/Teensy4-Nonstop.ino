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

#define FIRST_UPLOAD 0
//double Setpoint, Input, Output;
double Input = 0.0;
double Output = 0.0;
double Setpoint = 18913174.0;  // 61% of max for output control (of 31005200)
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
const int inPin = 22;           // Read command from mainboard (was 17) we need a new pin for the design @Bill
const int EOC = 21;
const int SDAPin = 18;          // SDA pin for I2C to main board
const int SCLPin = 19;          // SCL pin for I2C
const int PWMPin = 20;          // PWM output to motor control board


const int PreReadings = 1000;    // 1000 Average 5s pressure sensor for PID
const int OutReadings = 10; //orginally 5     // 5 Average 5s PID output to control pump
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
byte single_start = 0xAA;  //single measurement
byte start_avg2 = 0xAC;  //2 averaged
byte start_avg4 = 0xAD;  //4 " "
byte start_avg8 = 0xAE; //8 " "
byte start_avg16 = 0xAF; //16 " "

//I2C address and sending info
int PM_slave_addr = 0x29;  //address for Pressure sensor
boolean first_time = true;  // timing variable to help startup

//I2C 
unsigned long currentMillis = 0;    // tracks time throughout program
long previousMillis_send = 0;       // will store last time i2c data was sent to PS sensor
long previousMillis_receive = 0;    // will store last time i2c data was received by PS Sensor
long interval_read = 65;            // every 65 msec check to see if the 16 avg sensor is finished
long interval_send = 65;            // every 65 msec check to see if the 16 avg sensor is finished
boolean send_flag = false;          // make sure we don't send too many requests 
long send_ms = 0;                   // track sending time
//I2C values read from Sensors
 uint32_t pressure_read = 0;         // store pressure
 uint32_t temperature = 0;           // store temp
int status_byte = 0;                // status_byte for reading Pressure sensor

//Sensor References 
uint32_t REFERENCE =  1 << 16;  // reference adc bits
uint32_t FULL_RES = 1 << 23;   // full pressure sensor resolution (according to manual)
uint32_t D_REFERENCE = (uint32_t)(REFERENCE * 0.5);  // Half reference for sensor resolution
uint32_t RESOLUTION = 24;  // 24 bits of Full Resolution according to manual
uint32_t SENSOR_RES = 18;  // 18 bits for our sensor pressure
uint32_t TEMP_RES = 16;   // 16 bits for temperature
uint32_t FSS = 10;        // Full scale measurements go to 10
// Math Masks
uint32_t pressure_resolution_mask    = ~(((uint32_t) 1 << (RESOLUTION - SENSOR_RES)) - 1);  // Mask for pressure
uint32_t temperature_resolution_mask = ~(((uint32_t) 1 << (RESOLUTION - TEMP_RES)) - 1);    // mask for temperature
// Timing Variables
int period = 1000;  // each second we should be requested or send the output data
//int intervals = 3;  // placeholder for design decisions

double average_pres = 0;
double average_temp = 0;
double output_power = 0;
PID myPID(&average_pres, &output_power, &Setpoint,3,3,.2, DIRECT); // BALDOR #2:  
/* PID notes
 * INPUT == average_pres.  This is the thing our PID is controlling
 * OUTPUT == output_power. This is used to control our Input and is changed by the PID
 * Setpoint: Balue we want the input to maintain== 18913174.0
 * Kp, Ki, Kd: Tuning Parameters. these affect how the pid will change the output. 
 * Direction: Either DIRECT or REVERSE. determines which direction the output will move when faced with a given error. 
 * Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
 * Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
 * Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)
 */ 
 
void setup(){

  pinMode(EOC, INPUT);    // sets the digital pin 21 as input]

  Wire1.begin(); //use for PM pump**********
  Wire.begin(8); // use foe VOC pump
  Wire.onRequest(requestEvent); // register event for I2C with mainboard
  Wire.onReceive(receiveEvent);
  
  analogReadRes(13); // Set Teensy analog resolution to 13 bits
  
  pinMode(PWMPin, OUTPUT);    
  pinMode(PrePin, INPUT);
  pinMode(StandbyPin, OUTPUT);
  pinMode(inPin, INPUT);
  digitalWrite(inPin,LOW);
  //digitalWrite(0, HIGH);
 


 //Clean up Average
//Out.clear();
 TEMP.clear();
 PRES.clear();
 
  digitalWrite(StandbyPin, LOW);  // turn off pump 
  delay(50);
  Serial.begin(115200);    //open Serial port for reading from teensy
  delay(100);
  
 
  #if FIRST_UPLOAD
  EEPROM.put(200, 1002);  // !!!ONLY USE THE VERY FIRST TIME LOADING TEENSY!!!  What value was this 
  EEPROM.put(100, 18913174);  // !!!ONLY USE THE VERY FIRST TIME LOADING TEENSY!!!  Pressure?
  #endif
  //analogWriteFrequency(20, 488); // pwm frequenc. default = 488.28
  //analogWriteResolution(12);     // default = 8 (255)
}

void loop(){
//Send is currently here as a test for I2C comms
Request_PS();
read_PS(); // adds values to averages in this function
 /*
  // read from the sensor and add into array
   //Pre.addValue(analogRead(PrePin))  //Which pin is this ??
   //Out.addValue(average_output)
   */
//  // advance to the next position in the array: this happens automatically in the library code for Running Average
  average_P = PRES.getAverage();  // calculate average again for the Pressure
  //average_pres = PRES.getAverage();  // calculate average again for the Pressure
  average_temp = TEMP.getAverage();
  if (nopid == 0){
      if (/*digitalRead(inPin) == HIGH*/  true){
     // Receive command to turn on pump
       digitalWrite(StandbyPin, HIGH); 
     if (ShutDownCount == 0){ output_power= 127; }
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
  Setpoint = 18913174.0;
  // 18913174
  // 37826348
 // myPID.Initialize();
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
   // PRES.getAverage();
   
    //PID myPID(&average_pres, &output_power, &Setpoint,3,3,.2, DIRECT); // BALDOR #2
      // advance to the next position in the array:  
      myindex = myindex + 1;                    
    
      // Update control after 1s     
        myindex = 0;         
        // Get the difference of sensor reading and set point
        //Old values==Input = average_P - SetPre;
        average_temp= TEMP.getAverage();
        average_pres = PRES.getAverage();
        // Calculate PID output
        myPID.Compute();
        // Obtain PID output:  
        Out.addValue(output_power);
        // Calculate output average
        average_O = Out.getAverage();
        // Control pump power
        analogWrite(PWMPin,output_power); 
       // pumpspeed = PMax + (Output/32);   
        // Print out to the screen
        Serial.print("Output Power [PWM] (0-255): ");  // PWM 
        Serial.print(output_power);
        Serial.println(' ');
        Serial.print("Input Pressure (0-31005200): "); // 0 - 31005200
        Serial.print(average_pres);
//        Serial.print(',');
//        Serial.print(PMax);
//        Serial.print(',');
//        Serial.print(average_P);
//        Serial.print(',');
//        Serial.print(average_O/32);
//        Serial.print(',');
//        Serial.print(pumpspeed);
//        Serial.print(',');        
        Serial.println(' ');
        Serial.print("Temperature (C): ");
        Serial.println(average_temp);
       
  delay(10);
}

void requestEvent()  // function that executes whenever data is requested by main
{

  Serial.println("Request");
  char PresArray[12];
  String str;
  int new_p = shift_pressure(average_pres);
  str = String(PumpStatus) + ',' + int(new_p) + ',' + int(output_power); //1+1+4+1+3 // pressure needs to be 4 characters
  str.toCharArray(PresArray,12);
  Wire.print(PresArray);
 //Serial.println(PresArray);
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
//Serial.println(digitalRead(EOC));
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

uint8_t status0;
int status1;
int status2;
int status3;
int status4;
int status5;
int status6;
int status7;

byte ready_byte = 0;
if(send_flag){
//Serial.println("read");
//Serial.print("EOC: ");
//Serial.println(digitalRead(EOC));
if(digitalRead(EOC)==1){
 //Serial.print("Wait bit is ");
 //Serial.println(bitRead(ready_byte,5));
 Serial.print("Hex of Ready byte is  0");
 //Serial.println(ready_byte,HEX);
 Data_done = true;
}
 

if((send_flag) && (Data_done)){ /*(currentMillis - send_ms >= interval_read) && */
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
  // Serial.println("H2");
 status_byte = data[0];
 pressure2 =   data[1];  //<< 16;
 pressure1  =  data[2];  //<< 8;
 pressure0= data[3] ;
 temp2= data[4]; //<< 16;
 temp1= data[5]; //<< 8;
 temp0= data[6];
 uint32_t mp = 0;
 pressure_read = pressure2 << 16 | (pressure1 & 0xffff ) << 8 | (pressure0 & 0xff);

 mp += (uint32_t)pressure0;
 mp += (uint32_t)(pressure1 << 8);
 mp += (uint32_t)(pressure2 << 16) ;
//Serial.print("Raw Pressure 1: "); // raw value is 10000000 << 8 which is 2^15 which is 32768
//Serial.println(pressure1, BIN);
Serial.print("Raw Pressure: ");
mp &= 0x00ffffff;
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
  mp = ((1.25*FSS*((mp-D_REFERENCE)))/(REFERENCE));
 temperature = temp2 + temp1 + temp0;
// temperature = temperature & temperature_resolution_mask;
 
// temperature = ((temperature*125)/REFERENCE)-40;
 //mt &= ~0x000000ff;
 mt = ((mt*125)/FULL_RES)-5.7;
 mt = (int)round(mt);
 send_flag = false;
 //Serial.println("R");

Serial.print("Pressure: ");
/*
Serial.print(data[1]);
Serial.print(" ");
Serial.print(data[2]);
Serial.print(" ");
Serial.println(data[3]);
*/
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

Serial.print(data[4]);
Serial.print(" ");
Serial.print(data[5]);
Serial.print(" ");
Serial.println(data[6]);

//Serial.println(temperature);
Serial.println(mt);

//Serial.print("Status Byte: ");
//Serial.println(data[0], HEX);
PRES.addValue(pressure_read);
TEMP.addValue(temperature);


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

if (first_time == true){ // allow for startup
  delay(1000);
  first_time = false;
}

currentMillis = millis();
if((currentMillis - previousMillis_send > interval_send)  && (!send_flag)) {
    // save the last time sent data
     send_flag = true;
    previousMillis_send = currentMillis;   
// start transmission to address 0x29 and ask for a average of 16 samples
    Wire1.beginTransmission(41);
    Wire1.write(start_avg16);
    Wire1.endTransmission();
    
     send_flag = true;  //mark we have a send that is currently waiting to be processed

     send_ms = currentMillis;
     Serial.println("Send");
  }
  
}

int shift_pressure(double p){

  // 18913174
  // 37826348
  double new_p = p/1024;   //x1/y1 == x2/y2
  int pressure_shifted = (int)new_p*8192;
}

double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
void correct_pid(){
 double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
}
