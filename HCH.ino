
// I2C Slave code based on Arduino I2C Wire Slave version 0.21 by Racer993 <https://raspberrypi4dummies.wordpress.com/&gt;

// HCH Arduino Sensors
// Wayne Bekker

#include <Wire.h>
#include <math.h>

//For Digital reads
float fBuffer = 0;
float fRawData = 0;

//For status
int iheartbeatPin = 9; //cpwm
int iheartbeatVal = 0; //starting brightness
boolean iheartbeatDirection = 1; //up or down

boolean bdebugMode = 1;

//Digital inputs
int iSHDemandPin = 4;
int iGSRDemandPin = 5;
int iGSRFlowPin = 6;
int iLowWaterDetectPin = 7;

// Analog pins
// Water Heater - A0
// Outdoor Thermistor - A1
// Indoor Thermistor - A2
// Flame Lux - A3
// A4 & A5 reserved for I2C
  
// Variables - Temperature
int ipinWH = A0;
int ipinID = A1;
int ipinOD = A2;
int ipinFL = A3;

// Variables - status
String sStatusCode = "000";
float ftempWH = 0;
float ftempID = 0; 
float ftempOD = 0; 
float fluxFL = 0; 
boolean bBoilerOn = 0;
boolean bLockout = 0;
boolean bLowWater = 0;
int iWHAlarm1 = 85;
boolean bAlarm1 = 0;
int iWHAlarm2 = 97; // should be lower but set here for pot testing
boolean bAlarm2 = 0;
int iWHTargetHigh = 80;
int iWHTargetLow = 60;
boolean bSHDemand = 0;
boolean bGSRDemand = 0;
boolean bGSRWaterFlow = 0;
boolean bFrostProt = 0;
int iFrostValue = 15; // should be lower but set here for pot testing
int iFrostLockoutValue = 2;
boolean bFuelValve = 0;
boolean bFuelPump = 0;
boolean bIgniter = 0;
boolean bSpacePump = 0;
boolean bGsrPump = 0;

//I2C signalling / control
// 
/* csendStatus Placeholders
 * Digital Pin status 0-13 (14)
 * A0 Reading 14-17 (4)
 * A1 Reading 18-21 (4)
 * A2 Reading 22-25 (4)
 * A3 Reading 26-29 (4)
 * Status 30-32 (3) sStatusCode
 * WH Temp 33-36 (4) ftempWH
 * ID Temp 37-40 (4) ftempID
 * OD Temp 41-44 (4) ftempOD
 * Flame Lux 45-49 (5) fluxFL
 * Boiler On 50 (1) bBoilerOn
 * Lockout 51 (1) bLockout
 * WH Low water 52 (1) bLowWater
 * WH Alarm 1 Temp Setting 53-54 (2) iWHAlarm1
 * WH Alarm 1 55 (1) bAlarm1
 * WH Alarm 2 Temp Setting 56-57 (2) iWHAlarm2
 * WH Alarm 2 58 (1) bAlarm2
 * WH Target High Temp Setting 59-60 (2) iWHTargetHigh
 * WH Target Low Temp Setting 61-62 (2) iWHTargetLow
 * Space Heat Demand 63 (1) bSHDemand
 * Geyser Heat Demand 64 (1) bGSRDemand
 * Geyser flow switch 65 (1) bGSRWaterFlow // how do we work with the pwm / 20 min cycle if demand stays on?
 * Frost Protection 66 (1) bFrostProt
 * Fuel Valve 0/1 67 (1) bFuelValve
 * Fuel Pump 0/1 68 (1) bFuelPump
 * Igniter 0/1 69 (1) bIgniter
 * Space Heat Pump 0/1 70 (1) bSpacePump
 * Geyser Circ Pump 0/1 71 (1) bGsrPump
 */
int istatusLength = 72; 
char csendStatus[73] = "000000000000000000000000000000000000000000000000000000000000000000000000"; // initialize the container variable
int iindex = 0; // initialize the iindex variable
char cpwm[15] = "00000000000000"; // initialize the cpwm flag container


//——————————————————————————–——————————————————————————–
// Setup
void setup(void) {
  int iarduinoI2CAddress = 0x04; // set the slave address for the Arduino on the I2C bus
  
  Wire.begin(iarduinoI2CAddress); // join i2c bus with specified address
  Wire.onRequest(requestEvent); // register wire.request interrupt event
  Wire.onReceive(receiveEvent); // register wire.write interrupt event

  pinMode(iSHDemandPin, INPUT);
  pinMode(iGSRDemandPin, INPUT);
  pinMode(iGSRFlowPin, INPUT);
  pinMode(iLowWaterDetectPin, INPUT);
  pinMode(iheartbeatPin, OUTPUT);
  digitalWrite(iheartbeatPin, LOW); // turn it off
  
  Serial.begin(9600);  

  //wait a bit and then show that we are good to go
  delay(2000);
  blinkReady(5);
  Serial.println("Ready!");
}
 
//——————————————————————————–——————————————————————————–
// Main loop
void loop(void) {

  getReadings(); // get data
  testFailures(); // test temp, water level, frost protection and?? to cause a shutdown / lockout

  /*
   * Check for lockout
   * If we are in lockout, we cannot do anything
   */
  if(!bLockout){
    //Do stuff here until lockout is reset  
    doActions(); //make hot water on demand : we need to code restrictions for 20 min cycles?
  }
  //the code below if outside the lockout as a failsafe
  testBoilerOff(); // check if we need to turn off the Boiler    

  // Gather status info for I2C
  compileStatus();
  
  if(bdebugMode == 1){
    displayDebugStuff();
    Serial.println(csendStatus);
  }  
  analogHeartbeat(); //control heartbeat led  
  delay(20); // chill for a bit. a really short bit.
}


//——————————————————————————–
// function that reads sensors
void getReadings(){  
  ftempWH = GetWaterTemp(ipinWH);
  if(ftempWH < 0){ ftempWH = 0;} // do not go negative. it breaks stuff
  ftempID = GetAirTemp(ipinID);
  ftempOD = GetAirTemp(ipinOD);  
  fluxFL = GetFlameLux(ipinFL); 
  if(fluxFL>99999){ fluxFL = 99999;}

  // Read and invert low water sensor - on while there is water
  bLowWater = digitalRead(iLowWaterDetectPin); bLowWater = !bLowWater;

  //Hot water demands?
  bSHDemand = digitalRead(iSHDemandPin); // Space heating demand 
  bGSRDemand = digitalRead(iGSRDemandPin); // Geyser heating demand
  bGSRWaterFlow = digitalRead(iGSRFlowPin); // Geyser water flow demand
}


//——————————————————————————–
// function that tests for lockout conditions
void testFailures(){
  // Temperature
  // Alarm 1
  if(ftempWH >= iWHAlarm1){
    bAlarm1 = 1;
    sStatusCode = "020";
    //Serial.println("Alarm 1 on");
  }
  // Alarm 1 auto reset
  if(bAlarm1 && ftempWH < iWHAlarm1){
    bAlarm1 = 0;
    //Serial.println("Alarm 1 off");
  }  
  // Alarm 2
  if(ftempWH >= iWHAlarm2){
    bAlarm2 = 1;
    sStatusCode = "022";
    Serial.println("Alarm 2 on");
  }
  //Over Temp Lockout
  if(ftempWH >= iWHAlarm2){
    bLockout = 1;
    sStatusCode = "022";
    Serial.println("Lockout");
  }

  // Low water
  if(bLowWater){    
    bLockout = 1;
    sStatusCode = "024";
  }
  
  //frost lockout
  if(ftempWH <= iFrostLockoutValue){
    bFrostProt = 1;
    sStatusCode = "035";
    bLockout = 1;
  }

  //reset frost lockout if the temp rises but we still need warming
  if(ftempWH <= iFrostValue && ftempWH > iFrostLockoutValue && bFrostProt){
    sStatusCode = "030";
    bLockout = 0;
  }
  
  //reset frost protection
  if(ftempWH > iFrostValue && bFrostProt){
    bFrostProt = 0;
    sStatusCode = "000";
    bLockout = 0;
  }  
}


//——————————————————————————–
// function that runs the WH when we need it to
// we are going to run these only if the water temp is below the min target?
void doActions(){    
  boolean bMakeItWarm = 0;
  if(!bLockout){ // idk why but check again 
    if(ftempWH <= iFrostValue){ //frost protection
      bFrostProt = 1;
      bMakeItWarm = 1;
      sStatusCode = "030";
    }

    //this is a bit unclear. what to do if there is a heating demand
    if(ftempWH < iWHTargetLow){ //is the current water temp below desired temp and we need hot water?
      }  
      if(bSHDemand || bGSRDemand || bGSRWaterFlow || bFrostProt){
        bMakeItWarm = 1;
      }
    }  

    //is the current water temp below desired temp and we are within a cycle time (fuel valve)
    // we need to code cycle times
    if(ftempWH <= iWHTargetLow){
      bMakeItWarm = 1;
    }
    
    //now we turn it on if we have to
    if(bMakeItWarm){
      doBoilerOn(); // we need hot water!
  }
}


//——————————————————————————–
// function that checks if we should pause / turn off the boiler
void testBoilerOff(){
  // we need to figure out pause / off stuffs
  boolean bTurnOff = 0;

  // Make sure we are not in lockout. if so turn it off.
  // this is a failsafe
  if(bLockout){
    bTurnOff = 1;
  }

  // Are we above the max desired temp? turn off or do we pause?
  if(ftempWH >= iWHTargetHigh){
    bTurnOff = 1;
  }
  
  if(bTurnOff){
    doBoilerOff();
  }
}


//——————————————————————————–
// function that turns the Boiler on
void doBoilerOn(){
  // yeah i got nothing. we need to code this.
  // if we are allow to run with the fuel switch cycle
  // if we are in an off state, run the startup sequence. check for a fail = lockout

  // if we are in a limbo state just send fuel & light the flame. check for a fail = lockout

  bBoilerOn = 1;
  if(bdebugMode){ Serial.println("Boiler On!"); }
}


//——————————————————————————–
// function that turns the Boiler off into a limbo state if we need
void doBoilerPause(){
  // yeah i got nothing. we need to code this.

  bBoilerOn = 0;
  if(bdebugMode){ Serial.println("Boiler Pause!"); }
}


//——————————————————————————–
// function that turns the Boiler off whe we hit the max desired temp
void doBoilerOff(){
  // yeah i got nothing. we need to code this.
  // shutdown sequence etc

  bBoilerOn = 0;
  if(bdebugMode){ Serial.println("Boiler Off!"); }
}


//——————————————————————————–
// function that compiles a status string for I2C transmission
void compileStatus(){
  String pinStatus=""; // initialize pinStatus variable  
  for(int digitalPin = 0; digitalPin <= 13; digitalPin++) // loop through 14 digital pins 0 – 13
  {
    if (cpwm[digitalPin] == 0) // in case cpwm is off for the pin, read the pin status
    {
      pinStatus += String (digitalRead(digitalPin)); // read the pin status & add it to the container variable
    }
    else
    {
      pinStatus += "P"; // in case cpwm is on for the pin, add P to the pin status container string
    }
  }  
  for(int analogPin = 0; analogPin <= 3; analogPin++) // loop through the 4 analog pins 0 – 3
  {
    pinStatus += String (1000+analogRead(analogPin)); // read the analog value from the pin, add 1000 to make it 4 digit & add it to the container variable
  }  
  //add local status stuff
  pinStatus += sStatusCode;
  pinStatus += paddedValue(ftempWH, 4);
  pinStatus += paddedValue(ftempID, 4);
  pinStatus += paddedValue(ftempOD, 4);
  pinStatus += paddedValue(fluxFL, 5);
  pinStatus += String(bBoilerOn);
  pinStatus += String(bLockout);
  pinStatus += String(bLowWater);
  pinStatus += paddedValue(iWHAlarm1, 2);  
  pinStatus += String(bAlarm1);
  pinStatus += paddedValue(iWHAlarm2, 2);
  pinStatus += String(bAlarm2);
  pinStatus += paddedValue(iWHTargetHigh, 2);
  pinStatus += paddedValue(iWHTargetLow, 2);  
  pinStatus += String(bSHDemand);
  pinStatus += String(bGSRDemand);
  pinStatus += String(bGSRWaterFlow);
  pinStatus += String(bFrostProt);
  pinStatus += String(bFuelValve);
  pinStatus += String(bFuelPump);
  pinStatus += String(bIgniter);
  pinStatus += String(bSpacePump);
  pinStatus += String(bGsrPump);    
  
  //publish the status for I2C
  int StatusLen = pinStatus.length() + 1;
  pinStatus.toCharArray(csendStatus, StatusLen); // convert the container variable pinStatus to a char array which can be send over i2c
}

//——————————————————————————–
// function that pulses  led to show we are alive
void analogHeartbeat(){
  // by usig cpwm here we are not adding unnecessary delays :)
  // i may need to look at optimising variables / memory here
  // maybe use odd numbers to show up and evens for down?
  analogWrite(iheartbeatPin, iheartbeatVal);
  if(iheartbeatDirection == 1){    
    iheartbeatVal += 25;
    if(iheartbeatVal >= 205){iheartbeatVal = 205; iheartbeatDirection = 0;}
  }else{
    iheartbeatVal += -25;
    if(iheartbeatVal <= 0){iheartbeatVal = 0; iheartbeatDirection = 1;}
  }  
}

//——————————————————————————–
// function that blinks the hearbeat led
void blinkReady(int _count){
  //do a blinky thing to show we are ready
  for (int i=0; i <= _count; i++){
    digitalWrite(iheartbeatPin,1);
    delay(50);
    digitalWrite(iheartbeatPin,0);
    delay(50);
  }  
}


//——————————————————————————–
// function that outputs to serial debug info
void displayDebugStuff(){  
  Serial.print("WH : "); 
  Serial.print(ftempWH);
  Serial.print(" *C - ");
  Serial.print("ID : "); 
  Serial.print(ftempID);
  Serial.print(" *C - "); 
  Serial.print("OD : "); 
  Serial.print(ftempOD);
  Serial.print(" *C - ");
  Serial.print("Lux : "); 
  Serial.print(fluxFL);
  Serial.println(" lux");


  Serial.print("bSHDemand : "); 
  Serial.println(bSHDemand);
  Serial.print("bGSRDemand : "); 
  Serial.println(bGSRDemand);
  Serial.print("bGSRWaterFlow : "); 
  Serial.println(bGSRWaterFlow);
}


//——————————————————————————–
// function that left pads a string with 0
String paddedValue(long _value, int _length){
  String _tmpStr = String(_value);
  while(_tmpStr.length() < _length){
    _tmpStr = "0" + _tmpStr;
  }
  return _tmpStr;
}


//——————————————————————————–
// function that samples a digital pin and averages readings
float analogReadAvg(int PIN, int NUMSAMPLES, int DELAY){
  float average;
  uint16_t samples[NUMSAMPLES];
  uint8_t i;
  
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(PIN);
   delay(DELAY);
  } 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  return average;
}


//——————————————————————————–
// function that calculates analog pin voltage
float getVout(int PIN){
  fRawData = analogReadAvg(PIN, 5, 10);
  fBuffer = fRawData * 5; // 5 volt input
  return (fBuffer)/1024.0;  
}


//——————————————————————————–
// function that calculates the resistance of a sensor connected to GND
float getR2(int PIN, float R1){
  float Vout = getVout(PIN);
  fBuffer = (5/Vout) -1; //5v input
  return Vout*R1/(5-Vout);
}


//——————————————————————————–
// function that reads a thermistor for the air temp
float GetAirTemp(int PIN){ 
  float AnalogSample = analogReadAvg(PIN, 5, 10);
    
  // convert the value to resistance
  AnalogSample = 1023 / AnalogSample - 1;
  AnalogSample = 10000 / AnalogSample; // average = SERIESRESISTOR / average;
 
  float steinhart;
  steinhart = AnalogSample / 3900;   // (R/Ro) - steinhart = average / THERMISTORNOMINAL;
  steinhart = log(steinhart);   // ln(R/Ro)
  steinhart /= 3950;    // 1/B * ln(R/Ro) - steinhart /= BCOEFFICIENT;
  steinhart += 1.0 / (25 + 273.15); // + (1/To) - steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  //round to 1 decimal
  steinhart = round(steinhart * 10);  
  steinhart = steinhart / 10;
  return steinhart;
}

//——————————————————————————–
// function that gets water temp from a sensor
float GetWaterTemp(int PIN){
  //Range -10c to 120c = 130 intervals
  //While using the pot we are faking it based on resistance
  //Pot1 = 50k
  float R2 = getR2(PIN, 47000); //47k other resistor
  float WaterTemp = (R2 / 50000);
  WaterTemp = (WaterTemp * 130) - 10;
  //round to 1 decimal
  WaterTemp = round(WaterTemp * 10);  
  WaterTemp = WaterTemp / 10;
  return WaterTemp;
}

//——————————————————————————–
// function that gets light value from an ldr
float GetFlameLux(int PIN){
  float R2val = getR2(PIN, 10000);
  double LuxReading = (1.25 * pow(10,7)) * (pow(R2val, -1.4059)); //   (R2val^(-1 * 1.4059))
  //round - no decimals
  LuxReading = round(LuxReading); 
  return LuxReading;
}



//——————————————————————————–
// function that executes whenever a status update is requested by master
void requestEvent() {
  Wire.write(csendStatus[iindex]);
  ++iindex;
  if (iindex >= istatusLength) {
    iindex = 0;
  }
}

//——————————————————————————–
// function that executes whenever a message is received from master
void receiveEvent(int howMany)
{
  int receiveByte = 0; // set index to 0
  char command[7]; // expect 7 char + 1 end byte
  String mode = ""; // initialize mode variable for holding the mode
  String pin = ""; // initialize pin variable for holding the pin number as a String
  String awValue = ""; // initialize the variable for holding the analogWrite value
  int pinVal; // initialize the variable for holding the pin number as integer
  int awValueVal; // initialize the variable for holding the analog write value as integer (only cpwm pins!)
  
  while(Wire.available()) // loop through all incoming bytes
  {
    command[receiveByte] = Wire.read(); // receive byte as a character
    receiveByte++; // increase iindex by 1
  }
  

  if (String(command[0]) == "C" ) { // We have a command via i2c not relating to pin actions
    int cmdaction = (String(command[1]) + String(command[2]) + String(command[3])).toInt(); // combine byte 2, 3 and 4 in order to get the command
    /*
     * Command ranges 
     * 000 - 899 - normal operation
     * 900 - 999 - test override commands
     */

     /*
      * Normal Operation Commands
      * 010 Reset lockout
      * 
      * Lockout Status Map
      * 020 Alarm 1
      * 022 Alarm 2
      * 024 Low Water Alarm
      * 030 Frost Protection
      * 035 Frost lockout
      * 
      */
    switch (cmdaction){
      case 10: { // Reset lockout
        sStatusCode = "000";
        bLockout = 0;
        bAlarm1 = 0;
        bAlarm2 = 0;
        bLowWater = 0;        
        break;
      }
            
     /*
      * Test Operation Commands
      * 
      * 
      */

      
      default: { // some unknown command sent, ignore it.
        break;
      }
    }
  }
  else{ // We have a command via i2c relating to pin actions - do whats needed
    if(bdebugMode == 1){
      //For testing output the command received
      Serial.print("Master command received : ");
      Serial.print(command[0]);
      Serial.print(command[1]);
      Serial.print(command[2]);
      Serial.print(command[3]);
      Serial.print(command[4]);
      Serial.print(command[5]);
      Serial.println(command[6]);
    }    
    pin = String(command[1]) + String(command[2]); // combine byte 2 and 3 in order to get the pin number
    awValue = String(command[4]) + String(command[5]) + String(command[6]); // combine byte 5, 6 and 7 in order to get the analogWrite value
    awValueVal = awValue.toInt(); // convert the awValue string to a value
    
    if (String(command[0]) != "A" ) { pinVal = pin.toInt();} // in case of not an analog pin assignment convert into digital pin number
    if (String(command[0]) != "A" ) { cpwm[pinVal] = 0;} // in case of not an analog pin assignment set cpwm flag to 0
    
    // incase of analog pin assignment determine analog pin to be set
    if (String(command[0]) == "A" && String(command[2]) == "0") { pinVal = A0;}
    if (String(command[0]) == "A" && String(command[2]) == "1") { pinVal = A1;}
    if (String(command[0]) == "A" && String(command[2]) == "2") { pinVal = A2;}
    if (String(command[0]) == "A" && String(command[2]) == "3") { pinVal = A3;}
    
    // if requested set pinmode
    if (String(command[0]) == "S" && String(command[3]) == "I") { pinMode(pinVal, INPUT);}
    if (String(command[0]) == "S" && String(command[3]) == "O") { pinMode(pinVal, OUTPUT);}
    if (String(command[0]) == "S" && String(command[3]) == "P") { pinMode(pinVal, INPUT_PULLUP);}
    
    // if requested perform digital write
    if (String(command[0]) == "W" && String(command[3]) == "H") { digitalWrite(pinVal, HIGH);}
    if (String(command[0]) == "W" && String(command[3]) == "L") { digitalWrite(pinVal, LOW);}
    
    // if requested perform analog write
    if (String(command[0]) == "A" && pinVal == 3 || pinVal == 5 || pinVal == 6 || pinVal == 9 || pinVal == 10 || pinVal == 11 )
    {
      analogWrite(pinVal, awValueVal);
      cpwm[pinVal] = 1;
    }
  }
  
}
