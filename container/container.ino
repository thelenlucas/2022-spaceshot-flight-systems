#define _GLIBCXX_USE_C99 1
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LM92.h>
#include "MS5611.h"
#include <math.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //GNSS Library
#include <string>
#include <iostream>
#include <stdlib.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <SparkFun_TB6612.h>

#define latchPin 6

//Motor Instantiations
#define AIN1 11
#define AIN2 10
#define PWMA 9
#define STBY 12

Motor tetherMotor = Motor(AIN1, AIN2, PWMA, 35, STBY);

//Data for keeping track of various times
int timeSinceLastTransmission = 0;
int transmissionInterval = 1000;
int payloadInterval = 200;
int timeSinceLastPayload = 0;
int deltaTime;
int lastExecutionTime = 0;

//Transmission Variables
int packetsTransmitted = 0;
bool transmitting = true;

//Current Flight State
String flight_state = "FS_STANDBY";

//Positional Data
float lastAltitude = 0;

//Beacon variables
boolean blinking = true;
int timeSinceLastBlink = 0;
int ledPin = 13;
int blinkDelay = transmissionInterval/2;
int ledOn = 1;

//BNO Instance
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

//GPS instantiation
SFE_UBLOX_GNSS myGNSS;

//Various logic variables
boolean simulationMode = false;
boolean payloadReleased = false;

MS5611 MS5611(0x77); //Air pressure/altitude sensor
float altitudeOffset; //Offsett for calibration
bool offsetNeeded = true;

LM92 lm92; //Temperature Sensor

String timeConvert(int n) {
  String o = String(n);
  if (o.length() != 2) {o = "0" + o;}
  return o;
}

void Cansat_Raise_Issue(char *issue) {
  Serial.print("Issue Raised: ");
  Serial.print(issue);
  Serial.println();
}

//Backup Variables
int packetAdress = 0;
int addOffset = 0;
int backupStateOffset = 5;

void set_clock(time_t t) {
  RTC.set(t);
  setTime(t);
}

void backupAltitudeOffset (float a) {
  EEPROM.put(packetAdress + addOffset + backupStateOffset*2, a);
}

float restoreAltitudeOffset () {
  float n;
  EEPROM.get(packetAdress + addOffset + backupStateOffset*2, n);
  if (n == -3.14) {
    offsetNeeded = true;
  }
  return n;
}

void clearAltitudeOffset () {
  EEPROM.put(packetAdress + addOffset + backupStateOffset*2, -3.14);
}

//Clock Backups
void backupTime (time_t a) {
  float n = a;
  EEPROM.put(packetAdress + addOffset + backupStateOffset*3, n);
}

time_t restoreTime () {
  float n;
  EEPROM.get(packetAdress + addOffset + backupStateOffset*3, n);
  time_t t = n;
  return t;
}

void backupState (String state) {
  int saveInt;
  if (state == "FS_STANDBY") {
    saveInt = 0;
  } else if (state == "FS_ASCENT") {
    saveInt = 1;
  } else if (state == "FS_PEAK") {
    saveInt = 2;
  } else if (state == "FS_LANDING") {
    saveInt = 3;
  } else if (state == "FS_LANDED") {
    saveInt = 4;
  }

  EEPROM.put(packetAdress + addOffset + backupStateOffset, saveInt);
}

String restoreState() {
  int i;

  EEPROM.get(packetAdress + addOffset + backupStateOffset, i);

  String out;

  if (i == 0) {
    out = "FS_STANDBY";
  } else if (i==1) {
    out = "FS_ASCENT";
  } else if (i==2) {
    out = "FS_PEAK";
  } else if (i==3) {
    out = "FS_LANDING";
  } else if (i==4) {
    out = "FS_LANDED";
  }

  return out;
}

void clearState () {
  EEPROM.put(packetAdress + addOffset + backupStateOffset, 0);
}

void backupPackets(int numberOfPackets) {
  EEPROM.put(packetAdress + addOffset, numberOfPackets);
}

int restorePackets(){
  int out = 0;
  EEPROM.get(packetAdress + addOffset, out);
  return out;
}

void clearPackets() {
  EEPROM.put(packetAdress + addOffset, 0);
}

float MBA_To_Altitude_Meters(float millbars) {
  float altitudeInFeet = (1 - pow(millbars/1013.25, 0.190284)) * 145366.45;
  float altitudeInMeters = altitudeInFeet * 0.3048;
  return altitudeInMeters;
}

void transmitPacket(String transmit) {
  Serial.println(transmit);
  Serial2.println(transmit);  
  //Serial5.println(transmit);
}

void transmitPayload(String transmit) {
  String toTransmit = "1091," + timeConvert(hour()) + ":" + timeConvert(minute()) + ":" + timeConvert(second());
  toTransmit += "," + String(packetsTransmitted) + "," + "T" + "," + transmit;
  transmitPacket(toTransmit);
  packetsTransmitted++;
}

Servo latchServo;

void closeLatch() {
}

void openLatch() {
}

void land_container() {
  transmitting = true;
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
}

void setup(){
  setSyncProvider(RTC.get);
  
  set_clock(restoreTime());
  
  Serial.begin(115200);
  Serial2.begin(115200); //Container to groundstation
  Serial5.begin(115200); //Container to payload
  Serial4.begin(9600); //GPS UART

  //LED setup
  pinMode(ledPin, OUTPUT);

  //Servo setup
  latchServo.attach(latchPin);
  latchServo.write(35);

  if(!myGNSS.begin(Serial4))
  {
    //Check for GPS
    Cansat_Raise_Issue("GNSS Error");
  } else {
    myGNSS.setUART1Output(COM_TYPE_UBX);
    myGNSS.saveConfiguration();
  }

  //Test that our MS is working
  uint8_t scl = 24;
  uint8_t sda = 25;
  
  lm92.ResultInCelsius = true;

  //Use scientific units for our temperature

  //Restore packets
  packetsTransmitted = restorePackets();
  //Restore State
  flight_state = restoreState();

  delay(1000);
  //&Wire2
  if (MS5611.begin(&Wire2) == true)
  {
  } else {
    Cansat_Raise_Issue("MS ERROR");
  }
  
  bno.setExtCrystalUse(true);
}

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

void getDataFromPC() {

  // receive data from PC and save it into inputBuffer
    
  if(Serial2.available() > 0) {

    char x = Serial2.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      processCommand(inputBuffer);
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

void getDataFromPC_wired() {

  // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

    // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      processCommand(inputBuffer);
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

String packetBuffer = "";
void getDataFromPayload() {

  // receive data from PC and save it into inputBuffer
    
  while (Serial5.available() > 0) {
    char x = Serial5.read();
    String s = String(x);
    if (s != "\n") {
      packetBuffer += s;
    } else {
      transmitPayload(packetBuffer);
      packetBuffer = "";
    }
  }
}

void triggerCamera() {
  //Trigger high for 150ms
}

int strToInt(String s) {
  char char_array[s.length() + 1];
  strcpy(char_array, s.c_str());
  int n = atoi(char_array);
  return n;
}

float pressure;
float temperature;
bool simEnabled = false;
String lastCommand = "NULL_CMD";
bool latchOverride = false;
bool tetherGoingOut = false;
bool tetherGoingIn = false;

//CMD,1091,CX,ON
void processCommand(String com) {
  if (com == "RST_PACKET") {
    lastCommand = com;
    clearPackets();
    packetsTransmitted = 0;
  } else if (com.substring(9,11) == "CX") {
    //Telemetry ON/OFF
    if (com.substring(12,14) == "ON") {
      lastCommand = "CX_ON";
      transmitting = true;
    } else if (com.substring(12, 15) == "OFF") {
      lastCommand = "CX_OFF";
      transmitting = false;
    }
    
  } else if (com.substring(9,11) == "RS") {
    reset_payload();
    lastCommand = "RS";
    
  } else if (com.substring(9, 12) == "SIM" && com.substring(9, 13) != "SIMP") {
    if (com.substring(13, com.length()) == "ACTIVATE") {
      if (simEnabled) {
        lastCommand = "SIM_ACTIVATE";
        simulationMode = true;
        pressure = 0;
        altitudeOffset = 0;
        offsetNeeded = true;
      }
    } else if (com.substring(13, com.length()) == "DISABLE") {
      lastCommand = "SIM_DISABLE";
      simulationMode = false;
      offsetNeeded = true;
      simEnabled = false;
      
    } else if (com.substring(13, com.length()) == "ENABLE") {
      lastCommand = "SIM_ENABLE";
      simEnabled = true;
    }
    
  } else if (com.substring(9, 13) == "SIMP") {
    lastCommand = "SIMP";
    String toNumber = com.substring(14, com.length());
    char char_array[toNumber.length() + 1];
    strcpy(char_array, toNumber.c_str());
    int n = atoi(char_array);

    if (simulationMode) {
      pressure = n;
      if (offsetNeeded) {
        altitudeOffset = MBA_To_Altitude_Meters(pressure);
        offsetNeeded = false;
      }
    }
    
  } else if (com.substring(9, 11) == "ST") {
    String toNumber = com.substring(12, com.length());
    lastCommand = "ST";
    char ar[toNumber.length() + 1];
    strcpy(ar, toNumber.c_str());
    int n = atoi(ar);
    time_t converted = n;
    set_clock(converted);

    setTime(strToInt(com.substring(12, 14)), strToInt(com.substring(15, 17)),strToInt(com.substring(18, 20)),10,6,2022);
    
  } else if (com == "PRESSURE") {
    altitudeOffset = MBA_To_Altitude_Meters(pressure);
    
  } else if (com.substring(9, 12) == "RST") {
    lastCommand = "RST";
    reset_payload();
    
  } else if (com.substring(9, 12) == "DRP") {
    lastCommand = "DRP";
    //drop_payload();
    flight_state = "FS_LANDING";
    Serial5.println("{DROP}");
    backupState(flight_state);
    drop_payload();
  } else if (com.substring(9, 11) == "PR") {
    Serial5.println("{" + com.substring(12, com.length()) + "}");
  } else if (com.substring(9, 11) == "LT") {
    if (latchOverride) {latchOverride = false;} else {latchOverride = true;}
  } else if (com.substring(9, 11) == "PT") {
    Serial5.println("{L}");
  } else if (com.substring(9, 11) == "PO") {
    tetherGoingOut = true;
  } else if (com.substring(9, 11) == "PI") {
    tetherGoingIn = true;
  }
}

bool dropNeeded = true;
int timeSpentOut = 0;
int timeSpentIn = 0;

void drop_payload() {
  if (dropNeeded) {
    Serial5.println("{DROP}");
    dropNeeded = false;
    
  }
  payloadReleased = true;
  //tetherMotor.drive(50);
  
}

void reset_payload(){
  clearPackets();
  packetsTransmitted = restorePackets();

  clearState();
  flight_state = restoreState();

  clearAltitudeOffset();
  offsetNeeded = true;
};

void loop() {
  //Methods for determining time passage
  deltaTime = millis() - lastExecutionTime;
  lastExecutionTime = millis();

  //Time since last transmission
  timeSinceLastTransmission += deltaTime;

  //Time since last blink
  timeSinceLastBlink += deltaTime;

  sensors_event_t event;
  sensors_event_t linearAccelData;

  MS5611.read();
  if (!simulationMode) {
    pressure = MS5611.getPressure();
  }

  temperature = MS5611.getTemperature();
  
  if (offsetNeeded && !simulationMode) {
    float testPressure = MS5611.getPressure();
    if (testPressure > 0) {
      offsetNeeded = false;
      altitudeOffset = MBA_To_Altitude_Meters(testPressure);
    }
  }

  //Parse data shit
  getDataFromPC();
  getDataFromPC_wired();
  getDataFromPayload();

  
  float realAltitude = MBA_To_Altitude_Meters(pressure) - altitudeOffset;
  
  float vertVel = (realAltitude - lastAltitude) / (deltaTime*1000);
  lastAltitude = realAltitude;

  int millisTime1;
  int millisTime;
  
  //Should we transmit?
  if (timeSinceLastTransmission > transmissionInterval) {
   
      String toTransmit;
      String cMode = "C";
      if (simulationMode) {
        String cMode = "S";
      } else {
        String cMode = "F";
      }

      String tp_released = "N";
      if (payloadReleased) {
        String tp_released = "R";
      }

      if (transmitting) { 
        long longitude;
        long latitude;
        long alti;
        String gpsTime;
        String gpsHour;
        String gpsMinute;
        String gpsSecond;
        int sats;

        millisTime1 = millis();
        if (true) {
          longitude = myGNSS.getLongitude();
          latitude = myGNSS.getLatitude();

          gpsHour = String(myGNSS.getHour());
          gpsMinute = String(myGNSS.getMinute());
          gpsSecond = String(myGNSS.getSecond());

          sats = myGNSS.getSIV();

          alti = myGNSS.getAltitude();
        } else {
          gpsHour = "00";
          gpsMinute = "00";
          gpsSecond = "00";
        }

        millisTime = millis();

        gpsTime = gpsHour + ":" + gpsMinute + ":" + gpsSecond;

        float gpsLat = latitude / 10000000;
        float gpsLong = longitude / 10000000;
        float voltage = analogRead(A2) * (3.3/1023.0);

        toTransmit = "1091," + timeConvert(hour()) + ":" + timeConvert(minute()) + ":" + timeConvert(second());
        toTransmit += "," + String(packetsTransmitted) + "," + cMode + "," + tp_released + "," + String(realAltitude) + "," + String(temperature) + "," + String(voltage) + ",";
        toTransmit = toTransmit + gpsTime + "," + String(gpsLat) +  "," + String(gpsLong) + "," + String(alti) + "," + String(sats) + "," + flight_state + "," + lastCommand;
        
        transmitPacket(toTransmit);
        
        packetsTransmitted++;
        timeSinceLastTransmission = 0;
        backupPackets(packetsTransmitted);
        backupTime(now());
      }
  }

  timeSinceLastPayload += deltaTime;
  if (timeSinceLastPayload > payloadInterval) {
    timeSinceLastPayload = 0;
    if (flight_state == "FS_LANDING") {
      Serial5.println("{T}");
      Serial.println("Transmitted");
    }
  }

  //Should we blink?
  if (timeSinceLastBlink > blinkDelay) {
    if (blinking) {
      ledOn = 1 - ledOn;
      if (ledOn == 1) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
      timeSinceLastBlink = 0;
    }
  }

  //State Machine
  if (flight_state == "FS_STANDBY") {
    latchServo.write(140);
    if (realAltitude > 100) {
      flight_state = "FS_ASCENT";
      
    }
  } else if (flight_state == "FS_ASCENT") {
    if (realAltitude > 650) {
      flight_state = "FS_PEAK";
      latchServo.write(140);
      triggerCamera();
    }
  } else if (flight_state == "FS_PEAK") {
    if (realAltitude < 400) {
      flight_state = "FS_LANDING";
      latchServo.write(140);
    }
  } else if (flight_state == "FS_LANDING") {
    if (realAltitude < 350) {
      latchServo.write(175);
      if (dropNeeded) {
        
        drop_payload();
        dropNeeded = false;
      }
    }
    if (realAltitude < 10) {
      triggerCamera();
      flight_state = "FS_LANDED";
      land_container();
    }
  }

  if (latchOverride == true) {
    latchServo.write(175);
  } else {
    latchServo.write(140);
  }

  if (tetherGoingOut == true) {
    tetherMotor.drive(25);
    timeSpentOut += deltaTime;
    if (timeSpentOut >= 2000) {
      tetherMotor.drive(0);
      tetherGoingOut = false;
    }
  } else if (tetherGoingIn == true) {
    tetherMotor.drive(25);
    timeSpentIn += deltaTime;
    if (timeSpentIn >= 1750) {
      tetherMotor.drive(0);
      tetherGoingIn = false;
    }
  }

  backupState(flight_state);
  //Delay for clarity, maybe?
}
