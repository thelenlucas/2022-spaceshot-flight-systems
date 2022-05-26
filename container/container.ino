#define _GLIBCXX_USE_C99 1
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LM92.h>
#include <MS5611.h>
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

#define latchPin 5

//Motor Instantiations
#define AIN1 11
#define AIN2 10
#define PWMA 9
#define STBY 12

Motor tetherMotor = Motor(AIN1, AIN2, PWMA, 35, STBY);

//Data for keeping track of various times
int timeSinceLastTransmission = 0;
int transmissionInterval = 1000;
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
int blinkDelay = 500;
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

void Cansat_Raise_Issue(char *issue) {
  Serial.print("Issue Raised: ");
  Serial.print(issue);
  Serial.println();
}

//Backup Variables
int packetAdress = 0;
int addOffset = 0;
int timeAdressOffset = 100;

void backupPackets(int numberOfPackets) {
  EEPROM.put(packetAdress + addOffset, numberOfPackets);
}

void set_clock(time_t t) {
  RTC.set(t);
  setTime(t);
  backupTime(t);
}

void backupTime(time_t tTime) {
  EEPROM.put(packetAdress + addOffset + timeAdressOffset, tTime);
}

time_t restoreTime() {
  int out = 0;
  EEPROM.get(packetAdress + addOffset + timeAdressOffset, out);
  return out;
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
   Serial5.println(transmit);  
}

void transmitPayload(String transmit) {
  String toTransmit = "1091," + String(hour()) + ":" + String(minute()) + ":" + String(second());
  toTransmit += "," + String(packetsTransmitted) + "," + "P" + "," + transmit;
  transmitPacket(toTransmit);
  packetsTransmitted++;
}

Servo latchServo;

void closeLatch() {
}

void openLatch() {
}

void setup(){
  setSyncProvider(RTC.get);
  
  set_clock(1653494690);
  
  Serial.begin(115200);
  Serial2.begin(115200); //Container to groundstation
  Serial5.begin(115200); //Container to payload
  Serial4.begin(9600); //GPS UART

  //LED setup
  pinMode(ledPin, OUTPUT);

  //Servo setup
  latchServo.attach(latchPin);
  latchServo.write(35);

  //Test that our BNO is working
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Cansat_Raise_Issue("BNO Error");
    //while(1);
  }

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
  //&Wire2
  if (MS5611.begin(&Wire2) == true)
  {
  } else {
    Cansat_Raise_Issue("MS ERROR");
  }
  
  lm92.ResultInCelsius = true;

  //Use scientific units for our temperature

  //Restore packets
  packetsTransmitted = restorePackets();
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

void reset_payload(){
  clearPackets();
  restorePackets();
};

float pressure;
float temperature;

//CMD,1091,CX,ON
void processCommand(String com) {
  transmitPacket(com.substring(9, 13));
  if (com == "RST_PACKET") {
    clearPackets();
    packetsTransmitted = 0;
  } else if (com.substring(9,11) == "CX") {
    //Telemetry ON/OFF
    if (com.substring(12,14) == "ON") {
      transmitting = true;
    } else if (com.substring(12, 15) == "OFF") {
      transmitting = false;
    }
    
  } else if (com.substring(9,11) == "RS") {
    reset_payload();
    
  } else if (com.substring(9, 12) == "SIM" && com.substring(9, 13) != "SIMP") {
    if (com.substring(13, 16) == "ON") {
      simulationMode = true;
      pressure = 0;
      altitudeOffset = 0;
      offsetNeeded = true;
    } else if (com.substring(13, 17) == "OFF") {
      simulationMode = false;
      offsetNeeded = true;
    }
    
  } else if (com.substring(9, 13) == "SIMP") {
    String toNumber = com.substring(14, com.length());
    char char_array[toNumber.length() + 1];
    strcpy(char_array, toNumber.c_str());
    int n = atoi(char_array);
    transmitPacket(n);

    if (simulationMode) {
      pressure = n;
      if (offsetNeeded) {
        altitudeOffset = MBA_To_Altitude_Meters(pressure);
        offsetNeeded = false;
      }
    }
    
  } else if (com == "PRESSURE") {
    altitudeOffset = MBA_To_Altitude_Meters(pressure);
  }
}

void drop_payload() {
  Serial5.println("{DROP}");
  payloadReleased = true;
}
void land_container() {
  transmitting = true;
}

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
  //getDataFromPC_wired();
  getDataFromPayload();
  
  float realAltitude = MBA_To_Altitude_Meters(pressure) - altitudeOffset;
  
  float vertVel = (realAltitude - lastAltitude) / (deltaTime*1000);
  lastAltitude = realAltitude;

  int millisTime1;
  int millisTime;
  
  //Should we transmit?
  if (timeSinceLastTransmission > transmissionInterval) {
      
      

      MS5611.read();
      if (!simulationMode) {
        pressure = MS5611.getPressure();
      }
    
      temperature = MS5611.getTemperature();

      
      //How we read orientation
      float x_orientation = event.orientation.x;
      float y_orientation = event.orientation.y;
      float z_orientation = event.orientation.z;
    
      float x_accel = linearAccelData.acceleration.x;
      float y_accel = linearAccelData.acceleration.y;
      float z_accel = linearAccelData.acceleration.z;
      
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
        if (false) {
          longitude = myGNSS.getLongitude();
          latitude = myGNSS.getLatitude();

          gpsHour = String(myGNSS.getHour());
          gpsMinute = String(myGNSS.getMinute());
          gpsSecond = String(myGNSS.getSecond());

          sats = myGNSS.getSIV();

          alti = myGNSS.getAltitude();
        }

        millisTime = millis();

        gpsTime = gpsHour + ":" + gpsMinute + ":" + gpsSecond;

        float gpsLat = latitude / 10000000;
        float gpsLong = longitude / 10000000;
        float voltage = analogRead(A2) * (3.3/1023.0);

        toTransmit = "1091," + String(hour()) + ":" + String(minute()) + ":" + String(second());
        toTransmit += "," + String(packetsTransmitted) + "," + cMode + "," + tp_released + "," + String(realAltitude) + "," + String(temperature) + "," + String(voltage) + ",";
        toTransmit = toTransmit + gpsTime + "," + String(gpsLat) +  "," + String(gpsLong) + "," + String(alti) + "," + String(sats) + "," + flight_state + "," + String("LS_CMD");

        
        
        transmitPacket(toTransmit);
        
        packetsTransmitted++;
        timeSinceLastTransmission = 0;
        backupPackets(packetsTransmitted);
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
    if (realAltitude > 100) {
      flight_state = "FS_ASCENT";
    }
  } else if (flight_state == "FS_ASCENT") {
    if (realAltitude > 650) {
      flight_state = "FS_PEAK";
    }
  } else if (flight_state == "FS_PEAK") {
    if (realAltitude < 400) {
      flight_state = "FS_LANDING";
      latchServo.write(65);
    }
  } else if (flight_state == "FS_LANDING") {
    if (realAltitude < 350) {
      tetherMotor.drive(50);
      drop_payload();
    }
    if (realAltitude < 10) {
      flight_state = "FS_LANDED";
      land_container();
      digitalWrite(0, HIGH);
      digitalWrite(1, HIGH);
    }
  }

  Serial2.println(millisTime1 - millisTime);
  //Delay for clarity
  delay(1);
}
