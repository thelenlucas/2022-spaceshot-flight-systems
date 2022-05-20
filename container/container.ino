#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LM92.h>
#include <MS5611.h>
#include <math.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //GNSS Library
#include <string>

#define latchPin 6

//Data for keeping track of various times
int timeSinceLastTransmission = 0;
int transmissionInterval = 1000;
int deltaTime;
int lastExecutionTime = 0;

//Transmission Variables
int packetsTransmitted = 0;
bool transmitting = false;

//Current Flight State
String flight_state = "PS_STANDBY";

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
  Serial5.println(transmit);
}

Servo latchServo;

void closeLatch() {
  latchServo.write(0);
}

void openLatch() {
  latchServo.write(0);
}

void setup(){
  Serial.begin(9600);
  Serial2.begin(9600); //Container to groundstation
  Serial5.begin(9600); //Container to payload
  Serial4.begin(9600); //GPS UART

  //LED setup
  pinMode(ledPin, OUTPUT);

  //Servo setup
  latchServo.attach(latchPin);
  closeLatch();
  latchServo.write(0);

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
  if (MS5611.begin(&Wire2) == true)
  {
  } else {
    Cansat_Raise_Issue("MS ERROR");
  }

  lm92.ResultInCelsius = true;

  //Use scientific units for our temperature

  //Restore packets
  packetsTransmitted = restorePackets();
  
  delay(1000);
  
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
      transmitPacket("boop");
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

void reset_payload();

//CMD,1091,CX,ON
void processCommand(String com) {
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
  }


  transmitPacket(com.substring(12,14));
}

void drop_payload() {}
void land_container() {}

void loop() {
  //Methods for determining time passage
  deltaTime = millis() - lastExecutionTime;
  lastExecutionTime = millis();

  //Time since last transmission
  timeSinceLastTransmission += deltaTime;

  //Time since last blink
  timeSinceLastBlink += deltaTime;
  
  if (offsetNeeded) {
    float testPressure = MS5611.getPressure();
    if (testPressure > 0) {
      offsetNeeded = false;
      altitudeOffset = MBA_To_Altitude_Meters(testPressure);
    }
  }
  
  sensors_event_t event;
  sensors_event_t linearAccelData;
  
  bno.getEvent(&event);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  //How we read orientation
  float x_orientation = event.orientation.x;
  float y_orientation = event.orientation.y;
  float z_orientation = event.orientation.z;

  float x_accel = linearAccelData.acceleration.x;
  float y_accel = linearAccelData.acceleration.y;
  float z_accel = linearAccelData.acceleration.z;

  //Parse data shit
  getDataFromPC();
  getDataFromPC_wired();
  //Reading pressure
  //MS5611.read();
  float pressure = MS5611.getPressure();
  float realAltitude = MBA_To_Altitude_Meters(pressure) - altitudeOffset;
  
  float vertVel = (realAltitude - lastAltitude) / (deltaTime*1000);
  lastAltitude = realAltitude;
  //Serial.println(realAltitude);

  //How we read temperature
  double temperature = MS5611.getTemperature();

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
        long longitude = myGNSS.getLongitude();
        long latitude = myGNSS.getLatitude();
  
        float gpsLat = latitude / 10000000;
        float gpsLong = longitude / 10000000;
        toTransmit = "1091," + String("timePlaceholder") + "," + String(packetsTransmitted) + "," + cMode + "," + tp_released + "," + String(realAltitude) + "," + String(temperature) + "," + "3.3v" + "," + "GPS Time" + "," + String(gpsLat);
        toTransmit = toTransmit + "," + String(gpsLong) + "," + String(myGNSS.getAltitude()) + "," + String(myGNSS.getSIV()) + "," + String("LS_CMD");
        
        transmitPacket(toTransmit);
        packetsTransmitted++;
        timeSinceLastTransmission = 0;
        backupPackets(packetsTransmitted);

        openLatch();
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
    //Transition missing - Taken care of via command
  } else if (flight_state == "FS_ASCENT") {
    if (vertVel < -1) {
      flight_state = "FS_DESCENT";
    }
  } else if (flight_state == "FS_DESCENT") {
    if (realAltitude < 400) {
      drop_payload();
      flight_state = "FS_LANDING";
    }
  } else if (flight_state == "FS_LANDING") {
    if (realAltitude < 10) {
      flight_state = "FS_LANDED";
      land_container();
    }
  }

  //Delay for clarity
  delay(1);
}
