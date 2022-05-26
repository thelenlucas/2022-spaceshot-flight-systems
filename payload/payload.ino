#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LM92.h>
#include <MS5611.h>
#include <math.h>
#include <EEPROM.h>
#include <SparkFun_TB6612.h>

//Pin Reads
int PhaseA = 16;
int ReadOutA = 0;
int PhaseB = 17;
int ReadOutB = 0;

//Motor Instantiations
#define AIN1 21
#define AIN2 20
#define PWMA 23
#define STBY 22

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

//Data for keeping track of various times
int timeSinceLastTransmission = 0;
int transmissionInterval = 250;
int deltaTime;
int lastExecutionTime = 0;

//Transmission Variables
int packetsTransmitted = 0;
bool transmitting = true;

//Current Flight State
String flight_state = "S";

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
  for (int i = 0; i != transmit.length(); i++) {
    Serial8.print(transmit[i]);
  }
  Serial8.print("\n");
  Serial.println(transmit);
}

void setup(){
  Serial.begin(9600);
  Serial8.begin(115200);

  //Motor Setup
  pinMode(PhaseA, INPUT);
  pinMode(PhaseB, INPUT);

  //LED setup
  pinMode(ledPin, OUTPUT);

  //Test that our BNO is working
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Cansat_Raise_Issue("BNO Error");
    //while(1);
  }

  //Test that our MS is working
  if (MS5611.begin() == true)
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
const char startMarker = '{';
const char endMarker = '}';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

void getDataFromPC() {

  // receive data from PC and save it into inputBuffer
    
  if(Serial8.available() > 0) {

    char x = Serial8.read();

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

void processCommand(String com) {
  if (com == "DROP") {
    flight_state = "O";
    transmitting = true;
  }
}

//Motor Variables
int spd = 35;
int timeSinceSwitch = 0;
boolean driving = false;

float deg = 0;
int lastB = 0;

float targetAngle = 90;

void loop() {
  //Methods for determining time passage
  deltaTime = millis() - lastExecutionTime;
  lastExecutionTime = millis();

  //Time since last transmission
  timeSinceLastTransmission += deltaTime;

  //Time since last blink
  timeSinceLastBlink += deltaTime;

  //Motor Reads
  ReadOutA = digitalRead(PhaseA);
  ReadOutB = digitalRead(PhaseB);

  int dir = 1;
  if (deg < targetAngle) {
    dir = 1;
  } else {
    dir = -1;
  }
  
  //Measure Distance of motor travel
  if (lastB != ReadOutB) {
    float mOffset = 0.305;
    deg += mOffset * dir;
    lastB = ReadOutB;
  }

  spd = 200 * dir;

  if (abs(targetAngle - deg) < 5) {
    spd = 0;
    
  }

  if (driving) {
    motor1.drive(spd);
  }
  
  if (offsetNeeded) {
    float testPressure = MS5611.getPressure();
    if (testPressure > 0) {
      offsetNeeded = false;
      altitudeOffset = MBA_To_Altitude_Meters(testPressure);
    }
  }
  
  sensors_event_t event;
  sensors_event_t linearAccelData;
  sensors_event_t magData;
  
  bno.getEvent(&event);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  //How we read orientation
  float x_orientation = event.orientation.x;
  float y_orientation = event.orientation.y;
  float z_orientation = event.orientation.z;

  float x_accel = linearAccelData.acceleration.x;
  float y_accel = linearAccelData.acceleration.y;
  float z_accel = linearAccelData.acceleration.z;

  float x_mag = magData.magnetic.x;
  float y_mag = magData.magnetic.x;
  float z_mag = magData.magnetic.x;

  //Parse data shit
  getDataFromPC();
  //Reading pressure
  MS5611.read();
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
      String cMode = "P";
      if (simulationMode) {
        String cMode = "S";
      } else {
        String cMode = "F";
      }

      String tp_released = "N";
      if (payloadReleased) {
        String tp_released = "R";
      }

      float voltage = analogRead(A0) * (3.3/1023.0);
      
      toTransmit = String(realAltitude) + "," + String(temperature) + "," + String(voltage) + "," + String(x_orientation) + "," + String(y_orientation) + "," + String(z_orientation);
      toTransmit = toTransmit + "," + String(x_accel) + "," + String(y_accel) + "," + String(z_accel) + "," + String(x_mag) + "," + String(y_mag) + "," + String(z_mag) + ",0," + flight_state;
      
      if (transmitting) { 
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
  if (flight_state == "S") {
    
  } else if (flight_state == "O") {
    if (realAltitude < 10) {
      flight_state = "L";
      transmitting = false;
      blinking = true;
      digitalWrite(2, HIGH);
    }
  }
  
}
