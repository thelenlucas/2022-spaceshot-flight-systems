#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LM92.h>
#include <MS5611.h>
#include <math.h>
#include <EEPROM.h>
#include <SparkFun_TB6612.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <math.h>
#define PI 3.14159265
#include <Servo.h>

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

int latchedDegree = -10;
int unlatchedDegree = 0;

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

//Data for keeping track of various times
int timeSinceLastTransmission = 0;
int transmissionInterval = 100;
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
  Serial8.print(transmit);
  Serial8.print("\n");
  Serial.println(transmit);
}

void toggleCam() {
  digitalWrite(3,LOW);
  delay(1000);
  digitalWrite(3,HIGH);
}


Servo latchServo;

void setup(){
  
  digitalWrite(1, HIGH);
  pinMode(3, OUTPUT);
  digitalWrite(3,HIGH);
  
  Serial.begin(115200);
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

  latchServo.attach(0);

  lm92.ResultInCelsius = true;

  //Use scientific units for our temperature

  //Restore packets
  packetsTransmitted = restorePackets();
  
  delay(1000);

  //Test that our MS is working
  if (MS5611.begin() == true)
  {
  } else {
    Cansat_Raise_Issue("MS ERROR");
  }
  
  bno.setExtCrystalUse(true);
}
int transmissionNeeded = 0;
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

int servoPos = 180;

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

bool latched = false;
bool driving = false;
int latchedOverride = 0;
void processCommand(String com) {
  if (com == "DROP") {
    flight_state = "O";
    transmitting = true;
    driving = true;
    latchServo.write(unlatchedDegree);
  } else if (com == "L") {
    transmissionNeeded += 1;
    flight_state = "O";
    transmitting = true;
    driving = true;
    latchServo.write(unlatchedDegree);
  } else if (com == "T") {
    if (latchedOverride) {latchedOverride = false;} else {latchedOverride = true;}
  } else if (com == "C") {
    toggleCam();
  }
  //Serial8.println(com + ";");
  Serial.println(com);
}

//Motor Variables
int spd = 225;
int timeSinceSwitch = 0;

float deg = 0;
int lastB = 0;

float targetAngle = 90;

int dir = 1;
int target = 0;

float error = 0;
bool toggled = false;
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

  if (lastB != ReadOutB) {
    //ReadOutB == 1
    if (true) {
      deg += 6 * dir;
    //Serial.println("changed");
    }
    lastB = ReadOutB;
  }
  
  if (offsetNeeded) {
    float testPressure = MS5611.getPressure();
    if (testPressure > 0) {
      offsetNeeded = false;
      altitudeOffset = MBA_To_Altitude_Meters(testPressure);
    }
  }

  int time1 = millis();
  
  sensors_event_t event;
  sensors_event_t linearAccelData;
  sensors_event_t magData;
  
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  int time2 = millis();

  //How we read orientation
  float x_orientation = event.orientation.x;
  float y_orientation = event.orientation.y;
  float z_orientation = event.orientation.z;

  float x_accel = linearAccelData.acceleration.x;
  float y_accel = linearAccelData.acceleration.y;
  float z_accel = linearAccelData.acceleration.z;

  float x_mag = magData.magnetic.x;
  float y_mag = magData.magnetic.y;
  float z_mag = magData.magnetic.z;

  //Yaw calc
  double yaw = atan2(y_mag, x_mag) * 180/3.14159;

  //Parse data shit
  getDataFromPC();
  getDataFromPC_wired();
  //Reading pressure
  MS5611.read();
  float pressure = MS5611.getPressure();
  float realAltitude = MBA_To_Altitude_Meters(pressure) - altitudeOffset;
  
  float vertVel = (realAltitude - lastAltitude) / (deltaTime*1000);
  lastAltitude = realAltitude;
  //Serial.println(realAltitude);

  //How we read temperature
  double temperature = MS5611.getTemperature();

  //Should we transmit? - timeSinceLastTransmission > transmissionInterval
  //transmissionNeeded
  while (transmissionNeeded > 0) {
      bno.getEvent(&event);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      
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

      setSyncProvider(RTC.get);
      RTC.set(1653494690);
      
      toTransmit = String(realAltitude) + "," + String(temperature) + "," + String(voltage) + "," + String(x_orientation) + "," + String(y_orientation) + "," + String(z_orientation);
      toTransmit = toTransmit + "," + String(x_accel) + "," + String(y_accel) + "," + String(z_accel) + "," + String(x_mag) + "," + String(y_mag) + "," + String(z_mag) + "," + String(error) + "," + flight_state + "";
      
      if (true) { 
        transmitPacket(toTransmit);
        packetsTransmitted++;
        timeSinceLastTransmission = 0;
        backupPackets(packetsTransmitted);
      }
      transmissionNeeded -= 1;
  }

  //Motor driving code
  target = yaw;

  error = deg-yaw;

  if (deg > 180) {
    deg -= 360;
  } else if (deg < -180) {
    deg += 360;
  }

  if (yaw > 180) {
    deg -= 360;
  } else if (deg < -180) {
    deg += 360;
  }

  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }

  if (error < -20) {
    dir = 1;
  } else if (error > 20){
    dir = -1;
  } else {
    dir = 0;
  }

  if (true) {
    //motor1.drive(spd * dir); // I think 35 is about the least power I can send the motor and have it still driver.
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
    //latchServo.write(latchedDegree);
  } else if (flight_state == "O") {
    latchServo.write(unlatchedDegree);
    driving = true;
    if (toggled == false) {
      toggleCam();
      Serial.println("ah");
      toggled = true;
    }
    if (realAltitude < 10) {
      flight_state = "L";
      toggleCam();
      
      blinking = true;
      digitalWrite(2, HIGH);
    }
  }

  if (latchedOverride) {
    latchServo.write(unlatchedDegree);
  }
}
