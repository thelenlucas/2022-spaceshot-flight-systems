#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LM92.h>
#include <MS5611.h>
#include <math.h>

//Data for keeping track of various times
int timeSinceLastTransmission = 0;
int transmissionInterval = 250;
int deltaTime;
int lastExecutionTime = 0;

//Transmission Variables
int packetsTransmitted = 0;
bool transmitting = true;

//Current Flight State
String flight_state = "PS_STANDBY";

//Positional Data
float lastAltitude = 0;

//BNO Instance
Adafruit_BNO055 bno = Adafruit_BNO055(28);

MS5611 MS5611(0x77); //Air pressure/altitude sensor
float altitudeOffset; //Offsett for calibration
bool offsetNeeded = true;

LM92 lm92; //Temperature Sensor

void Cansat_Raise_Issue(char *issue) {
  Serial.print("Issue Raised: ");
  Serial.print(issue);
  Serial.println();
}

float MBA_To_Altitude_Meters(float millbars) {
  float altitudeInFeet = (1 - pow(millbars/1013.25, 0.190284)) * 145366.45;
  float altitudeInMeters = altitudeInFeet * 0.3048;
  return altitudeInMeters;
}

void transmitPacket(String transmit) {
  Serial1.println(transmit);
  Serial.println(transmit);
}

void setup(){
  Serial1.begin(9600);
  Serial.begin(9600);

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

void getDataFromContainer() {

  // receive data from PC and save it into inputBuffer
    
  if(Serial1.available() > 0) {

    char x = Serial1.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      transmitPacket(inputBuffer);
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

void loop() {
  //Methods for determining time passage
  deltaTime = millis() - lastExecutionTime;
  lastExecutionTime = millis();

  //Time since last transmission
  timeSinceLastTransmission += deltaTime;
  
  if (offsetNeeded) {
    float testPressure = MS5611.getPressure();
    if (testPressure > 0) {
      offsetNeeded = false;
      altitudeOffset = MBA_To_Altitude_Meters(testPressure);
    }
  }
  
  sensors_event_t event;
  bno.getEvent(&event);

  //How we read orientation
  float x_orientation = event.orientation.x;
  float y_orientation = event.orientation.y;
  float z_orientation = event.orientation.z;

  //Parse data stuff
  getDataFromContainer();
  //Reading pressure
  MS5611.read();
  float pressure = MS5611.getPressure();
  float realAltitude = MBA_To_Altitude_Meters(pressure) - altitudeOffset;
  
  float vertVel = (realAltitude - lastAltitude) / (deltaTime*1000);
  lastAltitude = realAltitude;
  //Serial.println(realAltitude);

  //How we read temperature
  double temperature = lm92.readTemperature();

  //Should we transmit?
  if (timeSinceLastTransmission > transmissionInterval) {
      String toTransmit;
      toTransmit = "1091," + String(packetsTransmitted) + ",T," + String(realAltitude) + "," + String(temperature) + "," + "3.3v" + "," + String(x_orientation) + "," + String(y_orientation) + "," + String(z_orientation) + ",accelPlaceHolder_x,accelPlaceHolder_y,accelPlaceHolder_z,magPlaceHolder_x,magPlaceHolder_y,magPlaceHolder_z,pointingErrorPlaceHolder," + flight_state;
      if (transmitting) {
        transmitPacket(toTransmit);
        packetsTransmitted++;
        timeSinceLastTransmission = 0;
      }
  }

  //State Machine
  if (flight_state == "PS_STANDBY") {
    if (vertVel > 0.03) {
      flight_state = "PS_FLIGHT";
    }
  } else if (flight_state == "PS_FLIGHT") {
    if (realAltitude > 350) {
      flight_state = "PS_PREPARE";
    }
  } else if (flight_state == "PS_PREPARE") {
    if (realAltitude < 300) {
      flight_state = "PS_OPERATIONS";
    }
  } else if (flight_state == "PS_OPERATIONS") {
    if (realAltitude < 10) {
      flight_state = "PS_LANDED";
      transmitting = false;
    }
  }
  
}
