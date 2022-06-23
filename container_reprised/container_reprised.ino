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
#include <SD.h>

#define latchPin 6
//Motor Instantiations
#define AIN1 11
#define AIN2 10
#define PWMA 9
#define STBY 12

#define baudRate 115200

String timeConvert(int n) {
  String o = String(n);
  if (o.length() != 2) {o = "0" + o;}
  return o;
}

float MBA_To_Altitude_Meters(float millbars) {
  float altitudeInFeet = (1 - pow(millbars/1013.25, 0.190284)) * 145366.45;
  float altitudeInMeters = altitudeInFeet * 0.3048;
  return altitudeInMeters;
}

int strToInt(String s) {
  char char_array[s.length() + 1];
  strcpy(char_array, s.c_str());
  int n = atoi(char_array);
  return n;
}

class flightComputer {
  public:
    int teamID = 1091;

    String flightState = "FS_STANDBY";

    String lastCommand = "NULL";
    
    Motor tetherMotor = Motor(AIN1, AIN2, PWMA, 35, STBY);

    int timeSinceLastPayload = 0;
    int deltaTime;
    int lastExecutionTime = 0;
    int lastTime = 0;

    int timeSinceLastTransmission = 0;
    int transmissionInterval = 1000;
    int payloadDelta = 0;
    int payloadInterval = 250;
    int packetsTransmitted = 0;
    bool transmitting = true;
    String currentPacket = "";

    int packetAdress = 0;
    int addOffset = 0;
    int backupStateOffset = 5;

    float altitudeOffset = 0;
    float realAltitude = 0;
    float relativeAltitude = 0;

    bool offsetNeeded = true;

    float temperature;
    float pressure;

    float gpsLat = 0;
    float gpsLong = 0;
    float gpsAlt = 0;
    String gpsHour = "";
    String gpsMinute = "";
    String gpsSecond = "";
    String gpsTime = "00:00:00";
    int gpsSats = 0;

    String payloadReleased = "N";

    MS5611 ms;
    SFE_UBLOX_GNSS myGNSS;
    float voltage = 0;

    bool simulationMode = false;

    void backupPackets() {
      EEPROM.put(packetAdress + addOffset, packetsTransmitted);
    }
    
    void restorePackets(){
      int out = 0;
      EEPROM.get(packetAdress + addOffset, packetsTransmitted);
    }
    
    void clearPackets() {
      EEPROM.put(packetAdress + addOffset, 0);
    }

    //Clock Backups
    void backupTime() {
      time_t a = now();
      float n = a;
      EEPROM.put(packetAdress + addOffset + backupStateOffset*3, n);
    }
    
    void restoreTime() {
      float n;
      EEPROM.get(packetAdress + addOffset + backupStateOffset*3, n);
      time_t t = n;
      setTime(t);
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
    
    char inputBuffer[80];
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
          if (bytesRecvd == 80) {
            bytesRecvd = 80 - 1;
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
          if (bytesRecvd == 80) {
            bytesRecvd = 80 - 1;
          }
        }
    
        if (x == startMarker) { 
          bytesRecvd = 0; 
          readInProgress = true;
        }
      }
    }

    int latchOverride = 0;

    void processCommand(String com) {
      if (com.substring(9,11) == "CX") {
        lastCommand = "CX";
        if (com.substring(12,14) == "ON") {
          transmitting = true;
        } else {
          transmitting = false;
        }
      } else if (com.substring(9, 11) == "ST") {
        lastCommand = "ST";
        setTime(strToInt(com.substring(12, 14)), strToInt(com.substring(15, 17)),strToInt(com.substring(18, 20)),11,6,2022);
      } else if (com.substring(9,12) == "RST") {
        getOffsetAltitude();
        flightState = "FS_STANDBY";
        packetsTransmitted = 0;
        lastCommand = "RST";
        clearPackets();
      } else if (com.substring(9,12) == "LT") {
        openLatch();
      } else if (com.substring(9,12) == "DRP") {
        lastCommand = "DRP";
        flightState = "FS_LANDING";
        openLatch();
      } else if (com.substring(9,11) == "TG") {
        toggleCamera();
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

    void transmitPayload(String transmit) {
      String toTransmit = "1091," + timeConvert(hour()) + ":" + timeConvert(minute()) + ":" + timeConvert(second());
      toTransmit += "," + String(packetsTransmitted) + "," + "T" + "," + transmit;
      transmitPacket(toTransmit);
      packetsTransmitted++;
    }

    Servo latchServo;
    
    void openLatch() {
      latchServo.write(170);
    }

    void closeLatch() {
      latchServo.write(140);
    }

    void getVoltage() {
      voltage = analogRead(A2) * (3.3/1023.0);
    }

    void getGPS() {
      gpsLong = myGNSS.getLongitude();
      gpsLong /= 10000000;
      gpsLat = myGNSS.getLatitude();
      gpsLat /= 10000000;

      gpsHour = timeConvert(myGNSS.getHour());
      gpsMinute = timeConvert(myGNSS.getMinute());
      gpsSecond = timeConvert(myGNSS.getSecond());

      gpsSats = myGNSS.getSIV();

      gpsAlt = myGNSS.getAltitude();
      gpsAlt /= 1000;
    }

    void assemblePacket() {
      getGPS();
      gpsTime = gpsHour +":"+ gpsMinute +":"+ gpsSecond; 
      currentPacket += String(teamID) + ",";
      currentPacket += timeConvert(hour()) + ":" + timeConvert(minute()) + ":" + timeConvert(second()) + ",";
      currentPacket += String(packetsTransmitted) + ",C," + payloadReleased + "," + String(relativeAltitude) + ",";
      currentPacket += String(temperature) + "," + String(voltage) + ",";
      currentPacket += gpsTime + "," + String(gpsLat) + "," + String(gpsLong) + "," + String(gpsAlt) + "," + String(gpsSats) + ",";
      currentPacket += flightState + "," + lastCommand;
    }

    void transmitPacket(String s) {
      Serial.println(s);
      Serial2.println(s);

      if (dataFile) {
        dataFile.println(s);
        dataFile.close();
      }
    }

    void refreshMS() {
      ms.read();
      getTemp();
      getPress();
    }

    void getTemp() {
      temperature = ms.getTemperature();
    }

    void getPress() {
      pressure = ms.getPressure();
    }

    void getBarAltitude() {
      realAltitude = MBA_To_Altitude_Meters(pressure);
    }

    void getOffsetAltitude() {
      refreshMS();
      getBarAltitude();
      altitudeOffset = realAltitude;
    }

    File dataFile;

    void toggleCamera() {
      digitalWrite(14, LOW);
      
      delay(1000);
      digitalWrite(14, HIGH);
    }

    int blinkInterval;
    int blinkDelta = 0;

    int ledON = 1;
    
    void flightComputerSetup() {

      SD.begin(BUILTIN_SDCARD);

      dataFile = SD.open("datalog.txt", FILE_WRITE);
      
      blinkInterval = transmissionInterval / 2;
      Serial.begin(baudRate);
      Serial2.begin(baudRate);
      Serial5.begin(baudRate);
      Serial4.begin(9600);

      pinMode(14, OUTPUT);
      digitalWrite(14, HIGH);

      latchServo.attach(latchPin);
      closeLatch();

      //MS Setup
      ms = MS5611(0x77);
      bool a = false;
      while (!a) {a = ms.begin(&Wire2);}

      restorePackets();

      getOffsetAltitude();

      //GPS Setup
      myGNSS.begin(Serial4);

      lastTime = millis();

      setSyncProvider(RTC.get);
      restoreTime();

      flightState = restoreState();
    }

    void calculateTime() {
      int currTime = millis();
      deltaTime = currTime - lastTime;
      lastTime = currTime;
      timeSinceLastTransmission += deltaTime;
      payloadDelta += deltaTime;
    }

    void calculateState() {
      if (flightState == "FS_STANDBY") {
        digitalWrite(0, LOW);
        digitalWrite(1, LOW);

        digitalWrite(15, HIGH);
        

        if (relativeAltitude > 100) {
          flightState = "FS_ASCENT";
        }
      } else if (flightState == "FS_ASCENT") {
        if (relativeAltitude > 350) {
          flightState = "FS_PEAK";
          //turn on camera
        }
      } else if (flightState == "FS_PEAK") {
        if (relativeAltitude < 400) {
          openLatch();
          flightState = "FS_LANDING";
        }
      } else if (flightState == "FS_LANDING") {
        openLatch();
        if (relativeAltitude < 350) {
          payloadReleased = "R";
          Serial5.println("{DROP}");
          tetherMotor.drive(-50);
        }
        
        if (relativeAltitude < 10) {
          //turn off camera
          flightState = "FS_LANDED";
        }
      } else if (flightState == "FS_LANDED") {
        transmitting = false;
        digitalWrite(0, HIGH);
        digitalWrite(1, HIGH);
      }
    }

    void flightComputerLoop() {
      calculateTime();
      
      refreshMS();
      getBarAltitude();
      getVoltage();
      
      relativeAltitude = realAltitude - altitudeOffset;

      getDataFromPC();
      getDataFromPC_wired();
      getDataFromPayload();

      if (timeSinceLastTransmission > transmissionInterval) {
        if (transmitting) {
          assemblePacket();
          transmitPacket(currentPacket);
          currentPacket = "";
          packetsTransmitted++;
          timeSinceLastTransmission = 0;
          backupPackets();
          backupTime();
          backupState(flightState);
        }
      }      

      blinkDelta += deltaTime;
      if (blinkDelta > blinkInterval) {
        ledON = 1 - ledON;
        if (ledON == 1) {
          digitalWrite(13, HIGH);
        } else {
          digitalWrite(13, LOW);
        }
        blinkDelta = 0;
      }

      if (payloadDelta > payloadInterval) {
        if (flightState == "FS_LANDING") {
          if (relativeAltitude < 350) {
            if (relativeAltitude > 10) {
              Serial5.println("{L}");
              payloadDelta = 0;
            }
          }
        }
      }

      calculateState();
    }
};

flightComputer comp;

void setup() {
  // put your setup code here, to run once:
  comp.flightComputerSetup();

}

void loop() {
  // put your main code here, to run repeatedly:
  comp.flightComputerLoop();

}
