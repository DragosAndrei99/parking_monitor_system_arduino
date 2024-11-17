#include <Servo.h>
#include "WiFiS3.h"
#include <ArduinoJson.h>
#include <NTPClient.h>
#include "RTC.h"


#define TRIG1 2
#define ECHO1 5
#define TRIG2 3
#define ECHO2 6
#define TRIG3 4
#define ECHO3 7

#define SERVO_PIN 9

WiFiServer server(80);
int status = WL_IDLE_STATUS;
WiFiUDP Udp;
NTPClient timeClient(Udp);

// replace these below based on wi-fi available
const char* ssid = "DIGI-83Fy";
const char* pass = "h8bE87J4";

Servo barrier;
int spotStatus[3] = {0, 0, 0};
RTCTime spotStatusTimeStart[3] = {0, 0, 0};
int barrierPosition = 0; 
bool automateBarrier = true;
// ---------------------------------------------------------------------------
// ---------------------------SETUP-------------------------------------------


void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                  

    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  printWifiStatus();             
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println(ip);
}

void connectToTimeServer() {
  RTC.begin();
  Serial.println("\nStarting connection to server...");
  timeClient.begin();
  timeClient.update();

  auto timeZoneOffsetHours = 2;
  auto unixTime = timeClient.getEpochTime() + (timeZoneOffsetHours * 3600);
  Serial.print("Unix time = ");
  Serial.println(unixTime);
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);

  RTCTime currentTime;
  RTC.getTime(currentTime); 
  Serial.println("The RTC was just set to: " + String(currentTime));

  spotStatusTimeStart[0] = currentTime;
  spotStatusTimeStart[1] = currentTime;
  spotStatusTimeStart[2] = currentTime;

}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  connectToWiFi();

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  barrier.attach(SERVO_PIN);
  barrier.write(barrierPosition); 

  Serial.println("System Initialized");
  server.begin();
  connectToTimeServer();                           
}


// ---------------------------------------------------------------------------
// -----------------------PARKING SENSOR--------------------------------------

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float distance = pulseIn(echoPin, HIGH) * 0.034 / 2;
  return distance;
}

void updateSpotStatus(int &singleSpotStatus, RTCTime &singleSpotStatusTimeStart, int singleCurrentSpotStatus,RTCTime currentTime) {
   if(singleCurrentSpotStatus != singleSpotStatus && String(singleSpotStatusTimeStart) != String(currentTime)) {
    singleSpotStatus = singleCurrentSpotStatus;
    singleSpotStatusTimeStart = currentTime;
  };
}


// ---------------------------------------------------------------------------
// -----------------------BARRIER---------------------------------------------

void moveBarrierSmooth(int targetPosition) {
  if (barrierPosition < targetPosition) {
    for (int pos = barrierPosition; pos <= targetPosition; pos += 5) {
      barrier.write(pos);
      delay(20); 
    }
  } else {
    for (int pos = barrierPosition; pos >= targetPosition; pos -= 5) {
      barrier.write(pos);
      delay(20); 
    }
  }
  barrierPosition = targetPosition; 
}

int calculateBarrierTargetPosition() {
  int occupiedSpots = spotStatus[0] + spotStatus[1] + spotStatus[2];
  int freeSpots = 3 - occupiedSpots;
  int targetPosition;
    if (freeSpots > 0) {
      targetPosition = 90; 
      // Serial.println("Barrier Opening: Free spots available.");
    } else {
      targetPosition = 0; 
      // Serial.println("Barrier Closing: No free spots.");
    }

  return targetPosition;
}

void operateBarrier() {
  int targetPosition = calculateBarrierTargetPosition();
  if (barrierPosition != targetPosition) {
    moveBarrierSmooth(targetPosition);
  }
}

// ---------------------------------------------------------------------------
// ----------------------SERVER-----------------------------------------------

void serveUpTheParkState(String parkingSpotsState, String barrierState) {
  
  WiFiClient client = server.available();
    if (client) {                             
    Serial.println("new client");           
    String currentLine = "";
    String requestPath = "";      
    while (client.connected()) {            
      if (client.available()) {             
        char c = client.read();            
        Serial.write(c);                    
        if (c == '\n') {                   

          Serial.println();

          if (currentLine.length() == 0) {
           
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:application/json");
            client.println("Access-Control-Allow-Origin:*");

            client.println();

            if (requestPath.startsWith("GET /barrier")) {
              client.print(barrierState);
            } else if (requestPath.startsWith("PUT /close-barrier")) {
              if(barrierPosition == 90) {
                automateBarrier = false;
                moveBarrierSmooth(0);
              }
            } else if (requestPath.startsWith("PUT /open-barrier")) {
              if(barrierPosition == 0) {
                automateBarrier = false;
                moveBarrierSmooth(90);
              }
            } else if (requestPath.startsWith("PUT /auto-barrier")) {
              automateBarrier = true;
            } else if (requestPath.startsWith("GET /park")) {
              client.print(parkingSpotsState);
            } else {
              client.println("{\"error\":\"Not Found\"}");
            }

            client.println();
            break;
          } else {
            if(currentLine.startsWith("GET") || currentLine.startsWith("PUT")) {
              int pathStart = currentLine.indexOf(' ') + 1;
              int pathEnd = currentLine.indexOf(' ', pathStart);
              String httpMethod = currentLine.startsWith("PUT ") ? "PUT " : "GET ";
              requestPath = httpMethod + currentLine.substring(pathStart, pathEnd);
              Serial.println("Requested Path: " + requestPath);
            }
            currentLine = "";
          }

        
        } else if (c != '\r') {
          currentLine += c;
        }
      }
      
    }
    client.stop();
    Serial.println("client disconnected");
  }
}

String composeParkingSpotsStateJsonString() {
  return String("{") + 
    "\"" + "parkingSpotsState" + "\": " + "[" + "[" + "\"" + String(spotStatus[0]) + "\"" + ", " + "\"" + String(spotStatusTimeStart[0]) + "\"" + "]" + String(",") + 
    + "[" + "\"" + String(spotStatus[1]) + "\"" + ", " + "\"" + String(spotStatusTimeStart[1]) + "\"" + "]" + String(",") + 
    + "[" + "\"" + String(spotStatus[2]) + "\"" + ", " + "\"" + String(spotStatusTimeStart[2]) + "\"" + "]" + "]" +   
    String("}");
}

String composeBarrierStateJsonString() {
    return String("{") + 
     "\"" + "barrier" + "\": " + "\"" + (barrierPosition == 90 ? String(0) : String(1)) + "\"" +
      String("}");
}

// ---------------------------------------------------------------------------
// -----------------------LOOP------------------------------------------------

void loop() {
  // spot status update
  RTCTime currentTime;
  RTC.getTime(currentTime); 

  int currentSpotStatus1 = (measureDistance(TRIG1, ECHO1) < 10) ? 1 : 0;
  updateSpotStatus(spotStatus[0], spotStatusTimeStart[0], currentSpotStatus1, currentTime);
  int currentSpotStatus2 = (measureDistance(TRIG2, ECHO2) < 10) ? 1 : 0;
  updateSpotStatus(spotStatus[1], spotStatusTimeStart[1], currentSpotStatus2, currentTime);
  int currentSpotStatus3 = (measureDistance(TRIG3, ECHO3) < 10) ? 1 : 0;
  updateSpotStatus(spotStatus[2], spotStatusTimeStart[2], currentSpotStatus3, currentTime);

  //automatic barrier operation
  if(automateBarrier) operateBarrier();

  // server
  String parkingSpotsState = composeParkingSpotsStateJsonString();
  String barrierState = composeBarrierStateJsonString();
  serveUpTheParkState(parkingSpotsState, barrierState);

  // ----------------------------------------------
  delay(1000); 
}
