#include <Servo.h>
#include "WiFiS3.h"
#include <ArduinoJson.h>


#define TRIG1 2
#define ECHO1 5
#define TRIG2 3
#define ECHO2 6
#define TRIG3 4
#define ECHO3 7

#define SERVO_PIN 9

WiFiServer server(80);
int status = WL_IDLE_STATUS;


// replace these below based on wi-fi available
const char* ssid = "DIGI-83Fy";
const char* pass = "h8bE87J4";

Servo barrier;
int spotStatus[3] = {0, 0, 0};
int barrierPosition = 0; 

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

}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float distance = pulseIn(echoPin, HIGH) * 0.034 / 2;
  return distance;
}

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

int calculateBarrierTargetPosition(int freeSpots) {
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


void serveUpTheParkState(String parkState) {
  
  WiFiClient client = server.available();
    if (client) {                             
    Serial.println("new client");           
    String currentLine = "";                
    while (client.connected()) {            
      if (client.available()) {             
        char c = client.read();            
        Serial.write(c);                    
        if (c == '\n') {                   

        
          if (currentLine.length() == 0) {
           
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:application/json");
            client.println("Access-Control-Allow-Origin:*");

            client.println();

            client.print(parkState);

            client.println();
            break;
          } else {    
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

void loop() {
  spotStatus[0] = (measureDistance(TRIG1, ECHO1) < 10) ? 1 : 0;
  spotStatus[1] = (measureDistance(TRIG2, ECHO2) < 10) ? 1 : 0;
  spotStatus[2] = (measureDistance(TRIG3, ECHO3) < 10) ? 1 : 0;

  int occupiedSpots = spotStatus[0] + spotStatus[1] + spotStatus[2];
  int freeSpots = 3 - occupiedSpots;
  
  int targetPosition = calculateBarrierTargetPosition(freeSpots);
  
  if (barrierPosition != targetPosition) {
    moveBarrierSmooth(targetPosition);
  }

  String parkState = String("{") + 
    String("0: ") + String(spotStatus[0]) + String(",") + 
    String("1: ") + String(spotStatus[1]) + String(",") + 
    String("2: ") + String(spotStatus[2]) +   
    String("}");

  Serial.println(parkState);
  serveUpTheParkState(parkState);

  delay(1000); 
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