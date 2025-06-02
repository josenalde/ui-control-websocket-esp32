#include <Arduino.h>

/********* ADAPTADO DE...
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-websocket-server-sensor/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Arduino_JSON.h>
#include <freertos/FreeRTOS.h>
#include "SPIFFS.h"

// Replace with your network credentials
const char* ssid = "Servos_2g";
const char* password = "discipulo";

// Settings according to resistor value and capacitor value
static const float R = 10000; //ohms 10k
static const float C = 0.0001; //faraday 100 uF
static const float timeConstant = R*C;
static const TickType_t samplingInterval = ((timeConstant*1000)/10.) / portTICK_PERIOD_MS; // 200ms = 0.2s
static const TickType_t timeToApplyMV = (timeConstant*4*1000) / portTICK_PERIOD_MS; 

// Globals
static const uint8_t SENSOR_PIN = 34;
static const uint8_t MV_PIN = 25;
static const uint8_t FLAG_DISCHARGE_PIN = 2;
volatile float VCC = 0.0; //initial value applied from t=0 to t=timeToApplyMV
volatile uint16_t sensorReadingInt;
volatile float sensorReadingVoltage;
String jsonString;

static TimerHandle_t getSensorReadingTimer = NULL;
static TimerHandle_t stepInputStartTimer = NULL;

// Create a WebSocket object
WebSocketsServer websocketserver = WebSocketsServer(8080);
WebServer webserver(80);

// Callbacks
void notifyClients(String sensorReadings) {
  websocketserver.broadcastTXT(sensorReadings);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] disconnected\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = websocketserver.remoteIP(num);
      Serial.printf("[%u] connection from ", num);
      Serial.println(ip.toString());
    }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload) ;
      notifyClients(jsonString);
      //websocketserver.sendTXT(num, payload); //send payload back to the client
      break;
    case WStype_ERROR:
    case WStype_BIN:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default: break;
  }
}
// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long refreshInterval = ((timeConstant*1000)/10.); 

void getSensorReadingCallback(TimerHandle_t xTimer) {
  readings["time"] = String(xTaskGetTickCount() / 1000.);
  sensorReadingInt = analogRead(SENSOR_PIN);
  sensorReadingVoltage = (sensorReadingInt) * (VCC/4096.);
  readings["MV"] =  String(VCC);
  readings["PV"] =  String(sensorReadingVoltage);
  Serial.print(xTaskGetTickCount() / 1000., 1);
  Serial.print(",");
  Serial.print(VCC);
  Serial.print(",");
  Serial.println(sensorReadingVoltage, 1);
  jsonString = JSON.stringify(readings);
  notifyClients(jsonString);
  //websocketserver.cleanupClients(); // in order to avoid exceeding maximum clients - a timer for this is necessary
}

// send step after <timeToStartInterval> seconds
void setStepInputReadingCallback(TimerHandle_t xTimer) {
  digitalWrite(MV_PIN, HIGH);
  VCC = 3.3;
}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println(WiFi.localIP());
}

void initWebSocket() {
  websocketserver.begin();
  websocketserver.onEvent(onWebSocketEvent);
}

/* suppose a button@frontend to reset the capacitor (discharge) */
  void dischargeCapacitor() { 
     String data = webserver.uri(); 
     if (data=="/discharge") {
       digitalWrite(MV_PIN, LOW);
       digitalWrite(FLAG_DISCHARGE_PIN, HIGH);
       vTaskDelay(pdMS_TO_TICKS(timeConstant*4*1000)); //RC * 4 * 1000 to get in ms
       digitalWrite(FLAG_DISCHARGE_PIN, LOW);
       webserver.send(200,"text/plain","ok");
     } 
  }

void setup() {
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();
  initWebSocket();

  webserver.serveStatic("/", SPIFFS, "/index.html");
  webserver.serveStatic("/style.css", SPIFFS, "/style.css");
  webserver.serveStatic("/script.js", SPIFFS, "/script.js");
  webserver.on("/discharge", dischargeCapacitor);
  webserver.begin();
  //---------------------

  // Configure I/O direction
  pinMode(MV_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(FLAG_DISCHARGE_PIN, OUTPUT);
    // Create a auto-reload timer for sensor readings
  getSensorReadingTimer = xTimerCreate(
                      "getSensorReadingTimer",     // Name of timer
                      samplingInterval,            // Period of timer (in ticks)
                      pdTRUE,              // Auto-reload TRUE, one_shot FALSE
                      (void *)0,            // Timer ID
                      getSensorReadingCallback);  // Callback function
  // Create a one shot timer for step output
  stepInputStartTimer  = xTimerCreate(
                      "stepInputStartTimer ",     // Name of timer
                      timeToApplyMV,            // Period of timer (in ticks)
                      pdFALSE,              // Auto-reload TRUE, one_shot FALSE
                      (void *)1,            // Timer ID
                      setStepInputReadingCallback);  // Callback function

  xTimerStart(getSensorReadingTimer, 0);
  xTimerStart(stepInputStartTimer, 0); 
}

void loop() {
  webserver.handleClient();
  websocketserver.loop();
}