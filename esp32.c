/*
  ESP32 RC Car Controller with Laser
  Receives joystick commands via WebSocket: F, B, L, R, S, FIRE
*/

#include <Arduino.h>
#include <WiFi.h>
#include "LittleFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ===== MOTOR PINS =====
#define MOTOR_LEFT_FWD    26
#define MOTOR_LEFT_BWD    27
#define MOTOR_RIGHT_FWD   32
#define MOTOR_RIGHT_BWD   33

// ===== LASER PIN =====
#define LASER_PIN         13

// ===== WiFi AP SETTINGS =====
#define AP_SSID           "RC_Car"
#define AP_PASS           "RCCar123"
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===== STATUS TRACKING =====
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 2000;

// ===== FUNCTION DECLARATIONS =====
void initLittleFS();
void initWebSocket();
void handleCommand(String cmd);
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void fireLaser();
void sendStatusUpdate();

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 RC Car Starting ===");

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  stopMotors();
  initLittleFS();

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("WiFi AP Started");
  Serial.println(WiFi.softAPIP());

  initWebSocket();

  // Serve website correctly
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.begin();
  Serial.println("Web server started!");
}

void loop() {
  ws.cleanupClients();

  unsigned long now = millis();
  if (now - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
    sendStatusUpdate();
    lastStatusUpdate = now;
  }
}

void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}

void initWebSocket() {
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {

    switch (type) {

      case WS_EVT_CONNECT:
        Serial.printf("Client #%u connected\n", client->id());
        stopMotors();
        sendStatusUpdate();
        break;

      case WS_EVT_DISCONNECT:
        Serial.printf("Client #%u disconnected\n", client->id());
        stopMotors();
        break;

      case WS_EVT_DATA: {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;

        if (info->final && info->index == 0 && info->len == len) {
          String cmd = "";
          cmd.reserve(len);

          for (size_t i = 0; i < len; i++) {
            cmd += (char)data[i];
          }

          cmd.trim();
          Serial.println("Received: " + cmd);
          handleCommand(cmd);
        }
        break;
      }

    } // switch end
  });

  server.addHandler(&ws);
}

// ===== COMMAND HANDLER =====

void handleCommand(String cmd) {
  if (cmd == "F") moveForward();
  else if (cmd == "B") moveBackward();
  else if (cmd == "L") turnLeft();
  else if (cmd == "R") turnRight();
  else if (cmd == "S") stopMotors();
  else if (cmd == "FIRE") fireLaser();
  else Serial.println("Unknown command: " + cmd);
}

// ===== MOTOR CONTROL =====

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  Serial.println("STOP");
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  Serial.println("FORWARD");
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  Serial.println("BACKWARD");
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  Serial.println("LEFT");
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  Serial.println("RIGHT");
}

void fireLaser() {
  Serial.println("FIRE LASER");
  digitalWrite(LASER_PIN, HIGH);
  delay(300);
  digitalWrite(LASER_PIN, LOW);
  ws.textAll("LASER_FIRED");
}

void sendStatusUpdate() {
  String msg = "STATUS:{\"clients\":" + String(ws.count()) + "}";
  ws.textAll(msg);
}
