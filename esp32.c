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
#define MOTOR_LEFT_FWD    25
#define MOTOR_LEFT_BWD    27
#define MOTOR_RIGHT_FWD   32
#define MOTOR_RIGHT_BWD   33
#define MOTOR_LEFT_SPD    19
#define MOTOR_RIGHT_SPD   18

// ===== LASER PIN =====
#define LASER_PIN         13
#define LASER_HIT         16

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

// ===== Global Variables =====
int motorSpeed = 200;
volatile bool isHit = false;
volatile unsigned long hitTime = 0;
const unsigned long coolDown = 3000;

// ===== Interrupts =====
void IRAM_ATTR hitDetected();

// ===== FUNCTION DECLARATIONS =====
void initLittleFS();
void initWebSocket();
void handleCommand(String cmd);
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void setSpeed(int motorSpeed);
void fireLaser();
bool isDisabled();
void sendStatusUpdate();

//Initializes pins and sets up WebSocket
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 RC Car Starting ===");

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(LASER_HIT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LASER_HIT), hitDetected, FALLING);

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

//Removes old disconnected connections
void loop() {
  ws.cleanupClients();

  if (isHit && (millis() - hitTime < 100)) { // React immediately to hits
    stopMotors();
    ws.textAll("HIT");
  }

  unsigned long now = millis();
  if (now - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
    sendStatusUpdate();
    lastStatusUpdate = now;
  }
}

//Checks if Website uploaded correctly
void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}

//Starts WebSocket
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

// ===== INTERRUPTS =====
void IRAM_ATTR hitDetected(){
  isHit = true;
  hitTime = millis();
}

// ===== COMMAND HANDLER =====
void handleCommand(String cmd) {
  if (isDisabled()) {
    Serial.println("Commands disabled - tank was hit!");
    return;
  }

  if (cmd == "F") moveForward();
  else if (cmd == "B") moveBackward();
  else if (cmd == "L") turnLeft();
  else if (cmd == "R") turnRight();
  else if (cmd == "S") stopMotors();
  else if (cmd == "FIRE") fireLaser();
  else if (cmd.startsWith("SPEED:")){
    int newSpeed = cmd.substring(6).toInt();
    if(newSpeed >= 0 && newSpeed <= 255){
      motorSpeed = newSpeed;
      Serial.printf("Speed set to: %d\n", motorSpeed);
    }
    setSpeed(motorSpeed);
  }
  else Serial.println("Unknown command: " + cmd);
}

// ===== MOTOR CONTROL =====
void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setSpeed(0);
  Serial.println("STOP");
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setSpeed(motorSpeed);
  Serial.println("FORWARD");
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  setSpeed(motorSpeed);
  Serial.println("BACKWARD");
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setSpeed(motorSpeed);
  Serial.println("LEFT");
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  setSpeed(motorSpeed);
  Serial.println("RIGHT");
}

void setSpeed(int speed){
    analogWrite(19, speed);
    analogWrite(18, speed);
}

//===== LASER CONTROLS =====
void fireLaser() {
  Serial.println("FIRE LASER");
  digitalWrite(LASER_PIN, HIGH);
  delay(300);
  digitalWrite(LASER_PIN, LOW);
  ws.textAll("LASER_FIRED");
}

bool isDisabled() {
  if (isHit && (millis() - hitTime < coolDown)) {
    return true;
  }
  if (isHit && (millis() - hitTime >= coolDown)) {
    isHit = false; // Re-enable after timeout
  }
  return false;
}

void sendStatusUpdate() {
  String msg = "STATUS:{\"clients\":" + String(ws.count()) + "}";
  ws.textAll(msg);
}
