/*
  ESP32 RC Car Controller with Laser
  Receives joystick commands via WebSocket: F, B, L, R, S, FIRE
*/

#include <Arduino.h>
#include <WiFi.h>
#include "LittleFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/pcnt.h"
#include "driver/timer.h"

// ===== MOTOR PINS =====================================================================================================================
#define MOTOR_LEFT_FWD    25
#define MOTOR_LEFT_BWD    27
#define MOTOR_RIGHT_FWD   32
#define MOTOR_RIGHT_BWD   33
#define MOTOR_LEFT_SPD    19
#define MOTOR_RIGHT_SPD   18

// ===== PLAYER ID ======
#define PLAYER_ID 0

// ===== LASER PIN =====================================================================================================================
#define LASER_PIN         16
#define LASER_READ_HIT    34
#define LASER_INTERRUPT   35

// ===== WiFi AP SETTINGS ==============================================================================================================
#define AP_SSID           "RC_Car"
#define AP_PASS           "RCCar123"
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===== STATUS TRACKING ==============================================================================================================
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 2000;

// ===== Global Variables ==============================================================================================================
int motorSpeed = 200;
volatile bool isHit = false;
volatile unsigned long hitTime = 0;
const unsigned long coolDown = 3000;
  //Packet Decoding
volatile uint32_t last_time = 0;
volatile uint32_t duration = 0;
bool header_received = false;
int bit_count = 0;
uint8_t received_data = 0;
volatile uint8_t final_value = 0;
volatile bool new_data_ready = false;

// ===== Interrupts =====================================================================================================================
void IRAM_ATTR hit_detected_isr();

// ===== FUNCTION DECLARATIONS ==========================================================================================================
void initLittleFS();
void initWebSocket();
void handleCommand(String cmd);
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void setSpeed(int motorSpeed);
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

  pinMode(LASER_INTERRUPT, INPUT_PULLUP);
  // 'CHANGE' triggers the ISR on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(LASER_INTERRUPT), hit_detected_isr, CHANGE);
  
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
  ws.cleanupClients();  //Removes old disconnected connections
  unsigned long now = millis();

  // If we saw a header but 100ms has passed without finishing the packet
  if (header_received && (micros() - last_time > 100000)) {
      header_received = false;
      bit_count = 0;
      Serial.println("Packet Timeout - Resetting");
  }

  if (new_data_ready) {
    isHit = true;
    hitTime = millis();
    //Stop Motors Immediately
    stopMotors();
    String hitmsg = "HIT_BY_P" + String(final_value + 1);
    ws.textAll(hitmsg);
    Serial.println("Hit received from ID: " + String(final_value));
    new_data_ready = false;
  }
  //Cooldown Check and Re-enable
  if(isDisabled() && (now - hitTime >= coolDown)){
    //Cooldown expired, re-enable tank
    isHit = false;
    Serial.println("Tank re-enabled after cooldown.");
    ws.textAll("READY");
  }

  //Status Update
  if(now - lastStatusUpdate >= STATUS_UPDATE_INTERVAL){
    sendStatusUpdate();
    lastStatusUpdate = now;
  }
}

// ===== INTERRUPTS ==================================================================================================================
void IRAM_ATTR hit_detected_isr(){
  uint32_t now = micros();
  duration = now - last_time;
  last_time = now;

  bool pin_state = digitalRead(LASER_INTERRUPT); //High is space, Low is Mark

  if(pin_state == HIGH) { // We just finished a MARK (Laser Pulse)
        if (duration > 1800 && duration < 2200) { 
            header_received = true; 
            bit_count = 0;
            received_data = 0;
        }
    } else { // We just finished a SPACE (Gap)
        if (header_received) {
            if (duration > 300 && duration < 500) {
                // Logic 0 detected
                received_data <<= 1;
                bit_count++;
            } else if (duration > 1000 && duration < 1400) {
                // Logic 1 detected
                received_data = (received_data << 1) | 1;
                bit_count++;
            }
        }
    }
    if (bit_count == 2) { 
        // Success! Handle your 2-bit hit data here
        final_value = received_data; // Transfer the 2-bit number
        new_data_ready = true;        // Flag for the main loop
        header_received = false;
        bit_count = 0;
        received_data = 0;
    }
}

// ===== COMMAND HANDLER ==============================================================================================================
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

// ===== MOTOR CONTROL ===============================================================================================================
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
    analogWrite(MOTOR_LEFT_SPD, speed);
    analogWrite(MOTOR_RIGHT_SPD, speed);
}

//===== LASER CONTROLS ==================================================================================================================

//===== TANK HIT
bool isDisabled() {
  if (isHit && (millis() - hitTime < coolDown)) {
    return true;
  }
  return false;
}

//==== CONNECTION MAINTNENCE ============================================================================================================
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

void sendStatusUpdate() {
  String msg = "STATUS:{\"clients\":" + String(ws.count()) + "}";
  ws.textAll(msg);
}

//Checks if Website uploaded correctly
void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}


