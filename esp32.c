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

// ===== MOTOR PINS =====
#define MOTOR_LEFT_FWD    25
#define MOTOR_LEFT_BWD    27
#define MOTOR_RIGHT_FWD   32
#define MOTOR_RIGHT_BWD   33
#define MOTOR_LEFT_SPD    19
#define MOTOR_RIGHT_SPD   18

// ===== LASER PIN =====
#define LASER_PIN         16
#define LASER_READ_HIT    34
#define LASER_INTERRUPT   35
#define LEDC_CHANNEL       0  // LEDC channel for PWM
#define LEDC_RESOLUTION   10  // 10-bit resolution for PWM
#define LEDC_DUTY         512 // 50% duty cycle

// ==== PCNT CONFIGURATION ====
#define PCNT_HIT_UNIT    PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO LASER_READ_HIT //GPIO 34

// ===== Player Frequencies (Hz) =====
#define PLAYER1_FREQUENCY_HZ    5000
#define PLAYER2_FREQUENCY_HZ    4000
#define PLAYER3_FREQUENCY_HZ    3000
#define PLAYER4_FREQUENCY_HZ    2000
#define PLAYER5_FREQUENCY_HZ    1000
#define FREQUENCY_TOLERANCE_HZ  50     // +/- tolerance

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
int currentFrequencyHz = 5000; //5 kHz
unsigned long lastLaserFiredTime = 0;
const unsigned long laserFireDuration = 300; // milliseconds
volatile bool frequencyReady = false;
volatile int16_t pulseCount = 0;
volatile unsigned long measurementStartTime = 0;
volatile unsigned long measurementStopTime = 0;
String playerCode = "HIT_UNKNOWN";

// ===== Interrupts =====
void IRAM_ATTR hitDetected();

// ===== FUNCTION DECLARATIONS =====
void initLittleFS();
void initWebSocket();
void initPCNTFrequencyReader();
void handleCommand(String cmd);
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void setSpeed(int motorSpeed);
void setupLaserPWM();
void startLaserSignal();
void stopLaserSignal();
void fireLaser();
bool isDisabled();
void whoHitMe(int frequency_int);
void checkHitSignalEnd();
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
  pinMode(LASER_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LASER_INTERRUPT), hitDetected, FALLING);
  pinMode(LASER_READ_HIT, INPUT);
  setupLaserPWM();
  initPCNTFrequencyReader();
  
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
  unsigned long now = millis();

  //Check if hit signal ended and frequency can be read
  checkHitSignalEnd();

  //Process frequency reading
  if (frequencyReady){
    unsigned long measurementDuration_ms = measurementStopTime - measurementStartTime;
    if(measurementDuration_ms == 0) {
      Serial.println("Error: Measurement duration was zero.");
      ws.textAll("HIT_UNKNOWN");
    } else {
      float measuredFrequency_Hz = (float)pulseCount / ((float)measurementDuration_ms / 1000.0);
      int frequency_int = round(measuredFrequency_Hz);
      Serial.printf("Measured Frequency: %d Hz (Pulses: %d, Duration: %lu ms)\n",
         frequency_int, pulseCount, measurementDuration_ms);
      whoHitMe(frequency_int);
      ws.textAll(playerCode);
    }
    frequencyReady = false; //Reset flag
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
  //Laser stop logic (for the car's own firing)
  if(lastLaserFiredTime != 0 && (now - lastLaserFiredTime > laserFireDuration)){
    stopLaserSignal();
    lastLaserFiredTime = 0; //Reset
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

void initPCNTFrequencyReader() {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = PCNT_INPUT_SIG_IO,
    .ctrl_gpio_num = -1, // Not using control pin
    .lctrl_mode = PCNT_MODE_REVERSE,  //No control, just set to reverse
    .hctrl_mode = PCNT_MODE_KEEP,  // Control mode not used
    .pos_mode = PCNT_COUNT_INC,   // Count on rising edge
    .neg_mode = PCNT_COUNT_DIS,   // Do not count on falling edge
    .unit = PCNT_HIT_UNIT,
    .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config);
  pcnt_filter_enable(PCNT_HIT_UNIT);
  pcnt_set_filter_value(PCNT_HIT_UNIT, 100); // Filter out pulses shorter than 100us
  //IMPORTANT: Keep the counter paused, it will be started by the ISR
  pcnt_counter_pause(PCNT_HIT_UNIT);
  pcnt_counter_clear(PCNT_HIT_UNIT);
  Serial.println("PCNT initialized for GPIO 34");

  
}

// ===== INTERRUPTS =====
void IRAM_ATTR hitDetected(){
  if(!isHit){
    //Stops motors
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_BWD, LOW);
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_BWD, LOW);
    //Stops Laser Pulse
    ledcWrite(LEDC_CHANNEL, 0); // Turn off laser
    //Starts to caputre incoming signal frequency
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    //Set flags for the main loop
    isHit = true;
    hitTime = millis();
    measurementStartTime = hitTime;
  }
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
void setupLaserPWM() {
  // Configure LEDC timer for frequency 
  ledcSetup(LEDC_CHANNEL, currentFrequencyHz, LEDC_RESOLUTION);
  ledcAttachPin(LASER_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, 0); // Start with laser off
  Serial.printf("LEDC configured on GPIO %d at %d Hz.\n", LASER_PIN, currentFrequencyHz);
}

void startLaserSignal(){
  ledcWrite(LEDC_CHANNEL, LEDC_DUTY); // Turn on laser with specified duty cycle
  Serial.printf("Laser signal started at %d Hz.\n", currentFrequencyHz);
  ws.textAll("LASER_STARTED");
}

void stopLaserSignal(){
  ledcWrite(LEDC_CHANNEL, 0); // Turn off laser
  Serial.println("Laser signal stopped.");
  ws.textAll("LASER_STOPPED");
}

void fireLaser() {
  if(lastLaserFiredTime == 0 && !isDisabled()){
    Serial.println("FIRE LASER");
    lastLaserFiredTime = millis();
    startLaserSignal();
    ws.textAll("LASER_FIRED");
  }
}

bool isDisabled() {
  if (isHit && (millis() - hitTime < coolDown)) {
    return true;
  }
  return false;
}

void whoHitMe(int frequency_int){
  if (abs(frequency_int - PLAYER1_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ) {
      playerCode = "HIT_BY_P1";
  } else if (abs(frequency_int - PLAYER2_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ) {
      playerCode = "HIT_BY_P2";
  } else if (abs(frequency_int - PLAYER3_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ) {
      playerCode = "HIT_BY_P3";
  } else if (abs(frequency_int - PLAYER4_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ) {
      playerCode = "HIT_BY_P4";
  } else if (abs(frequency_int - PLAYER5_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ) {
      playerCode = "HIT_BY_P5";
  }
}

void checkHitSignalEnd(){
  if(isHit&& !frequencyReady){
    unsigned long now = millis();
    if((now - measurementStartTime > laserFireDuration + 50) || 
       (digitalRead(LASER_READ_HIT) == LOW && (now - measurementStartTime > 50))){
        //Stop PCNT and read frequency
        pcnt_counter_pause(PCNT_UNIT_0);
        pcnt_get_counter_value(PCNT_UNIT_0, (int16_t*)&pulseCount); //Use defined unit and cast to int16_t*
        measurementStopTime = now;
        //Set processing flag
        frequencyReady = true;
        //Clear PCNT for next hit (will remain cleared until the next hitDetected ISR runs)
        pcnt_counter_clear(PCNT_UNIT_0);
        Serial.println("Hit signal end detected. Reading count.");
    }
  }
}

void sendStatusUpdate() {
  String msg = "STATUS:{\"clients\":" + String(ws.count()) + "}";
  ws.textAll(msg);
}
