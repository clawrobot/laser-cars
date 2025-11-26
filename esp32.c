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
#define PCNT_EVT_CTRL_IO  -1             //Not using an external control pin

// ===== TIMER CONFIGURATION =====
#define TIMER_DIVIDER        16000  //16000 for 10ms gate time (80MHz(PCNT clock) / 16000 = 5kHz)
#define TIMER_SCALE          (80000000 / TIMER_DIVIDER)  //5000 ticks per second
#define TIMER_INTERVAL_SEC   0.01 // 10ms interval
#define TIMER_GROUP         TIMER_GROUP_0
#define TIMER_IDX           TIMER_0

// ===== Player Frequencies (Hz) =====
#define PLAYER1_FREQUENCY_HZ  5000
#define PLAYER2_FREQUENCY_HZ  4000
#define PLAYER3_FREQUENCY_HZ  3000
#define PLAYER4_FREQUENCY_HZ  2000
#define PLAYER5_FREQUENCY_HZ  1000
#define FREQUENCY_TOLERANCE_HZ 50 // +/- tolerance

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
volatile int pulseCount = 0;

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
void whoHitMe();
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

  if (isHit && (millis() - hitTime < 100)) { // React immediately to hits
    //Stop motors and notify clients and start frequency measurement
    detachInterrupt(digitalPinToInterrupt(LASER_INTERRUPT)); // Disable further interrupts temporarily
    stopMotors();
    ws.textAll("HIT");
    whoHitMe(); //Start PCNT
    attachInterrupt(digitalPinToInterrupt(LASER_INTERRUPT), hitDetected, FALLING); // Re-enable interrupts
  }

  // PCNT Frequency Check
  if (frequencyReady) {
    timer_pause(TIMER_GROUP, TIMER_IDX);
    //Calculate frequency (Pulses / Gate Time)
    float measuredFrequency_Hz = (float)pulseCount / TIMER_INTERVAL_SEC;
    int frequency_int = round(measuredFrequency_Hz);
    Serial.printf("Measured Frequency: %d Hz (Count: %d)\n", frequency_int, pulseCount);
    //Player Code Check (1kHz to 5kHz range)
    if(abs(frequency_int - PLAYER1_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ){
      Serial.println("Hit by Player 1!");
      ws.textAll("HIT_BY_P1");
    }
    else if(abs(frequency_int - PLAYER2_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ){
      Serial.println("Hit by Player 2!");
      ws.textAll("HIT_BY_P2");
    }
    else if(abs(frequency_int - PLAYER3_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ){
      Serial.println("Hit by Player 3!");
      ws.textAll("HIT_BY _P3");
    }
    else if(abs(frequency_int - PLAYER4_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ){
      Serial.println("Hit by Player 4!");
      ws.textAll("HIT_BY_P4");
    }
    else if(abs(frequency_int - PLAYER5_FREQUENCY_HZ) <= FREQUENCY_TOLERANCE_HZ){
      Serial.println("Hit by Player 5!");
      ws.textAll("HIT_BY_P5");
    }
    else{
      Serial.println("Hit by Unknown Frequency!");
      ws.textAll("HIT_UNKNOWN");
    }
    frequencyReady = false;
    // ---- END PCNT FREQUENCY CHECK ----
    if(now - lastStatusUpdate >= STATUS_UPDATE_INTERVAL){
      sendStatusUpdate();
      lastStatusUpdate = now;
    }
    //Laser stop logic
    if(now - lastLaserFiredTime > laserFireDuration && lastLaserFiredTime != 0){
      stopLaserSignal();
      lastLaserFiredTime = 0;
    }
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

void initPCNTFrequencyReader() {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = PCNT_INPUT_SIG_IO,
    .ctrl_gpio_num = PCNT_EVT_CTRL_IO,
    .lctrl_mode = PCNT_MODE_REVERSE,  //No control, just set to reverse
    .hctrl_mode = PCNT_MODE_KEEP,  // Control mode not used
    .pos_mode = PCNT_COUNT_INC,   // Count on rising edge
    .neg_mode = PCNT_COUNT_DIS,   // Do not count on falling edge
    .counter_h_lim = 0,
    .counter_l_lim = 0,
    .unit = PCNT_HIT_UNIT,
    .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config);
  pcnt_filter_enable(PCNT_UNIT_HIT);
  pcnt_set_filter_value(PCNT_UNIT_HIT, 100); // Filter out pulses shorter than 100us
  pcnt_counter_pause(PCNT_UNIT_HIT);
  pcnt_counter_clear(PCNT_UNIT_HIT);
  Serial.println("PCNT initialized for GPIO 34");

  //Configure timer for measurement gate
  timer_config_t config = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = TIMER_DIVIDER
  };
  timer_init(TIMER_GROUP, TIMER_IDX, &config);
  timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0);
  //Set alarm to trigger every 100ms
  timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, TIMER_INTERVAL_SEC * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP, TIMER_IDX);
  timer_isr_register(TIMER_GROUP, TIMER_IDX, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
}

// ===== INTERRUPTS =====
void IRAM_ATTR hitDetected(){
  isHit = true;
  hitTime = millis();
}

//ISR to read the count
void IRAM_ATTR timer_isr(void *arg){
  //Clear the interrupt
  TIMERG0.int_clr_timers.t0 = 1;

  //Stop and read PCNT
  pcnt_counter_pause(PCNT_HIT_UNIT);
  pcnt_get_counter_value(PCNT_HIT_UNIT, &pulseCount);
  pcnt_counter_clear(PCNT_HIT_UNIT);
  frequencyReady = true;

  //Re-enable alarm
  TIMERG0.hw_timer[TIMER_IDX].config.alarm_en = TIMER_ALARM_EN;
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

void startLaserSginal(){
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
  Serial.println("FIRE LASER");
  lastLaserFiredTime = millis();
  if(laserFireDuration > millis() - lastLaserFiredTime){
    startLaserSginal();
    while(LaserFireDuration < millis() - lastLaserFiredTime){
      // Wait for duration
    }
    stopLaserSignal();
    ws.textAll("LASER_FIRED");
  }
  else{
    return;
  }
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

void whoHitMe() {
  frequencyReady = false;
  pulseCount = 0;
  //Start the PCNT counter and the timer
  pcnt_counter_resume(PCNT_HIT_UNIT);
  timer_start(TIMER_GROUP, TIMER_IDX);
  Serial.println("PCNT measurement started...");
}

void sendStatusUpdate() {
  String msg = "STATUS:{\"clients\":" + String(ws.count()) + "}";
  ws.textAll(msg);
}
