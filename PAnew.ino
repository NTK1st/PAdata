// ESP32 — publish sensors to HiveMQ Cloud and subscribe for LED/NeoPixel control (MQTTS/TLS)
// Animation: 4 Pixels for Temp Status (color) + 4 Pixels for Rain Status (blue) + 4 Pixels (White) running circularly in all 12 pixels.
// Added: Servo Control on Pin 13 (60 <-> 180)

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h> // [NEW] เพิ่มไลบรารี Servo

// ==================== CONFIG ====================
const char* ssid = "saming loar tae";
const char* password = "Jakrapathps1234";

const char* mqttBroker = "9537a19f731146f885f64fdc978e77d2.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "NTK1st";
const char* mqttPassword = "Team15437%";

const char* mqttSensorTopic = "home/sensors";
const char* mqttNeoPixelSetTopic = "home/neopixel/set";
const char* mqttNeoPixelBrightness = "home/neopixel/brightness";
const char* mqttNeoPixelStatus = "home/neopixel/status";
const char* mqttServoTopic = "home/servo/set";     // [NEW] Topic รับคำสั่ง Servo
const char* mqttServoStatus = "home/servo/status"; // [NEW] Topic ส่งสถานะ Servo

// ==================== HARDWARE ====================
#define DHTPIN 33
#define DHTTYPE DHT11
#define RAIN_PIN 34
#define LDR_PIN 32
#define LED_PIN 18 
#define NEOPIXEL_PIN 23
#define NUM_PIXELS 12 
#define BUZZER_PIN 25 
#define SERVO_PIN 13 // [NEW] ใช้ GPIO 13 สำหรับ Servo

DHT dht(DHTPIN, DHTTYPE);
WiFiClientSecure espClient;
PubSubClient client(espClient);
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Servo myServo; // [NEW] สร้างตัวแปร Servo

// Timing
unsigned long lastSend = 0;
const unsigned long sendInterval = 10000; // 10 seconds

// Buzzer Timing and State
unsigned long lastBuzzerCycleTime = 0;
const unsigned long buzzerCycleDuration = 5000; 
const int MAX_BUZZER_COUNT = 4; 

const int melody[] = {
  392, 440, 494, 523, 
  494, 440, 392, 262  
};
const int noteDurations[] = {
  250, 250, 250, 500, 
  250, 250, 250, 750
};
const int melodyLength = sizeof(melody) / sizeof(noteDurations[0]);

// Buzzer State Variables
bool isBuzzerPlaying = false; 
int buzzerCount = 0; 
int rainAlertStatus = 0; 

// ตัวแปรสำหรับเล่นเพลง
int currentNote = 0;
unsigned long noteStartTime = 0;
const int PAUSE_BETWEEN_NOTES = 50; 

// Modes
bool autoStatusMode = true;
bool isManualOverride = false;
int pixelBrightness = 120;
int currentServoAngle = 60; // [NEW] เก็บค่ามุมปัจจุบัน

// Thresholds
const float TEMP_LOW_MAX = 29.0;
const float TEMP_NORMAL_MAX = 35.0;
const int RAIN_THRESHOLD = 800; 
const int RAIN_CLEAR_THRESHOLD = 300; 

bool isRainAnimationRunning = false;


// ==================== Buzzer Helper ====================
void handleBuzzerSound() {
  if (rainAlertStatus != 1 || buzzerCount >= MAX_BUZZER_COUNT) {
    if (isBuzzerPlaying) {
      noTone(BUZZER_PIN); 
      isBuzzerPlaying = false;
    }
    return;
  }
  
  unsigned long currentTime = millis();

  if (!isBuzzerPlaying && (currentTime - lastBuzzerCycleTime >= buzzerCycleDuration || buzzerCount == 0)) {
    if (buzzerCount > 0) {
      buzzerCount++; 
    } else {
      buzzerCount = 1; 
    }
    
    if (buzzerCount > MAX_BUZZER_COUNT) {
        rainAlertStatus = 2; 
        Serial.println("RAIN ALERT FINISHED (3 cycles done).");
        return;
    }

    lastBuzzerCycleTime = currentTime; 
    isBuzzerPlaying = true;
    currentNote = 0; 
    noteStartTime = currentTime; 
    Serial.print("Starting Buzzer Cycle ");
    Serial.println(buzzerCount);
    
    if (currentNote < melodyLength) {
        tone(BUZZER_PIN, melody[currentNote], noteDurations[currentNote]);
    }
    return;
  }
  
  if (isBuzzerPlaying) {
    if (currentNote < melodyLength && (currentTime - noteStartTime) >= (noteDurations[currentNote] + PAUSE_BETWEEN_NOTES)) {
      
      currentNote++;
      noteStartTime = currentTime; 
      
      if (currentNote >= melodyLength) {
        noTone(BUZZER_PIN);
        isBuzzerPlaying = false; 
        Serial.println("Melody segment finished. Waiting for next cycle.");
        return;
      }
      tone(BUZZER_PIN, melody[currentNote], noteDurations[currentNote]);
    }
  } 
}


// ==================== Helper ====================
void publishStatus(String colorStr = "") {
  String modeStr = (autoStatusMode && !isManualOverride) ? "AUTO" : "MANUAL";
  String json = "{";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"brightness\":" + String(pixelBrightness) + ",";

  if (colorStr != "") json += "\"color\":\"" + colorStr + "\"";
  else json += "\"color\":\"N/A\"";

  json += "}";

  client.publish(mqttNeoPixelStatus, json.c_str());
}

// [NEW] ฟังก์ชันส่งสถานะ Servo กลับไป MQTT
void publishServoStatus(int angle) {
    currentServoAngle = angle;
    // สร้าง JSON: {"angle": 180, "status": "OPEN"}
    String statusStr = (angle >= 170) ? "OPEN" : "CLOSE"; 
    String json = "{";
    json += "\"angle\":" + String(angle) + ",";
    json += "\"status\":\"" + statusStr + "\"";
    json += "}";
    
    client.publish(mqttServoStatus, json.c_str());
    Serial.print("Published Servo Status: ");
    Serial.println(json);
}

void setAll(uint32_t c) {
  strip.fill(c, 0, NUM_PIXELS);
  strip.show();
}

void handleRainAnimation(uint32_t tempColor) {
  static unsigned long lastAnimationTime = 0;
  const int ANIMATION_SPEED = 70; 
  const int TOTAL_PIXELS = NUM_PIXELS; 
  const int GROUP_SIZE = 4;     
  static uint32_t currentBuffer[TOTAL_PIXELS]; 
  static bool isBufferInitialized = false;
  uint32_t rainColor = strip.Color(0,0,255); 
  uint32_t whiteColor = strip.Color(255,255,255); 

  if (!isBufferInitialized) {
      for(int i = 0; i < GROUP_SIZE; i++) {
          currentBuffer[i] = tempColor;
      }
      for(int i = GROUP_SIZE; i < GROUP_SIZE * 2; i++) {
          currentBuffer[i] = rainColor;
      }
      for(int i = GROUP_SIZE * 2; i < TOTAL_PIXELS; i++) {
          currentBuffer[i] = whiteColor; 
      }
      isBufferInitialized = true;
  }
  
  if (millis() - lastAnimationTime >= ANIMATION_SPEED) {
    for(int i = 0; i < TOTAL_PIXELS; i++) {
        if (currentBuffer[i] != rainColor && currentBuffer[i] != whiteColor) {
             currentBuffer[i] = tempColor;
        }
    }

    uint32_t lastColor = currentBuffer[TOTAL_PIXELS - 1]; 
    for (int i = TOTAL_PIXELS - 1; i > 0; i--) {
        currentBuffer[i] = currentBuffer[i - 1]; 
    }
    currentBuffer[0] = lastColor; 
    
    for(int i = 0; i < TOTAL_PIXELS; i++) {
        strip.setPixelColor(i, currentBuffer[i]);
    }
    strip.show();
    lastAnimationTime = millis();
    client.loop(); 
  }
}

// ==================== MQTT CALLBACK ====================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on [");
  Serial.print(topic);
  Serial.print("] Msg: ");

  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim(); // [NEW] ตัดช่องว่างหัวท้ายออก
  Serial.println(msg);

  // [NEW] ส่วนควบคุม Servo
  if (String(topic) == mqttServoTopic) {
      // สั่งเปิด (180 องศา)
      if (msg == "180" || msg.equalsIgnoreCase("OPEN") || msg.equalsIgnoreCase("ON")) {
          myServo.write(180);
          publishServoStatus(180);
          Serial.println("Servo moved to 180 (OPEN)");
      } 
      // สั่งปิด (60 องศา)
      else if (msg == "60" || msg.equalsIgnoreCase("CLOSE") || msg.equalsIgnoreCase("OFF") || msg == "0") {
          myServo.write(60);
          publishServoStatus(60);
          Serial.println("Servo moved to 60 (CLOSE)");
      }
      else {
          Serial.println("Invalid Servo Command");
      }
      return; 
  }

  if (String(topic) == mqttNeoPixelBrightness) {
    pixelBrightness = constrain(msg.toInt(), 0, 255);
    strip.setBrightness(pixelBrightness);
    strip.show();
    Serial.print("Brightness set to: ");
    Serial.println(pixelBrightness);
    publishStatus();
    return;
  }

  if (String(topic) == mqttNeoPixelSetTopic) {
    if (msg == "AUTO" || msg == "ON") {
      Serial.println("Switched to AUTO Mode (Temp/Rain controlled)");
      autoStatusMode = true;
      isManualOverride = false;
      strip.fill(strip.Color(0,0,0), 0, NUM_PIXELS);
      strip.show();
      isRainAnimationRunning = false; 
      publishStatus("AUTO");
      return;
    }

    autoStatusMode = false;
    isManualOverride = true;
    isRainAnimationRunning = false; 
    
    strip.fill(strip.Color(0,0,0), 0, NUM_PIXELS);
    strip.show(); 
    Serial.println("Manual Override Active.");

    if (msg == "OFF") {
      setAll(strip.Color(0,0,0));
      Serial.println("NeoPixels turned OFF.");
      publishStatus("0,0,0");
    }
    else {
      int r,g,b;
      if (sscanf(msg.c_str(), "%d,%d,%d", &r,&g,&b)==3) {
        setAll(strip.Color(r,g,b));
        String colorStr = String(r) + "," + String(g) + "," + String(b);
        Serial.print("NeoPixels set to RGB: ");
        Serial.println(colorStr);
        publishStatus(colorStr);
      } else {
        Serial.println("Invalid NeoPixel color command format.");
      }
    }
    return;
  }
}

// ==================== MQTT CONNECT ====================
void reconnectMQTT() {
  Serial.print("Connecting to MQTT broker...");
  while (!client.connected()) {
    espClient.setInsecure(); // Required for HiveMQ Cloud TLS/MQTTS
    String id = "ESP-" + String((uint32_t)esp_random(),16);

    if (client.connect(id.c_str(), mqttUser, mqttPassword)) {
      Serial.println("Connected.");
      client.subscribe(mqttNeoPixelSetTopic);
      client.subscribe(mqttNeoPixelBrightness);
      client.subscribe(mqttServoTopic); // [NEW] Subscribe Topic Servo
      
      Serial.print("   Subscribed to: "); Serial.println(mqttNeoPixelSetTopic);
      Serial.print("   Subscribed to: "); Serial.println(mqttServoTopic);
      
      publishStatus();
      publishServoStatus(currentServoAngle); // รายงานสถานะ Servo เริ่มต้น
    }
    else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 2 seconds...");
      delay(2000);
    }
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP32 Sensor Dashboard Node ---");
  
  dht.begin();
  pinMode(RAIN_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); 

  // [NEW] Servo Config
  myServo.setPeriodHertz(50); 
  myServo.attach(SERVO_PIN, 500, 2400); 
  myServo.write(60); // เริ่มต้นที่ 60 องศา (ปิด)
  Serial.println("Servo initialized at 60 degrees on Pin 13");

  strip.begin();
  strip.setBrightness(pixelBrightness);
  strip.show();
  setAll(strip.Color(20,20,20)); 
  Serial.print("NeoPixel initial brightness: "); Serial.println(pixelBrightness);

  // WiFi Connect
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // MQTT Connect
  client.setServer(mqttBroker, mqttPort);
  client.setCallback(callback);
  reconnectMQTT();
}

// ==================== LOOP ====================
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // ตรวจสอบและเล่นเสียง Buzzer (Non-blocking)
  handleBuzzerSound();
  
  // ถ้าอยู่ในโหมด AUTO และมีการตรวจพบฝน ให้เรียกใช้ Animation ตลอดเวลา (Non-blocking)
  if (isRainAnimationRunning && autoStatusMode && !isManualOverride) {
      // ต้องคำนวณ tempColor ใหม่ก่อนเรียกใช้
      float t = dht.readTemperature(); 
      uint32_t tempColor;
      if (t <= TEMP_LOW_MAX) tempColor = strip.Color(0,255,0); 
      else if (t <= TEMP_NORMAL_MAX) tempColor = strip.Color(255,160,0);
      else tempColor = strip.Color(255,0,0);
      
      handleRainAnimation(tempColor);
  }


  unsigned long now = millis();
  if (now - lastSend >= sendInterval) {
    lastSend = now;
    
    // Reading Sensors
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    int rain = analogRead(RAIN_PIN);
    int light = analogRead(LDR_PIN);
    
    Serial.println("\n--- Sensor Reading & Publish ---");
    Serial.print("Temp/Hum: "); Serial.print(t); Serial.print("°C / "); Serial.print(h); Serial.println("%");
    Serial.print("Rain/Light (Analog): "); Serial.print(rain); Serial.print(" / "); Serial.println(light);

    // Check for valid readings before publishing
    if (isnan(t) || isnan(h)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // ตรรกะการจัดการสถานะฝนตก (Rain Alert State Machine)
    if (rain < RAIN_CLEAR_THRESHOLD) {
      rainAlertStatus = 0;
      buzzerCount = 0;
      currentNote = 0; 
      noTone(BUZZER_PIN); 
      isRainAnimationRunning = false; 
      
      uint32_t tempColor;
      if (t <= TEMP_LOW_MAX) tempColor = strip.Color(0,255,0); 
      else if (t <= TEMP_NORMAL_MAX) tempColor = strip.Color(255,160,0);
      else tempColor = strip.Color(255,0,0);
      
      strip.fill(tempColor, 0, NUM_PIXELS); 
      strip.show();

      Serial.println("Rain Cleared (<300). Alert status reset.");
    } 
    else if (rain > RAIN_THRESHOLD && rainAlertStatus == 0) {
      rainAlertStatus = 1;
      buzzerCount = 0; 
      currentNote = 0; 
      lastBuzzerCycleTime = now;
      noteStartTime = now; 
      isRainAnimationRunning = true; 
      Serial.println("Heavy Rain Detected (>800). Starting 3-cycle alert.");
    }

    // Publish sensors
    String json = "{";
    json += "\"temperature\":" + String(t,1) + ",";
    json += "\"humidity\":" + String(h,1) + ",";
    json += "\"rain\":" + String(rain) + ",";
    json += "\"light\":" + String(light);
    json += "}";
    
    if (client.publish(mqttSensorTopic, json.c_str())) {
      Serial.print("Published Sensor Data: ");
      Serial.println(json);
    } else {
      Serial.println("MQTT publish sensor data failed.");
    }
    
    // Auto mode control (NeoPixel)
    if (autoStatusMode && !isManualOverride) {
      uint32_t tempColor;
      String colorStatusStr;

      if (t <= TEMP_LOW_MAX) {
        tempColor = strip.Color(0,255,0); // Green
        colorStatusStr = "Green (Low)";
      }
      else if (t <= TEMP_NORMAL_MAX) {
        tempColor = strip.Color(255,160,0); // Orange/Yellow
        colorStatusStr = "Orange (Normal)";
      }
      else {
        tempColor = strip.Color(255,0,0); // Red
        colorStatusStr = "Red (High)";
      }

      Serial.print("Auto Mode Temp Status: "); Serial.println(colorStatusStr);

      if (rain > RAIN_THRESHOLD) {
        Serial.println("RAIN DETECTED. Animation controlled in loop().");
        isRainAnimationRunning = true;
      } 
      else {
        Serial.println("No Rain. Setting solid color on 12 pixels.");
        isRainAnimationRunning = false;
        strip.fill(tempColor, 0, NUM_PIXELS); 
        strip.show();
      }

      publishStatus("AUTO");
    } else {
      if (rainAlertStatus != 0) {
          rainAlertStatus = 0;
          buzzerCount = 0;
          currentNote = 0; 
          noTone(BUZZER_PIN); 
      }
      isRainAnimationRunning = false; 
      Serial.println("MANUAL Mode Active. Skipping Auto Color Update.");
    }
  }
}