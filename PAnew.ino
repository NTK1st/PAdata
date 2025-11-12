// ESP32 — publish sensors to HiveMQ Cloud and subscribe for LED control (MQTTS/TLS)

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <WiFiClientSecure.h> // **สำคัญ: สำหรับ HiveMQ Cloud (MQTTS)**

// ========== WiFi ==========
const char* ssid = "UDD6544_2.4G";
const char* password = "11249965";

// ========== MQTT (HiveMQ Cloud) ==========
const char* mqttBroker = "66c46947755f4602a6bbefe938335380.s1.eu.hivemq.cloud";
const int mqttPort = 8883; // **เปลี่ยนเป็น 8883 สำหรับ MQTTS/TLS**
const char* mqttUser = "NTK1st";
const char* mqttPassword = "Team15437%";
const char* mqttTopic = "home/sensors";
const char* mqttSetTopic = "home/led/set";

// ========== Pins ==========
#define DHTPIN 33
#define DHTTYPE DHT11
#define RAIN_PIN 34
#define LDR_PIN 32
#define LED_PIN 18

DHT dht(DHTPIN, DHTTYPE);

// WiFiClient espClient; // (ใช้ไม่ได้กับ MQTTS)
WiFiClientSecure espClient; // **ใช้ Secure Client สำหรับ 8883**
PubSubClient client(espClient);

unsigned long lastSend = 0;
const unsigned long sendInterval = 10000; // send every 10s

// ====================================================================
// ฟังก์ชัน CallBack: ถูกเรียกเมื่อได้รับข้อความจาก MQTT Broker
// ====================================================================
void callback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(msg);

    // ตรวจสอบ Topic ควบคุม LED
    if (String(topic) == mqttSetTopic) {
        if (msg.equalsIgnoreCase("ON")) {
            digitalWrite(LED_PIN, HIGH);
            Serial.println("💡 LED turned ON");
        } else if (msg.equalsIgnoreCase("OFF")) {
            digitalWrite(LED_PIN, LOW);
            Serial.println("⚫ LED turned OFF");
        }
    }
}

// ====================================================================
// ฟังก์ชัน Reconnect: จัดการการเชื่อมต่อ MQTT
// ====================================================================
void reconnectMQTT() {
    // โค้ดนี้จะถูกเรียกซ้ำๆ จนกว่าจะเชื่อมต่อสำเร็จ
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        
        // **สำคัญ: ตั้งค่า Insecure เพื่อให้ PubSubClient ข้ามการตรวจสอบ SSL Certificate**
        espClient.setInsecure(); 
        
        String clientId = "ESP32Client-" + String((uint32_t)esp_random(), 16);
        
        // พยายามเชื่อมต่อด้วย Username และ Password
        if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
            Serial.println("connected! ✅");
            // Subscribe เพื่อรับคำสั่ง LED
            client.subscribe(mqttSetTopic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2s ❌");
            delay(2000);
        }
    }
}

// ====================================================================
// Setup: เริ่มต้นการทำงาน
// ====================================================================
void setup() {
    Serial.begin(115200);
    dht.begin();
    pinMode(RAIN_PIN, INPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // 1. เชื่อมต่อ WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());

    // 2. ตั้งค่า MQTT Server และ Callback
    client.setServer(mqttBroker, mqttPort);
    client.setCallback(callback);
    
    // 3. เชื่อมต่อ MQTT ครั้งแรก
    reconnectMQTT();
}

// ====================================================================
// Loop: รันต่อเนื่อง
// ====================================================================
void loop() {
    if (!client.connected()) reconnectMQTT();
    client.loop(); // ต้องเรียก client.loop() เพื่อจัดการการรับส่งข้อความ

    unsigned long now = millis();
    if (now - lastSend >= sendInterval) {
        lastSend = now;

        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        int rain = analogRead(RAIN_PIN);
        int light = analogRead(LDR_PIN);

        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("Failed to read from DHT sensor ⚠️");
        } else {
            // ควบคุม LED อัตโนมัติ (สมมติฐาน: เปิดไฟเมื่อมืด)
            bool ledState = light < 500; // threshold — ปรับค่าความมืดเอง
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);

            // Build JSON payload
            String payload = "{";
            payload += "\"temperature\":" + String(temperature, 1) + ",";
            payload += "\"humidity\":" + String(humidity, 1) + ",";
            payload += "\"rain\":" + String(rain) + ",";
            payload += "\"light\":" + String(light) + ",";
            payload += "\"led\":\"" + String(ledState ? "ON" : "OFF") + "\"";
            payload += "}";

            Serial.println("Publishing: " + payload);
            boolean ok = client.publish(mqttTopic, payload.c_str());
            if (!ok) Serial.println("Publish failed ❌");
        }
    }

    delay(200);
}