# Embedded Systems Projects Collection

This repository contains various Arduino/ESP32 projects including sensor implementations (DS18B20, MAX30102) and processor testing codes (ESP32 mail/telegram alerts).

## Sensor Projects (DS18B20, MAX30102, etc.)

### Quick Start Guide:
1. Install required libraries in Arduino IDE
2. Connect Arduino and select:
   - Correct processor type
   - Appropriate COM port
3. For ESP32:
   - Add ESP32 board support (install from [ESP32 GitHub](https://github.com/espressif/arduino-esp32))
4. Write/paste the code in editor
5. Debug and upload

## ESP32 Telegram Alert System

### Setup Instructions:
1. Create a Telegram bot using [BotFather](https://t.me/botfather)
2. Securely store your Telegram API key
3. Configure in code:
   - WiFi SSID (your hotspot name)
   - WiFi password
   - Telegram bot token
4. Upload code and test

## ESP32 Mail Alert System

### IFTTT Method:
1. Create IFTTT applet with Webhooks trigger
2. Configure ESP32 to send HTTP requests to IFTTT webhook
3. Set up email action in IFTTT

### SMTP Server Method:
1. Configure email service credentials (Gmail, etc.)
2. Enable less secure apps or generate app password
3. Set SMTP server details in ESP32 code
4. Implement proper security measures

## ThingSpeak API Integration Guide

### Step 1: Channel Creation
1. Sign up/login at [ThingSpeak.com](https://thingspeak.com)
2. Click "New Channel"
3. Configure fields:
   - Field 1: `Temperature` (°C)
   - Field 2: `SpO₂` (%)
   - Field 3: `Pulse Rate` (BPM)
4. (Optional) Add description and metadata

### Step 2: API Keys
```plaintext
Channel ID:      Found in URL (e.g., 1234567)
Write API Key:   **************** (keep secret)
Read API Key:    **************** (for dashboards)
```
## Step 3: Complete ESP32 + ThingSpeak Integration Code

### Full Implementation Example
```cpp
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi Credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ThingSpeak Configuration
const char* server = "api.thingspeak.com";
const String apiKey = "YOUR_WRITE_API_KEY";
const long channelID = YOUR_CHANNEL_ID;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // Sample sensor values - replace with actual readings
    float temperature = 36.5;  // From DS18B20
    float spo2 = 98.0;         // From MAX30102
    int heartRate = 72;        // From MAX30102
    
    String url = "http://" + String(server) + "/update?api_key=" + apiKey +
                 "&field1=" + String(temperature) +
                 "&field2=" + String(spo2) +
                 "&field3=" + String(heartRate);
    
    http.begin(url);
    int httpCode = http.GET();
    
    if (httpCode > 0) {
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  delay(15000); // Wait 15 seconds between updates
}
```
---

# IoT-based Healthcare Monitoring System (Final Year Project)

## Project Overview
A real-time IoT system to monitor vital health parameters:
- Pulse rate (MAX30100/MAX30102)
- SpO₂ (MAX30100/MAX30102)
- Body temperature (DS18B20)

## Key Features
- **Accuracy**:
  - ±2% for body temperature
  - ±5% for SpO₂
  - ±8% for pulse rate
- **Real-time monitoring** via ThingSpeak dashboard
- **Alert system** with buzzer for abnormal readings
- **Data transmission** via ESP32 WiFi

## Technical Specifications
- **Hardware**: Arduino Uno, ESP32, MAX30100, DS18B20, IR sensor
- **Software**: Embedded C++, Arduino IDE
- **Cloud Integration**: ThingSpeak API
- **Version Control**: Git/GitHub (6 commits, 647 LOC)

## Setup Instructions
1. Assemble hardware as per circuit diagram
2. Configure ThingSpeak channel and API keys
3. Upload respective codes to Arduino/ESP32
4. Monitor data on ThingSpeak dashboard

---

## Author
**Abinash Dash**  
GitHub: [AbiDev2003](https://github.com/AbiDev2003)
