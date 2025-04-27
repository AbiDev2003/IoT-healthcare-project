////ESP32 CODE/////
#include <WiFi.h>
#include <HTTPClient.h>

String apiKey = "SKU6U8HKGWSBMI9E";  // Enter your Write API key from ThingSpeak
const char* ssid = "project";        // Replace with your Wi-Fi SSID
const char* pass = "12345678";       // Replace with your Wi-Fi password
const char* server = "api.thingspeak.com";

// IFTTT Settings
const char* iftttWebhookEvent = "esp32_health_alert";
const char* iftttWebhookKey = "YOUR_IFTTT_WEBHOOK_KEY"; // Replace with your IFTTT webhook key
const char* iftttWebhookURL = "https://maker.ifttt.com/trigger/%s/with/key/%s";

WiFiClient client;

// Variables to track previous values and alert states
int prevBPM = 0;
int prevSpO2 = 0;
int prevTemp = 0;
bool bpmAlertSent = false;
bool spo2AlertSent = false;
bool tempAlertSent = false;

void setup() {
  Serial.begin(9600);
  delay(10);

  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
}

void sendIFTTTAlert(String parameter, String value) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // Create the URL
    char url[256];
    snprintf(url, sizeof(url), iftttWebhookURL, iftttWebhookEvent, iftttWebhookKey);
    
    // Create the payload with Value1 (parameter) and Value2 (value)
    String payload = "{\"value1\":\"" + parameter + "\",\"value2\":\"" + value + "\"}";
    
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    
    int httpResponseCode = http.POST(payload);
    
    if (httpResponseCode > 0) {
      Serial.print("IFTTT webhook triggered successfully, response code: ");
      Serial.println(httpResponseCode);
      Serial.println("IFTTT will now send emails to all recipients configured in your applet");
    } else {
      Serial.print("Error triggering IFTTT webhook, error: ");
      Serial.println(httpResponseCode);
    }
    
    http.end();
  } else {
    Serial.println("WiFi not connected, cannot trigger IFTTT webhook");
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    if (data.indexOf('a') != -1 && data.length() > 1) {
      Serial.println(data);

      int indexA = data.indexOf("a") + 1;
      int indexB = data.indexOf("b") + 1;
      int indexC = data.indexOf("c") + 1;
      int indexD = data.indexOf("d") + 1;

      String valueA = data.substring(indexA, indexB - 1);
      String valueB = data.substring(indexB, indexC - 1);
      String valueC = data.substring(indexC, indexD - 1);

      Serial.println("Value a (BPM): " + valueA);
      Serial.println("Value b (SpO2): " + valueB);
      Serial.println("Value c (Temp): " + valueC);

      // Convert values to integers
      int currentBPM = valueA.toInt();
      int currentSpO2 = valueB.toInt();
      int currentTemp = valueC.toInt();

      // Check thresholds and send alerts if needed
      if ((currentBPM > 120 || currentBPM < 60) && !bpmAlertSent) {
        sendIFTTTAlert("Abnormal Heart Rate", String(currentBPM) + " BPM");
        bpmAlertSent = true;
      } else if (currentBPM >= 60 && currentBPM <= 120) {
        bpmAlertSent = false; // Reset alert flag if value returns to normal
      }

      if ((currentSpO2 > 120 || currentSpO2 < 60) && !spo2AlertSent) {
        sendIFTTTAlert("Abnormal SpO2 Level", String(currentSpO2) + "%");
        spo2AlertSent = true;
      } else if (currentSpO2 >= 60 && currentSpO2 <= 120) {
        spo2AlertSent = false; // Reset alert flag if value returns to normal
      }

      if (currentTemp > 34 && !tempAlertSent) {
        sendIFTTTAlert("High Temperature", String(currentTemp) + "°C");
        tempAlertSent = true;
      } else if (currentTemp <= 34) {
        tempAlertSent = false; // Reset alert flag if value returns to normal
      }

      if (client.connect(server, 80)) {
        String postStr = "api_key=" + apiKey;
        postStr += "&field1=" + valueA;
        postStr += "&field2=" + valueB;
        postStr += "&field3=" + valueC;

        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(postStr.length());
        client.print("\n\n");
        client.print(postStr);

        delay(1000);  // Allow time for the server to process the request
      }

      delay(15000);  // Minimum delay between updates for ThingSpeak
    }
  }
}
