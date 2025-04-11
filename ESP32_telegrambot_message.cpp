// Multiple wifi option and multiple alert message receiver (4 of receivers) for esp32 to connect

#include <WiFi.h>
#include <HTTPClient.h>

// Wi-Fi credentials for two networks
const char* ssid1 = "Redmi 10 Prime";          // Replace with your first Wi-Fi SSID
const char* password1 = "…………….."; // Replace with your first Wi-Fi password

const char* ssid2 = "AirFiber-DashFamily";     // Replace with your second Wi-Fi SSID
const char* password2 = "………………"; // Replace with your second Wi-Fi password

// Telegram bot details
const char* botToken = "…………………………… ";       // Replace with your bot token

// Array of chat IDs
const char* chatIDs[] = {
  "5405886382",  // Abinash Dash
  "5793457689",  // SR Singh
  "7010111569",  // Sanket Choudhury
  "6639258835"   // Debadatta Satapathy
};
const int numChatIDs = 4;  // Number of chat IDs

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  if (!connectToWiFi(ssid1, password1)) {
    Serial.println("Failed to connect to first network. Trying second network...");
    if (!connectToWiFi(ssid2, password2)) {
      Serial.println("Failed to connect to both networks. Please check credentials or signal strength.");
      return;
    }
  }

  Serial.println("Connected to Wi-Fi");
  Serial.println("IP Address: " + WiFi.localIP().toString());

  // Send a message via Telegram bot to all chat IDs
  for (int i = 0; i < numChatIDs; i++) {
    sendTelegramMessage(chatIDs[i], "Hello, this is a text from esp32");
  }
}

void loop() {
  // Nothing to do here
}

bool connectToWiFi(const char* ssid, const char* password) {
  Serial.println("Connecting to Wi-Fi: " + String(ssid));
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Attempting to connect... Attempt: " + String(++attempts));
    if (attempts > 10) {
      Serial.println("Failed to connect to Wi-Fi: " + String(ssid));
      return false;
    }
  }

  Serial.println("Connected to Wi-Fi: " + String(ssid));
  return true;
}

void sendTelegramMessage(const char* chatID, String message) {
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(botToken) + "/sendMessage?chat_id=" + String(chatID) + "&text=" + message;

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    Serial.println("Message sent successfully to chat ID: " + String(chatID));
  } else {
    Serial.println("Failed to send message to chat ID: " + String(chatID) + ". HTTP Code: " + String(httpCode));
    String response = http.getString();
    Serial.println("Response: " + response);
  }
  http.end();
}
