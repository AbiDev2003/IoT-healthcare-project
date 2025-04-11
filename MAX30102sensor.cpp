#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// LCD at 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 particleSensor;

uint32_t irBuffer[100];  // Stores IR values
uint32_t redBuffer[100]; // Stores Red values
int32_t spo2;            // SpO2 value
int32_t heartRate;       // Heart Rate
int8_t isSpO2Valid = 0, isHRValid = 0;

// Alert thresholds
const int HIGH_HR_THRESHOLD = 100;
const int LOW_HR_THRESHOLD = 60;
const int HIGH_SPO2_THRESHOLD = 100;
const int LOW_SPO2_THRESHOLD = 95;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing MAX30102...");

  Wire.begin();

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Initialize MAX30102 Sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Check wiring!");
    lcd.setCursor(0, 1);
    lcd.print("Sensor Error!");
    while (1);
  }

  Serial.println("MAX30102 found!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor Ready");

  particleSensor.setup();
  delay(1000);
}

void loop() {
  // Collect 100 samples
  for (int i = 0; i < 100; i++) {
    irBuffer[i] = particleSensor.getIR();
    redBuffer[i] = particleSensor.getRed();
    delay(10);  // Sampling delay
  }

  // Calculate SpO2 & Heart Rate
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &heartRate, &isHRValid, &spo2, &isSpO2Valid);

  // Display on Serial Monitor
  Serial.print("SpO2: ");
  Serial.print(spo2);
  Serial.print("% | HR: ");
  Serial.print(heartRate);

  // Check heart rate conditions
  String hrStatus = "Normal";
  if (heartRate > HIGH_HR_THRESHOLD) {
    hrStatus = "High HR!";
    Serial.print(" (High)");
  } else if (heartRate < LOW_HR_THRESHOLD) {
    hrStatus = "Low HR!";
    Serial.print(" (Low)");
  }

  // Check SpO2 conditions
  String spo2Status = "Normal";
  if (spo2 > HIGH_SPO2_THRESHOLD) {
    spo2Status = "High SpO2!";
    Serial.print(" (High)");
  } else if (spo2 < LOW_SPO2_THRESHOLD) {
    spo2Status = "Low SpO2!";
    Serial.print(" (Low)");
  }

  Serial.println();

  // Display on LCD
  lcd.clear();
  
  // First line: SpO2 with status
  lcd.setCursor(0, 0);
  lcd.print("SpO2:");
  lcd.print(spo2);
  lcd.print("% ");
  if (spo2 > HIGH_SPO2_THRESHOLD || spo2 < LOW_SPO2_THRESHOLD) {
    lcd.print(spo2Status);
  }

  // Second line: HR with status
  lcd.setCursor(0, 1);
  lcd.print("HR:");
  lcd.print(heartRate);
  lcd.print(" ");
  if (heartRate > HIGH_HR_THRESHOLD || heartRate < LOW_HR_THRESHOLD) {
    lcd.print(hrStatus);
  }

  delay(1000);
}