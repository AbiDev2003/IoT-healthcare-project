/////arduino uno code//////
#include <MAX3010x.h>
#include "filters.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_LiquidCrystal.h>  // Use this for normal 16x2 LCD

// LCD Setup (RS, EN, D4, D5, D6, D7)
Adafruit_LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

#define ONE_WIRE_BUS 2    // Pin for Dallas Temperature Sensor
#define IR_SENSOR_PIN 3   // IR sensor pin for respiration rate
const int buzzerPin = 7;  // Relay for mobile charger

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int temperatureC;

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

int m_bpm = 0;
int m_spo2 = 0;
int count = 0;
void setup() {
  Serial.begin(9600);
  sensors.begin();
  Serial.println("Setup complete.");

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    while (1)
      ;
  }

  lcd.begin(16, 2);
  lcd.clear();

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(buzzerPin, LOW);  // buzzer off
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop() {
  // Read and display temperature
  sensors.requestTemperatures();
  temperatureC = sensors.getTempCByIndex(0);  // Read temperature in Celsius

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature:");
  lcd.setCursor(0, 1);
  lcd.print(temperatureC, 1);
  delay(2000);
  if (temperatureC > 34) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("high Temp!");
    delay(2000);
    buzzerAlert();
  }

  // ir detection
  int IR_value = digitalRead(IR_SENSOR_PIN);
  if (IR_value == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Person detected!");
    count = 1;
  }
  do {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SPo2 Measuring:");

    lcd.setCursor(0, 1);
    lcd.print("place finger");

    auto sample = sensor.readSample(1000);
    float current_value_red = sample.red;
    float current_value_ir = sample.ir;

    // Detect Finger using raw sensor value
    if (sample.red > kFingerThreshold) {
      if (millis() - finger_timestamp > kFingerCooldownMs) {
        finger_detected = true;
      }
    } else {
      // Reset values if the finger is removed
      differentiator.reset();
      averager_bpm.reset();
      averager_r.reset();
      averager_spo2.reset();
      low_pass_filter_red.reset();
      low_pass_filter_ir.reset();
      high_pass_filter.reset();
      stat_red.reset();
      stat_ir.reset();

      finger_detected = false;
      finger_timestamp = millis();
    }

    if (finger_detected) {
      current_value_red = low_pass_filter_red.process(current_value_red);
      current_value_ir = low_pass_filter_ir.process(current_value_ir);

      // Statistics for pulse oximetry
      stat_red.process(current_value_red);
      stat_ir.process(current_value_ir);

      // Heart beat detection using value for red LED
      float current_value = high_pass_filter.process(current_value_red);
      float current_diff = differentiator.process(current_value);

      // Valid values?
      if (!isnan(current_diff) && !isnan(last_diff)) {

        // Detect Heartbeat - Zero-Crossing
        if (last_diff > 0 && current_diff < 0) {
          crossed = true;
          crossed_time = millis();
        }

        if (current_diff > 0) {
          crossed = false;
        }

        // Detect Heartbeat - Falling Edge Threshold
        if (crossed && current_diff < kEdgeThreshold) {
          if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
            // Show Results
            int bpm = 60000 / (crossed_time - last_heartbeat);
            float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
            float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
            float r = rred / rir;
            float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

            if (bpm > 50 && bpm < 250) {
              // Average?
              if (kEnableAveraging) {
                int average_bpm = averager_bpm.process(bpm);
                int average_r = averager_r.process(r);
                int average_spo2 = averager_spo2.process(spo2);
                m_bpm = average_bpm;
                m_spo2 = average_spo2;

                // Show if enough samples have been collected
                if (averager_bpm.count() >= kSampleThreshold) {
                  Serial.print("Time (ms): ");
                  Serial.println(millis());
                  Serial.print("Heart Rate (avg, bpm): ");
                  Serial.println(average_bpm);
                  Serial.print("R-Value (avg): ");
                  Serial.println(average_r);
                  Serial.print("SpO2 (avg, %): ");
                  Serial.println(average_spo2);
                }
              } else {
                Serial.print("Time (ms): ");
                Serial.println(millis());
                Serial.print("Heart Rate (current, bpm): ");
                Serial.println(bpm);
                Serial.print("R-Value (current): ");
                Serial.println(r);
                Serial.print("SpO2 (current, %): ");
                Serial.println(spo2);
                m_bpm = bpm;
                m_spo2 = spo2;
                break;
              }
            }

            // Reset statistic
            stat_red.reset();
            stat_ir.reset();
          }

          crossed = false;
          last_heartbeat = crossed_time;
        }
      }

      last_diff = current_diff;
    }
  } while (count == 1);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM: ");
  lcd.print(m_bpm);
  lcd.setCursor(0, 1);
  lcd.print("spo2: ");
  lcd.print(m_spo2);
  delay(2000);
  if (m_bpm > 120 || m_bpm < 60) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("abnormal bpm!");
    delay(2000);
    buzzerAlert();
  }
  if (m_spo2 > 120 || m_spo2 < 60) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("abnormal spo2!");
    delay(2000);
    buzzerAlert();
  }
  String Rf = String("a") + String(m_bpm) + String("b") + String(m_spo2) + String("c") + String(temperatureC) + String("d");
  Serial.println(Rf);
  count = 0;
  delay(2000);
}
void buzzerAlert() {
  // Buzzer ON, OFF, ON, OFF pattern for alerts
  digitalWrite(buzzerPin, HIGH);  // Buzzer OFF
  delay(500);
  digitalWrite(buzzerPin, LOW);  // Buzzer ON (Active LOW)
  delay(500);
  digitalWrite(buzzerPin, HIGH);  // Buzzer OFF
  delay(500);
  digitalWrite(buzzerPin, LOW);  // Buzzer ON (Active LOW)
  delay(500);
}
   

//////filters code///////////
#ifndef FILTERS_H
#define FILTERS_H

/**
 * @brief Statistic block for min/nax/avg
 */
class MinMaxAvgStatistic {
  float min_;
  float max_;
  float sum_;
  int count_;
public:
  /**
   * @brief Initialize the Statistic block
   */
  MinMaxAvgStatistic() :
    min_(NAN),
    max_(NAN),
    sum_(0),
    count_(0){}

  /**
   * @brief Add value to the statistic
   */
  void process(float value) {  
    min_ = isnan(min_) ? value : min(min_, value);
    max_ = isnan(max_) ? value : max(max_, value);
    sum_ += value;
    count_++;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    min_ = NAN;
    max_ = NAN;
    sum_ = 0;
    count_ = 0;
  }

  /**
   * @brief Get Minimum
   * @return Minimum Value
   */
  float minimum() const {
    return min_;
  }

  /**
   * @brief Get Maximum
   * @return Maximum Value
   */
  float maximum() const {
    return max_;
  }

  /**
   * @brief Get Average
   * @return Average Value
   */
  float average() const {
    return sum_/count_;
  }
};

/**
 * @brief High Pass Filter 
 */
class HighPassFilter {
  const float kX;
  const float kA0;
  const float kA1;
  const float kB1;
  float last_filter_value_;
  float last_raw_value_;
public:
  /**
   * @brief Initialize the High Pass Filter
   * @param samples Number of samples until decay to 36.8 %
   * @remark Sample number is an RC time-constant equivalent
   */
  HighPassFilter(float samples) :
    kX(exp(-1/samples)),
    kA0((1+kX)/2),
    kA1(-kA0),
    kB1(kX),
    last_filter_value_(NAN),
    last_raw_value_(NAN){}

  /**
   * @brief Initialize the High Pass Filter
   * @param cutoff Cutoff frequency
   * @pram sampling_frequency Sampling frequency
   */
  HighPassFilter(float cutoff, float sampling_frequency) :
    HighPassFilter(sampling_frequency/(cutoff*2*PI)){}

  /**
   * @brief Applies the high pass filter
   */
  float process(float value) { 
    if(isnan(last_filter_value_) || isnan(last_raw_value_)) {
      last_filter_value_ = 0.0;
    }
    else {
      last_filter_value_ = 
        kA0 * value 
        + kA1 * last_raw_value_ 
        + kB1 * last_filter_value_;
    }
    
    last_raw_value_ = value;
    return last_filter_value_;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    last_raw_value_ = NAN;
    last_filter_value_ = NAN;
  }
};

/**
 * @brief Low Pass Filter 
 */
class LowPassFilter {
  const float kX;
  const float kA0;
  const float kB1;
  float last_value_;
public:
  /**
   * @brief Initialize the Low Pass Filter
   * @param samples Number of samples until decay to 36.8 %
   * @remark Sample number is an RC time-constant equivalent
   */
  LowPassFilter(float samples) :
    kX(exp(-1/samples)),
    kA0(1-kX),
    kB1(kX),
    last_value_(NAN){}

  /**
   * @brief Initialize the Low Pass Filter
   * @param cutoff Cutoff frequency
   * @pram sampling_frequency Sampling frequency
   */
  LowPassFilter(float cutoff, float sampling_frequency) :
    LowPassFilter(sampling_frequency/(cutoff*2*PI)){}

  /**
   * @brief Applies the low pass filter
   */
  float process(float value) {  
    if(isnan(last_value_)) {
      last_value_ = value;
    }
    else {  
      last_value_ = kA0 * value + kB1 * last_value_;
    }
    return last_value_;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    last_value_ = NAN;
  }
};

/**
 * @brief Differentiator
 */
class Differentiator {
  const float kSamplingFrequency;
  float last_value_;
public:
  /**
   * @brief Initializes the differentiator
   */
  Differentiator(float sampling_frequency) :
    kSamplingFrequency(sampling_frequency),
    last_value_(NAN){}

  /**
   * @brief Applies the differentiator
   */
  float process(float value) {  
      float diff = (value-last_value_)*kSamplingFrequency;
      last_value_ = value;
      return diff;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    last_value_ = NAN;
  }
};

/**
 * @brief MovingAverageFilter
 * @tparam buffer_size Number of samples to average over
 */
template<int kBufferSize> class MovingAverageFilter {
  int index_;
  int count_;
  float values_[kBufferSize];
public:
  /**
   * @brief Initalize moving average filter
   */
  MovingAverageFilter() :
    index_(0),
    count_(0){}

  /**
   * @brief Applies the moving average filter
   */
  float process(float value) {  
      // Add value
      values_[index_] = value;

      // Increase index and count
      index_ = (index_ + 1) % kBufferSize;
      if(count_ < kBufferSize) {
        count_++;  
      }

      // Calculate sum
      float sum = 0.0;
      for(int i = 0; i < count_; i++) {
          sum += values_[i];
      }

      // Calculate average
      return sum/count_;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    index_ = 0;
    count_ = 0;
  }

  /**
   * @brief Get number of samples
   * @return Number of stored samples
   */
  int count() const {
    return count_;
  }
};

#endif // FILTERS_H
   

////ESP32 CODE/////

#include <WiFi.h>
String apiKey = "SKU6U8HKGWSBMI9E";  // Enter your Write API key from ThingSpeak
const char* ssid = "project";        // Replace with your Wi-Fi SSID
const char* pass = "12345678";       // Replace with your Wi-Fi password
const char* server = "api.thingspeak.com";

WiFiClient client;

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

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    if (data.indexOf('a') != -1 && data.length() > 1) {
      Serial.println(data);

      int indexA = data.indexOf("a") + 1;
      int indexB = data.indexOf("b") + 1;
      int indexC = data.indexOf("c") + 1;
      int indexD = data.indexOf("d") + 1;
      int indexE = data.indexOf("e") + 1;
      int indexF = data.indexOf("f") + 1;
      int indexG = data.indexOf("g") + 1;
      int indexH = data.indexOf("h") + 1;
      int indexI = data.indexOf("i") + 1;

      String valueA = data.substring(indexA, indexB - 1);
      String valueB = data.substring(indexB, indexC - 1);
      String valueC = data.substring(indexC, indexD - 1);
      String valueD = data.substring(indexD, indexE - 1);
      String valueE = data.substring(indexE, indexF - 1);
      String valueF = data.substring(indexF, indexG - 1);
      String valueG = data.substring(indexG, indexH - 1);
      String valueH = data.substring(indexH, indexI - 1);
      String valueI = data.substring(indexI);  // Assuming it goes to the end of the string

      Serial.println("Value a: " + valueA);
      Serial.println("Value b: " + valueB);
      Serial.println("Value c: " + valueC);
      Serial.println("Value d: " + valueD);
      Serial.println("Value e: " + valueE);
      Serial.println("Value f: " + valueF);
      Serial.println("Value g: " + valueG);
      Serial.println("Value h: " + valueH);
      Serial.println("Value i: " + valueI);

      if (client.connect(server, 80)) {
        String postStr = "api_key=" + apiKey;
        // String postStr = apiKey;
        postStr += "&field1=" + valueA;
        postStr += "&field2=" + valueB;
        postStr += "&field3=" + valueC;
        postStr += "&field4=" + valueD;
        postStr += "&field5=" + valueE;
        postStr += "&field6=" + valueF;
        postStr += "&field7=" + valueG;
        postStr += "&field8=" + valueI;

        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        // client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(postStr.length());
        client.print("\n\n");
        client.print(postStr);
        // client.stop();

        // Thingspeak requires a minimum 15-second delay between updates
        delay(1000);  // Allow time for the server to process the request

        //   if (client.connected()) {
        //     Serial.println("Data sent to ThingSpeak successfully");
        //     client.stop();
        //   } else {
        //     Serial.println("Failed to send data to ThingSpeak");
        //   }
      }
      // else {
      //   Serial.println("Failed to connect to ThingSpeak server");
      // }

      delay(15000);  // Minimum delay between updates for ThingSpeak
    }
  }
}