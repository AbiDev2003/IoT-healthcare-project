#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_PIN 22  // Connect Data pin of DS18B20 to Arduino Mega digital pin 22

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// Initialize the LCD (Check if your LCD is 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.backlight();

  sensors.begin();
  lcd.print("Welcome");
  delay(2000); // Display welcome message for 2 seconds
  lcd.clear();
}

void loop() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);  // Get temperature in Celsius

  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  if(temperature < 37.3){
    lcd.print("Normal Body Temp ");
  }
  else if (temperature >= 37.3 && temperature < 38.5) {
    lcd.print("Alert: Mild body Temp!");
  } else if(temperature >= 38.5) {
    lcd.print("High Body Temp ");
  }

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(2000);  // Update every 2 seconds
}
