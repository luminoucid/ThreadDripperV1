#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_VEML6070.h>
#include <Makerblog_TSL45315.h>
#include <Adafruit_HDC1000.h>
#include <L298N.h> 

// === LCD Setup ===
LiquidCrystal_I2C lcd(0x26, 20, 4);  // I2C address 0x26, 20 columns, 4 rows

// === Sensor Objects ===
Adafruit_VEML6070 uvSensor;
Makerblog_TSL45315 lightSensor(TSL45315_TIME_M4); // 400ms integration time
Adafruit_HDC1000 hdcSensor = Adafruit_HDC1000();  // Temperature and humidity sensor

// === Sensor Values ===
int waterLevel = 0;
int soilMoisture = 0;
uint16_t uvIndex = 0;
uint32_t lightLux = 0;
float temperature = 0.0;
float humidity = 0.0;

// === Motor Pins ===
const unsigned int MOTOR_IN1 = 7;
const unsigned int MOTOR_IN2 = 8;
L298N motor(MOTOR_IN1, MOTOR_IN2);  // Motor object using L298N

// === Condition Thresholds (Configuration Section) ===
const float TEMP_THRESHOLD = 15.0;         // Minimum temperature
const float HUMIDITY_MAX = 50.0;           // Maximum humidity
const uint16_t UV_THRESHOLD = 0;         // Minimum UV index
const uint32_t LIGHT_THRESHOLD = 100;      // Minimum light level
const int SOIL_MOISTURE_MAX = 100;         // Maximum soil moisture
const int WATER_LEVEL_MIN = 500;           // Minimum water tank level

// === Motor Runtime Configuration ===
const unsigned long MOTOR_RUN_TIME_MS = 500; // Motor runs for 0.5 seconds

// === Internal motor state tracking ===
bool motorIsRunning = false;
unsigned long motorStartTime = 0;

// === Check all environmental conditions ===
bool all_conditions_met(float temp, float hum, uint16_t uv, uint32_t lux, int soil, int water) {
  return (temp > TEMP_THRESHOLD &&
          hum < HUMIDITY_MAX &&
          uv > UV_THRESHOLD &&
          lux > LIGHT_THRESHOLD &&
          soil < SOIL_MOISTURE_MAX &&
          water > WATER_LEVEL_MIN);
}

// === Control motor based on condition ===
void control_motor(bool shouldRun) {
  if (shouldRun) {
    motor.forward();                         // (Re)start motor
    motorStartTime = millis();               // Reset timer
    motorIsRunning = true;
  }

  // Stop motor after the configured duration
  if (motorIsRunning && (millis() - motorStartTime >= MOTOR_RUN_TIME_MS)) {
    motor.stop();
    motorIsRunning = false;
  }
}

// === Arduino Setup ===
void setup() {
  Serial.begin(9600);
  Wire.begin();

  lcd.init();
  lcd.backlight();

  uvSensor.begin(VEML6070_1_T);
  lightSensor.begin();
  hdcSensor.begin(); // Just init, no fail handling here

  Serial.println("All sensors initialized.");
}

// === Main Loop ===
void loop() {
  // === Read all sensor values ===
  waterLevel = analogRead(A0);
  soilMoisture = analogRead(A1);
  uvIndex = uvSensor.readUV();
  lightLux = lightSensor.readLux();
  temperature = hdcSensor.readTemperature();
  humidity = hdcSensor.readHumidity();

  // === Serial Monitor Output ===
  Serial.print("Water Level: "); Serial.println(waterLevel);
  Serial.print("Soil Moisture: "); Serial.println(soilMoisture);
  Serial.print("UV Index: "); Serial.println(uvIndex);
  Serial.print("Light Lux: "); Serial.println(lightLux);
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

  // === Check Conditions ===
  bool conditionsOK = all_conditions_met(temperature, humidity, uvIndex, lightLux, soilMoisture, waterLevel);

  // === LCD Output ===
  lcd.clear();

  if (conditionsOK) {
    // Display only soil moisture and motor status
    lcd.setCursor(0, 1);
    lcd.print("Soil Moisture: ");
    lcd.print(soilMoisture);

    lcd.setCursor(3, 2); // Centered (approx) for 20 columns
    lcd.print(">> MOTOR ON <<");
  } else {
    // Full sensor data display
    lcd.setCursor(0, 0);
    lcd.print("Water: ");
    lcd.print(waterLevel);
    lcd.setCursor(10, 0);
    lcd.print("Soil: ");
    lcd.print(soilMoisture);

    lcd.setCursor(0, 1);
    lcd.print("UV: ");
    lcd.print(uvIndex);
    lcd.setCursor(10, 1);
    lcd.print("Lux: ");
    lcd.print(lightLux);

    lcd.setCursor(0, 2);
    lcd.print("Temp: ");
    lcd.print(round(temperature));
    lcd.print("C");
    lcd.setCursor(10, 2);
    lcd.print("Hum: ");
    lcd.print(humidity, 0);
    lcd.print("%");
  }

  // === Control Motor ===
  control_motor(conditionsOK);

  delay(1500); // Refresh every 1.5 seconds
}
