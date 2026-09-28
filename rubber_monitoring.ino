/* ============================================================================
   ML-Enabled IoT-Based Rubber Plantation Monitoring and Tapping Decision System
   ----------------------------------------------------------------------------
   File        : rubber_monitoring.ino
   Target      : ESP32 (Arduino IDE / Arduino-ESP32 core)
   Description : Firmware implementing the sensor, connectivity and fire-safety
                 requirements described in the Final Project Report:
                   - Reads DHT11 (temperature, humidity), soil moisture (ADC),
                     rain sensor (digital) and flame sensor (digital)
                   - Sends Temperature / Humidity / Soil Moisture to ThingSpeak
                   - Sends all four features to a FastAPI backend at /predict
                     and receives a Random-Forest tapping-suitability prediction
                   - Runs an independent, immediate fire-safety subsystem
                     (buzzer + relay + water pump) that does NOT depend on
                     Wi-Fi, the backend, or the ML prediction
                   - Notifies the backend of a fire event so it can trigger an
                     SMS alert through Twilio

   Required libraries (Arduino Library Manager):
                   - DHT sensor library (Adafruit)
                   - Adafruit Unified Sensor
                   - ArduinoJson  (by Benoit Blanchon)
                   (WiFi.h and HTTPClient.h ship with the ESP32 board package)
   ============================================================================ */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

/* --------------------------------------------------------------------------
   1. USER CONFIGURATION — edit these values for your deployment
   -------------------------------------------------------------------------- */

// ---- Wi-Fi credentials ----
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ---- ThingSpeak configuration ----
const char* THINGSPEAK_HOST       = "http://api.thingspeak.com";
const char* THINGSPEAK_WRITE_API_KEY = "YOUR_THINGSPEAK_WRITE_API_KEY";

// ---- Backend (FastAPI) configuration ----
// Example: "http://192.168.1.50:8000"  or  "https://your-backend-domain.com"
const char* BACKEND_HOST = "http://YOUR_BACKEND_HOST:8000";

// ---- Timing configuration (milliseconds) ----
const unsigned long SENSOR_READ_INTERVAL   = 5000;   // read sensors every 5 s
const unsigned long THINGSPEAK_INTERVAL    = 20000;  // ThingSpeak min. 15 s between writes
const unsigned long PREDICTION_INTERVAL    = 30000;  // request a prediction every 30 s
const unsigned long FIRE_SMS_COOLDOWN      = 60000;  // avoid spamming fire SMS

/* --------------------------------------------------------------------------
   2. PIN CONFIGURATION (matches Section 7.1 of the report)
   -------------------------------------------------------------------------- */

#define PIN_DHT11          27   // DHT11 data           - single-wire digital
#define PIN_SOIL_MOISTURE  32   // Soil moisture sensor  - analog / ADC
#define PIN_RAIN           5    // Rain sensor           - digital GPIO
#define PIN_FLAME          22   // Flame sensor          - digital GPIO
#define PIN_BUZZER         21   // Buzzer                - digital output
#define PIN_RELAY          33   // Relay (drives pump)   - digital output

#define DHTTYPE DHT11

/* Logic levels — adjust to match the specific sensor modules used.
   Many low-cost rain and flame sensor modules pull the digital pin LOW
   when the condition (rain / flame) is detected. */
#define RAIN_DETECTED_LEVEL   LOW
#define FLAME_DETECTED_LEVEL  LOW
#define RELAY_ON_LEVEL        HIGH
#define RELAY_OFF_LEVEL       LOW

/* Soil moisture ADC calibration (ESP32 ADC is 12-bit: 0-4095).
   Calibrate AIR_VALUE (sensor in dry air) and WATER_VALUE (sensor in water)
   for the specific soil sensor and soil type before deployment. */
#define SOIL_ADC_AIR_VALUE    3300
#define SOIL_ADC_WATER_VALUE  1200

/* --------------------------------------------------------------------------
   3. GLOBAL OBJECTS AND STATE
   -------------------------------------------------------------------------- */

DHT dht(PIN_DHT11, DHTTYPE);

struct SensorReadings {
  float temperature   = NAN;
  float humidity      = NAN;
  int   soilMoisture  = 0;     // percentage 0-100
  int   rain          = 0;     // 0 = no rain, 1 = rain
  bool  valid         = false;
};

SensorReadings currentReadings;

bool pumpActive       = false;
bool lastFlameState   = false;

unsigned long lastSensorReadTime   = 0;
unsigned long lastThingSpeakTime   = 0;
unsigned long lastPredictionTime   = 0;
unsigned long lastFireAlertTime    = 0;

/* --------------------------------------------------------------------------
   4. SETUP
   -------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(200);

  // Configure actuator pins
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY, RELAY_OFF_LEVEL);

  // Configure sensor pins
  pinMode(PIN_RAIN, INPUT);
  pinMode(PIN_FLAME, INPUT);
  // PIN_SOIL_MOISTURE is analog, no pinMode needed on ESP32

  dht.begin();

  connectToWiFi();

  Serial.println("System initialized. Entering main loop.");
}

/* --------------------------------------------------------------------------
   5. MAIN LOOP
   -------------------------------------------------------------------------- */

void loop() {
  // 5.1 Fire safety check runs on EVERY loop iteration for fast response,
  //     independent of Wi-Fi / backend / ML availability.
  checkFlameSafety();

  unsigned long now = millis();

  // 5.2 Periodic sensor read
  if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = now;
    readAllSensors();
    printReadings();
  }

  // 5.3 Periodic ThingSpeak upload (Temperature, Humidity, Soil Moisture)
  if (currentReadings.valid && (now - lastThingSpeakTime >= THINGSPEAK_INTERVAL)) {
    lastThingSpeakTime = now;
    sendToThingSpeak(currentReadings);
  }

  // 5.4 Periodic ML tapping-suitability prediction via FastAPI backend
  if (currentReadings.valid && (now - lastPredictionTime >= PREDICTION_INTERVAL)) {
    lastPredictionTime = now;
    requestTappingPrediction(currentReadings);
  }

  maintainWiFiConnection();
}

/* --------------------------------------------------------------------------
   6. Wi-Fi HANDLING
   -------------------------------------------------------------------------- */

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi connection failed. Will retry in background.");
  }
}

void maintainWiFiConnection() {
  static unsigned long lastRetry = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastRetry > 10000) {
    lastRetry = millis();
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    WiFi.disconnect();
    WiFi.reconnect();
  }
}

/* --------------------------------------------------------------------------
   7. SENSOR READING
   -------------------------------------------------------------------------- */

int readSoilMoisturePercent() {
  int raw = analogRead(PIN_SOIL_MOISTURE);
  // Map raw ADC value to a 0-100% moisture scale.
  // Dry soil -> higher raw value, wet soil -> lower raw value (typical resistive sensor).
  int percent = map(raw, SOIL_ADC_AIR_VALUE, SOIL_ADC_WATER_VALUE, 0, 100);
  percent = constrain(percent, 0, 100);
  return percent;
}

void readAllSensors() {
  float temperature = dht.readTemperature();   // Celsius
  float humidity     = dht.readHumidity();

  bool dhtOk = !isnan(temperature) && !isnan(humidity);
  if (!dhtOk) {
    Serial.println("Warning: failed to read from DHT11 sensor. Skipping this cycle.");
    currentReadings.valid = false;
    return;
  }

  int soilPercent = readSoilMoisturePercent();

  int rainRaw   = digitalRead(PIN_RAIN);
  int rainState = (rainRaw == RAIN_DETECTED_LEVEL) ? 1 : 0;

  currentReadings.temperature  = temperature;
  currentReadings.humidity     = humidity;
  currentReadings.soilMoisture = soilPercent;
  currentReadings.rain         = rainState;
  currentReadings.valid        = true;
}

void printReadings() {
  if (!currentReadings.valid) return;
  Serial.println("---- Sensor Readings ----");
  Serial.printf("Temperature   : %.1f C\n", currentReadings.temperature);
  Serial.printf("Humidity      : %.1f %%\n", currentReadings.humidity);
  Serial.printf("Soil Moisture : %d %%\n", currentReadings.soilMoisture);
  Serial.printf("Rain          : %s\n", currentReadings.rain ? "Yes" : "No");
  Serial.println("-------------------------");
}

/* --------------------------------------------------------------------------
   8. THINGSPEAK INTEGRATION
      Fields: 1 = Temperature, 2 = Humidity, 3 = Soil Moisture
   -------------------------------------------------------------------------- */

void sendToThingSpeak(const SensorReadings& r) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ThingSpeak upload skipped: Wi-Fi not connected.");
    return;
  }

  HTTPClient http;
  String url = String(THINGSPEAK_HOST) + "/update?api_key=" + THINGSPEAK_WRITE_API_KEY +
               "&field1=" + String(r.temperature, 1) +
               "&field2=" + String(r.humidity, 1) +
               "&field3=" + String(r.soilMoisture);

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0) {
    String response = http.getString();
    Serial.printf("ThingSpeak update sent. HTTP %d, entry id: %s\n", httpCode, response.c_str());
  } else {
    Serial.printf("ThingSpeak update failed. HTTP error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

/* --------------------------------------------------------------------------
   9. BACKEND / ML PREDICTION INTEGRATION
      POST JSON to {BACKEND_HOST}/predict
      Expected response: {"prediction": "Suitable"/"Not Suitable", "probability": 0.xx}
   -------------------------------------------------------------------------- */

void requestTappingPrediction(const SensorReadings& r) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Prediction request skipped: Wi-Fi not connected.");
    return;
  }

  HTTPClient http;
  String url = String(BACKEND_HOST) + "/predict";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<128> requestDoc;
  requestDoc["temperature"]    = r.temperature;
  requestDoc["humidity"]       = r.humidity;
  requestDoc["soil_moisture"]  = r.soilMoisture;
  requestDoc["rain"]           = r.rain;

  String requestBody;
  serializeJson(requestDoc, requestBody);

  int httpCode = http.POST(requestBody);

  if (httpCode == 200) {
    String responseBody = http.getString();

    StaticJsonDocument<256> responseDoc;
    DeserializationError err = deserializeJson(responseDoc, responseBody);

    if (!err) {
      const char* prediction = responseDoc["prediction"];
      float probability      = responseDoc["probability"] | 0.0f;

      Serial.printf("Tapping Prediction: %s (probability %.2f)\n", prediction, probability);
      // Note: SMS notification for the prediction is triggered server-side
      // by the FastAPI backend through Twilio, as per the system architecture.
    } else {
      Serial.println("Failed to parse prediction response JSON.");
    }
  } else {
    Serial.printf("Prediction request failed. HTTP code: %d\n", httpCode);
  }

  http.end();
}

/* --------------------------------------------------------------------------
   10. FIRE SAFETY SUBSYSTEM
       Independent of Wi-Fi / ML / backend availability.
       Flame detected -> Buzzer ON, Relay ON (pump ON), best-effort SMS alert.
       Flame cleared   -> Buzzer OFF, Relay OFF (pump OFF).
   -------------------------------------------------------------------------- */

void checkFlameSafety() {
  int flameRaw    = digitalRead(PIN_FLAME);
  bool flameNow   = (flameRaw == FLAME_DETECTED_LEVEL);

  if (flameNow && !lastFlameState) {
    // Rising edge: fire just detected
    activateFireResponse();
  } else if (!flameNow && lastFlameState) {
    // Falling edge: fire condition cleared
    deactivateFireResponse();
  }

  lastFlameState = flameNow;
}

void activateFireResponse() {
  Serial.println("!!! FIRE DETECTED !!! Activating buzzer, relay and pump.");

  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_RELAY, RELAY_ON_LEVEL);
  pumpActive = true;

  // Best-effort SMS alert; the local safety response above does NOT wait
  // on this network call, so fire suppression is never delayed by Wi-Fi.
  unsigned long now = millis();
  if (now - lastFireAlertTime >= FIRE_SMS_COOLDOWN) {
    lastFireAlertTime = now;
    sendFireAlertToBackend();
  }
}

void deactivateFireResponse() {
  Serial.println("Fire condition cleared. Deactivating buzzer, relay and pump.");

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY, RELAY_OFF_LEVEL);
  pumpActive = false;
}

void sendFireAlertToBackend() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Fire alert not sent: Wi-Fi not connected.");
    return;
  }

  HTTPClient http;
  String url = String(BACKEND_HOST) + "/fire-alert";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<128> doc;
  doc["event"] = "flame_detected";
  doc["message"] = "FIRE ALERT: Flame detected in plantation. Pump activated.";

  String body;
  serializeJson(doc, body);

  int httpCode = http.POST(body);
  if (httpCode > 0) {
    Serial.printf("Fire alert sent to backend. HTTP %d\n", httpCode);
  } else {
    Serial.printf("Fire alert failed to send. HTTP error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}
