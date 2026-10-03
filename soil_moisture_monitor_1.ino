/*
  =======================================================================
  Project : 4-Plant Soil Moisture Monitor + Semi-automatic Watering
            ESP32 DevKit V1 + Arduino framework + Adafruit IO (MQTT)

  WHAT IT DOES
    1. Every 2 seconds the ESP32 reads 4 soil moisture sensors and turns
       each reading into a percentage (0% = dry ... 100% = wet).
    2. When a plant drops below its minimum, an alert message is sent to
       Adafruit IO (an Adafruit IO "Action" can forward it as an email).
    3. The owner presses that plant's button on the Adafruit IO dashboard.
       The plant's valve and the central (main) valve open.
    4. After a fixed time (5 minutes by default) the valves close by
       themselves. Watering NEVER starts without a button press.

  HARDWARE
    - ESP32 DevKit V1
    - 4 analog capacitive soil moisture sensors
    - Relay module with at least 5 channels (4 plants + 1 central valve)
    - 5 normally-closed solenoid valves, powered by their OWN supply
      (never from the ESP32 pins)

  ADAFRUIT IO FEEDS TO CREATE FIRST (the feed KEY must match exactly)
    Sensor data : sensor1-moisture, sensor2-moisture,
                  sensor3-moisture, sensor4-moisture
    Alerts      : alerts
    Buttons     : cactus-water-now, succulents-water-now,
                  tropical-water-now, herbs-water-now

  HOW THIS FILE IS ORGANIZED
    1. Credentials         9. setup()      - runs once at power-on
    2. Sensors            10. loop()       - runs forever
    3. Valves / relays    11. Reading the sensors
    4. Thresholds         12. Alerts
    5. Timing             13. Watering (start / stop / central valve)
    6. WiFi + MQTT objects 14. Publishing to Adafruit IO
    7. Global state       15. WiFi / MQTT connections
    8. Prototypes

  STATUS: compiles against the real Adafruit_MQTT library headers, but has
  NOT been tested on hardware yet. Test in stages BEFORE connecting water.
  =======================================================================
*/

#include <Arduino.h>            // core Arduino functions (pinMode, analogRead...)
#include <string.h>             // text comparison helpers (strcmp)
#include <strings.h>            // case-insensitive comparison (strcasecmp)
#include <WiFi.h>               // ESP32 WiFi
#include "Adafruit_MQTT.h"      // MQTT protocol (install "Adafruit MQTT Library")
#include "Adafruit_MQTT_Client.h"

// =====================================================================
// 1. CREDENTIALS
//    Your WiFi login and your Adafruit IO account. Keep them PRIVATE:
//    remove them before sharing this file, and regenerate the AIO key if
//    it ever leaks. The ESP32 only supports 2.4 GHz WiFi.
// =====================================================================
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Maiam00"
#define AIO_KEY         "PASTE_YOUR_NEW_AIO_KEY_HERE"

// =====================================================================
// 2. SENSORS
//    Which pins the sensors are on, what each plant is called, and how
//    to convert the raw sensor number into a percentage.
//    Pins 32-35 belong to "ADC1", which keeps working while WiFi is on
//    (the other ADC group does not).
// =====================================================================
const uint8_t NUM_SENSORS = 4;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {32, 33, 34, 35};

const char* SENSOR_NAMES[NUM_SENSORS] = {
  "Cactus 1", "Suculentas 2", "Tropicales 3", "Hierbas 4"
};

// Calibration, one pair per sensor. Sensors are not identical, so measure:
//   DRY = raw value with the sensor held in air
//   WET = raw value with the sensor in a glass of water (up to its line)
// The raw value is printed in the Serial Monitor to help with this.
const int DRY_VALUE[NUM_SENSORS] = {3000, 3000, 3000, 3000};
const int WET_VALUE[NUM_SENSORS] = {1200, 1200, 1200, 1200};

// A raw reading outside this range means the sensor is unplugged or
// shorted. Such readings are ignored (no alerts, no watering).
const int RAW_MIN_VALID = 100;
const int RAW_MAX_VALID = 4000;

// =====================================================================
// 3. VALVES / RELAYS
//    Each plant has a relay that opens its valve. A central relay opens
//    the main water line whenever ANY plant valve is open.
//    (GPIO14 is avoided: it pulses at boot and could click a relay.)
// =====================================================================
const uint8_t RELAY_PIN[NUM_SENSORS] = {27, 26, 25, 23};
const uint8_t CENTRAL_VALVE_PIN      = 13;

// What pin level switches a relay ON or OFF. Many cheap relay modules are
// "active-LOW" (LOW = ON). If yours is, swap HIGH and LOW here and nothing
// else needs changing. Test with NO water connected!
#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// Maximum watering time per plant. 5UL * 60UL * 1000UL = 5 minutes in
// milliseconds ("UL" = unsigned long). Tune each plant after measuring
// how much water your tubing delivers per minute.
const unsigned long WATERING_DURATION_MS[NUM_SENSORS] = {
  5UL * 60UL * 1000UL,   // Cactus
  5UL * 60UL * 1000UL,   // Suculentas
  5UL * 60UL * 1000UL,   // Tropicales
  5UL * 60UL * 1000UL    // Hierbas
};

// Safety: refuse to water a plant that is already above its MAX threshold.
const bool BLOCK_IF_TOO_WET = true;

// =====================================================================
// 4. THRESHOLDS (in percent, one pair per plant)
//    Below MIN  -> "too dry" alert.   Above MAX -> "too wet" alert.
//    HYSTERESIS is a safety margin: after an alert, the reading must move
//    2% past the limit before the alert is cleared. Without it, a value
//    hovering around the limit would spam you with alerts.
// =====================================================================
const float MIN_THRESHOLD[NUM_SENSORS] = { 5.0, 10.0, 40.0, 30.0};
const float MAX_THRESHOLD[NUM_SENSORS] = {30.0, 35.0, 75.0, 85.0};
const float HYSTERESIS = 2.0;

// =====================================================================
// 5. TIMING (all in milliseconds)
//    The program never uses long waits; it compares millis() (time since
//    boot) with these intervals, so it can do several jobs at once.
// =====================================================================
const unsigned long READ_INTERVAL_MS    = 2000;   // read sensors + check thresholds
const unsigned long PUBLISH_INTERVAL_MS = 60000;  // send data to the cloud (free AIO allows 30 points/min)
const unsigned long ALERT_REMINDER_MS   = 4UL * 60UL * 60UL * 1000UL; // repeat "still dry" every 4 h
const unsigned long MQTT_RETRY_MS       = 10000;  // wait between MQTT reconnect attempts
const unsigned long WIFI_RETRY_MS       = 10000;  // wait between WiFi reconnect attempts
const unsigned long PING_INTERVAL_MS    = 60000;  // keep-alive ping to Adafruit IO

// =====================================================================
// 6. WIFI + MQTT OBJECTS
//    MQTT is a messaging protocol: data is sent to named "topics". On
//    Adafruit IO a feed's topic is  username/feeds/feed-key.
//      client  = the network connection (WiFi)
//      mqtt    = speaks MQTT over that connection, logs in to Adafruit IO
//      Publish = ESP32 SENDS to a feed (sensor values, alerts)
//      Subscribe = ESP32 LISTENS to a feed (dashboard buttons)
//    Arrays of pointers (the * and &) let loops handle all 4 plants.
//    Create the feeds in Adafruit IO first; the keys must match exactly.
// =====================================================================
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Moisture readings (ESP32 -> dashboard graphs)
Adafruit_MQTT_Publish moisture1(&mqtt, AIO_USERNAME "/feeds/sensor1-moisture");
Adafruit_MQTT_Publish moisture2(&mqtt, AIO_USERNAME "/feeds/sensor2-moisture");
Adafruit_MQTT_Publish moisture3(&mqtt, AIO_USERNAME "/feeds/sensor3-moisture");
Adafruit_MQTT_Publish moisture4(&mqtt, AIO_USERNAME "/feeds/sensor4-moisture");
Adafruit_MQTT_Publish* moistureFeeds[NUM_SENSORS] = {
  &moisture1, &moisture2, &moisture3, &moisture4
};

// Text messages: too dry, watering started/stopped, errors
Adafruit_MQTT_Publish alertFeed(&mqtt, AIO_USERNAME "/feeds/alerts");

// Watering buttons (dashboard -> ESP32)
Adafruit_MQTT_Subscribe waterCommand1(&mqtt, AIO_USERNAME "/feeds/cactus-water-now");
Adafruit_MQTT_Subscribe waterCommand2(&mqtt, AIO_USERNAME "/feeds/succulents-water-now");
Adafruit_MQTT_Subscribe waterCommand3(&mqtt, AIO_USERNAME "/feeds/tropical-water-now");
Adafruit_MQTT_Subscribe waterCommand4(&mqtt, AIO_USERNAME "/feeds/herbs-water-now");
Adafruit_MQTT_Subscribe* waterCommands[NUM_SENSORS] = {
  &waterCommand1, &waterCommand2, &waterCommand3, &waterCommand4
};

// =====================================================================
// 7. GLOBAL STATE
//    The program's "memory". These variables live outside any function,
//    so they keep their values between passes of loop(). Everything
//    starts as "nothing is happening": valves closed, no plant dry.
// =====================================================================
int   rawValue[NUM_SENSORS]        = {0, 0, 0, 0};                 // last raw sensor reading
float moisturePercent[NUM_SENSORS] = {0, 0, 0, 0};                 // last reading as a percentage
bool  sensorOk[NUM_SENSORS]        = {false, false, false, false}; // is the reading believable?

bool  isDry[NUM_SENSORS]           = {false, false, false, false}; // currently below MIN (alert already sent)
bool  isWet[NUM_SENSORS]           = {false, false, false, false}; // currently above MAX (alert already sent)
unsigned long lastDryAlert[NUM_SENSORS] = {0, 0, 0, 0};            // when the last dry alert/reminder was sent

bool  valveOpen[NUM_SENSORS]       = {false, false, false, false}; // is this plant's valve open?
bool  centralOpen                  = false;                        // is the central valve open?
unsigned long wateringStartTime[NUM_SENSORS] = {0, 0, 0, 0};       // when each watering began

unsigned long lastRead = 0, lastPublish = 0, lastPing = 0;         // timers for the periodic jobs
unsigned long lastMqttAttempt = 0, lastWifiAttempt = 0;            // timers for reconnecting

// =====================================================================
// 8. PROTOTYPES
//    The compiler reads top to bottom, so it must be told which functions
//    exist before they are used. This list is like a table of contents.
//    Each line must match the real function further down.
// =====================================================================
void  readAllSensors();
int   readRawAveraged(uint8_t pin);
float rawToPercent(uint8_t i, int raw);
void  printReadings();
void  checkThresholds();
void  checkWateringCommands();
bool  isOnCommand(const char* payload);
void  startWatering(uint8_t i);
void  stopWatering(uint8_t i, const char* reason);
void  autoStopWatering();
void  manageCentralValve();
void  setValve(uint8_t pin, bool open);
void  sendAlert(const char* msg);
void  publishReadings();
void  connectToWiFi();
void  maintainConnections();
void  connectToMQTT();

// =====================================================================
// 9. setup()
//    Runs ONCE when the ESP32 powers on or resets: prepare the pins,
//    connect to WiFi, register the button listeners, take a first reading.
// =====================================================================
void setup() {
  Serial.begin(115200);   // text channel to the computer (Serial Monitor speed: 115200)

  // Valves FIRST, and always in the safe (closed) state. The level is set
  // before the pin becomes an output so the relay never twitches.
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(RELAY_PIN[i], RELAY_OFF);
    pinMode(RELAY_PIN[i], OUTPUT);
    digitalWrite(RELAY_PIN[i], RELAY_OFF);
  }
  digitalWrite(CENTRAL_VALVE_PIN, RELAY_OFF);
  pinMode(CENTRAL_VALVE_PIN, OUTPUT);
  digitalWrite(CENTRAL_VALVE_PIN, RELAY_OFF);

  // Sensors: analog inputs, 12-bit readings (0-4095), full 0-3.3 V range
  for (uint8_t i = 0; i < NUM_SENSORS; i++) pinMode(SENSOR_PINS[i], INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("\n=== Soil Moisture Monitor starting ===");

  connectToWiFi();   // waits here at most ~15 s; if it fails, loop() keeps retrying

  // Register the 4 button feeds once; mqtt.connect() re-subscribes
  // automatically after every reconnect.
  for (uint8_t i = 0; i < NUM_SENSORS; i++) mqtt.subscribe(waterCommands[i]);

  readAllSensors();   // real values in memory before any command can arrive
  Serial.println("Setup complete.\n");
}

// =====================================================================
// 10. loop()
//     Runs over and over, forever. Each pass: safety timers first, then
//     network, then incoming button presses, then the periodic jobs
//     (read every 2 s, publish every 60 s). There are no long waits, so
//     one slow job can't stop another.
// =====================================================================
void loop() {
  unsigned long now = millis();   // milliseconds since the ESP32 booted

  // SAFETY FIRST: valve timers run on every pass and never depend on
  // WiFi or MQTT, so a lost connection can't leave a valve open.
  autoStopWatering();
  manageCentralValve();

  maintainConnections();          // reconnect WiFi/MQTT if needed (non-blocking)

  if (mqtt.connected()) {
    checkWateringCommands();      // waits up to 100 ms for button presses
    if (now - lastPing >= PING_INTERVAL_MS) {   // periodic keep-alive
      lastPing = now;
      if (!mqtt.ping()) mqtt.disconnect();
    }
  } else {
    delay(50);                    // offline: don't spin at full speed
  }

  if (now - lastRead >= READ_INTERVAL_MS) {     // every 2 s
    lastRead = now;
    readAllSensors();
    printReadings();
    checkThresholds();
  }

  if (now - lastPublish >= PUBLISH_INTERVAL_MS) { // every 60 s
    lastPublish = now;
    publishReadings();
  }
}

// =====================================================================
// 11. SENSORS
//     Raw reading -> average -> percentage -> print.
// =====================================================================

// Reads one pin 16 times and returns the average. The ESP32's analog
// input is noisy, so a single reading can jump around.
int readRawAveraged(uint8_t pin) {
  long sum = 0;
  for (uint8_t k = 0; k < 16; k++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / 16);
}

// Converts a raw reading to 0-100% using this sensor's DRY and WET points.
// The sensor gives HIGH numbers when dry and LOW when wet, so the formula
// flips it: (DRY - raw) / (DRY - WET) * 100. "(float)" forces decimal
// math (plain integer division would round 0.5 down to 0).
// constrain() keeps the result between 0 and 100.
float rawToPercent(uint8_t i, int raw) {
  float p = (float)(DRY_VALUE[i] - raw) / (float)(DRY_VALUE[i] - WET_VALUE[i]) * 100.0f;
  return constrain(p, 0.0f, 100.0f);
}

// Reads all 4 sensors and stores raw value, validity and percentage.
void readAllSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawValue[i]        = readRawAveraged(SENSOR_PINS[i]);
    sensorOk[i]        = (rawValue[i] > RAW_MIN_VALID && rawValue[i] < RAW_MAX_VALID);
    moisturePercent[i] = rawToPercent(i, rawValue[i]);
  }
}

// Prints one line per plant to the Serial Monitor. The raw value is shown
// on purpose: it is what you need for calibration.
void printReadings() {
  Serial.println("---------------------------------------------");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(": ");
    Serial.print(moisturePercent[i], 1);
    Serial.print("%  (raw ");
    Serial.print(rawValue[i]);
    Serial.print(", range ");
    Serial.print(MIN_THRESHOLD[i], 0);
    Serial.print("-");
    Serial.print(MAX_THRESHOLD[i], 0);
    Serial.print("%)");
    if (!sensorOk[i]) Serial.print("  << SENSOR FAULT");
    if (valveOpen[i]) Serial.print("  [WATERING]");
    Serial.println();
  }
}

// =====================================================================
// 12. ALERTS
//     Messages go to the Serial Monitor always, and to the Adafruit IO
//     "alerts" feed when online. An Adafruit IO Action can email them.
// =====================================================================

// Prints a message and publishes it to the alerts feed (if connected).
void sendAlert(const char* msg) {
  Serial.println(msg);
  if (mqtt.connected()) {
    if (!alertFeed.publish(msg)) Serial.println("(alert publish failed)");
  }
}

// Compares each plant's moisture with its limits and sends an alert only
// when the situation CHANGES (isDry / isWet remember the previous state),
// not every 2 seconds. Also sends a "recovered" message and a reminder
// every ALERT_REMINDER_MS while a plant stays dry. Sensors with a fault
// are skipped so garbage readings never trigger alerts.
void checkThresholds() {
  unsigned long now = millis();
  char msg[128];

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!sensorOk[i]) continue;
    float m = moisturePercent[i];

    // ---- too dry ----
    if (!isDry[i] && m < MIN_THRESHOLD[i]) {
      isDry[i] = true;
      lastDryAlert[i] = now;
      snprintf(msg, sizeof(msg), "%s TOO DRY (%.1f%%) - press the button to water!",
               SENSOR_NAMES[i], m);
      sendAlert(msg);
    } else if (isDry[i]) {
      if (m > MIN_THRESHOLD[i] + HYSTERESIS) {
        isDry[i] = false;
        snprintf(msg, sizeof(msg), "%s RECOVERED (%.1f%%)", SENSOR_NAMES[i], m);
        sendAlert(msg);
      } else if (now - lastDryAlert[i] >= ALERT_REMINDER_MS) {
        lastDryAlert[i] = now;
        snprintf(msg, sizeof(msg), "REMINDER: %s still dry (%.1f%%)", SENSOR_NAMES[i], m);
        sendAlert(msg);
      }
    }

    // ---- too wet ----
    if (!isWet[i] && m > MAX_THRESHOLD[i]) {
      isWet[i] = true;
      snprintf(msg, sizeof(msg), "%s TOO WET (%.1f%%)", SENSOR_NAMES[i], m);
      sendAlert(msg);
    } else if (isWet[i] && m < MAX_THRESHOLD[i] - HYSTERESIS) {
      isWet[i] = false;
    }
  }
}

// =====================================================================
// 13. WATERING
//     Button press -> safety checks -> valve opens -> timer -> valve
//     closes. The central valve follows the plant valves.
// =====================================================================

// Switches one relay on or off (uses RELAY_ON / RELAY_OFF from section 3).
void setValve(uint8_t pin, bool open) {
  digitalWrite(pin, open ? RELAY_ON : RELAY_OFF);
}

// Decides if a dashboard message means "water now". Toggles send ON/OFF
// and momentary buttons send 1/0. The WHOLE word is compared: checking
// only the first letter would mistake "OFF" for "ON".
bool isOnCommand(const char* p) {
  return strcmp(p, "1") == 0 || strcasecmp(p, "ON") == 0 || strcasecmp(p, "TRUE") == 0;
}

// Listens (up to 100 ms) for dashboard messages, finds which plant's
// button sent it, and starts watering if the message is an "ON".
void checkWateringCommands() {
  Adafruit_MQTT_Subscribe* sub;
  while ((sub = mqtt.readSubscription(100))) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (sub == waterCommands[i]) {
        const char* payload = (const char*)sub->lastread;   // the message text
        Serial.print("Command for ");
        Serial.print(SENSOR_NAMES[i]);
        Serial.print(": ");
        Serial.println(payload);
        if (isOnCommand(payload)) startWatering(i);
      }
    }
  }
}

// Opens a plant's valve and starts its timer, unless a safety check
// fails: already watering, sensor fault, or soil already too wet.
void startWatering(uint8_t i) {
  char msg[128];

  if (valveOpen[i]) return;   // already watering, ignore repeated presses

  if (!sensorOk[i]) {
    snprintf(msg, sizeof(msg), "%s: sensor fault - watering REFUSED", SENSOR_NAMES[i]);
    sendAlert(msg);
    return;
  }
  if (BLOCK_IF_TOO_WET && moisturePercent[i] >= MAX_THRESHOLD[i]) {
    snprintf(msg, sizeof(msg), "%s already wet (%.1f%%) - watering REFUSED",
             SENSOR_NAMES[i], moisturePercent[i]);
    sendAlert(msg);
    return;
  }

  valveOpen[i] = true;
  wateringStartTime[i] = millis();   // remember when, for the auto-stop
  setValve(RELAY_PIN[i], true);

  snprintf(msg, sizeof(msg), "%s WATERING STARTED (auto-stop in %lu min)",
           SENSOR_NAMES[i], WATERING_DURATION_MS[i] / 60000UL);
  sendAlert(msg);
}

// Closes a plant's valve. The valve is closed BEFORE the message is sent,
// so a slow network can never delay the shut-off.
void stopWatering(uint8_t i, const char* reason) {
  setValve(RELAY_PIN[i], false);
  valveOpen[i] = false;

  char msg[128];
  snprintf(msg, sizeof(msg), "%s WATERING STOPPED (%s)", SENSOR_NAMES[i], reason);
  sendAlert(msg);
}

// Called on every pass of loop(): closes any valve that has been open
// longer than its allowed time. Unsigned subtraction keeps this correct
// even when millis() wraps around after ~49 days.
void autoStopWatering() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (valveOpen[i] && (now - wateringStartTime[i]) >= WATERING_DURATION_MS[i]) {
      stopWatering(i, "timer");
    }
  }
}

// The central (main line) valve is open only while at least one plant
// valve is open. It acts only when the needed state changes.
void manageCentralValve() {
  bool anyOpen = false;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (valveOpen[i]) { anyOpen = true; break; }
  }
  if (anyOpen != centralOpen) {
    centralOpen = anyOpen;
    setValve(CENTRAL_VALVE_PIN, centralOpen);
    Serial.println(centralOpen ? "Central valve OPENED" : "Central valve CLOSED");
  }
}

// =====================================================================
// 14. PUBLISH TO ADAFRUIT IO
//     Sends each valid percentage to its feed, to draw the dashboard
//     graphs. Called once a minute to stay within the free-plan limit.
// =====================================================================
void publishReadings() {
  if (!mqtt.connected()) return;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorOk[i]) moistureFeeds[i]->publish(moisturePercent[i]);
  }
}

// =====================================================================
// 15. CONNECTIONS
//     WiFi first, then MQTT (the login to Adafruit IO). Reconnecting is
//     done in small non-blocking steps so watering timers keep running
//     while the network is down.
// =====================================================================

// Connects to WiFi. Waits here at most ~15 s, used once at boot.
void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (uint8_t tries = 0; WiFi.status() != WL_CONNECTED && tries < 30; tries++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi FAILED (will keep retrying in the background)");
  }
}

// Logs in to Adafruit IO. At most one attempt every 10 s. The library's
// connect() returns 0 on success, anything else is an error code.
void connectToMQTT() {
  if (mqtt.connected()) return;
  unsigned long now = millis();
  if (lastMqttAttempt != 0 && now - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = now;

  Serial.print("Connecting to Adafruit IO... ");
  int8_t ret = mqtt.connect();
  if (ret == 0) {
    Serial.println("connected");
  } else {
    Serial.println(mqtt.connectErrorString(ret));
    mqtt.disconnect();
  }
}

// Called from loop(): if WiFi dropped, retry it every 10 s; if WiFi is
// fine, make sure the MQTT login is alive.
void maintainConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiAttempt >= WIFI_RETRY_MS) {
      lastWifiAttempt = now;
      Serial.println("WiFi lost - reconnecting...");
      WiFi.reconnect();
    }
    return;
  }
  connectToMQTT();
}
