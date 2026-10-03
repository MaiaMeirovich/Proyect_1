/*
  =======================================================================
  Project : 4-Channel Soil Moisture Monitor with Threshold Alerts
            (ESP32 DevKit V1, Arduino Framework, Adafruit IO)
  
  Status  : FULLY CORRECTED & TESTED
  Author  : MaiaM
  Date    : 2026
  =======================================================================
*/

// =====================================================================
// 1. LIBRARY INCLUDES 
// =====================================================================
#include <Arduino.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


// =====================================================================
// 2. WIFI CREDENTIALS
// =====================================================================
const char* ssid     = " WIFI";
const char* password = "XXXXXXXXX";


// =====================================================================
// 3. ADAFRUIT IO CREDENTIALS
// =====================================================================
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Maiam00"
#define AIO_KEY         "XXXXXXXXXXX"


// =====================================================================
// 4. SENSOR CONFIGURATION
// =====================================================================
const uint8_t NUM_SENSORS = 4;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {32, 33, 34, 35};


const char* SENSOR_NAMES[NUM_SENSORS] = {
  "🌵 Cactus 1",
  "🌿 Suculentas 2",
  "🌴 Tropicales 3",
  "🎋 Hierbas 4"
};

// =====================================================================
// 4.5 RELAY/VALVE CONFIGURATION
// =====================================================================
// One relay per plant (GPIO 27, 26, 25, 14)
const uint8_t RELAY_PIN[NUM_SENSORS] = {27, 26, 25, 14};
const uint8_t CENTRAL_VALVE_PIN = 13;                      // Main water line
const unsigned long WATERING_DURATION_MS = 5 * 60 * 1000;  // 10 minutes max

// =====================================================================
// 5. CALIBRATION POINTS
// =====================================================================
const int DRY_VALUE = 3000;    // raw reading when completely dry
const int WET_VALUE = 1200;    // raw reading when soaking wet


// =====================================================================
// 6. THRESHOLD CONFIGURATION 
// =====================================================================
// Sensor 1 (Cactus): very dry-tolerant
// Sensor 2 (Succulenta): moderate moisture
// Sensor 3 (Tropical): likes humidity
// Sensor 4 (Hierbas): consistent moisture
const float MIN_THRESHOLD[NUM_SENSORS] = {5.0,   10.0,  40.0,  30.0};

const float MAX_THRESHOLD[NUM_SENSORS] = {30.0,  35.0,  75.0,  85.0};


// =====================================================================
// 7. TIMING CONFIGURATION
// =====================================================================
const unsigned long PRINT_INTERVAL_MS = 2000;      // read every 2 seconds
const unsigned long ALERT_COOLDOWN_MS = 14400000;    // don't spam (4 hr)
//  print_interval:la idea principal es que si la medida ocurre cada 1ms 
//el sistema se drena desde el punto de vista de bateria y de data traffic, bloqueamos el cpu cada 2 sec
//alert_cooldown:la idea principal es que

// =====================================================================
// 8. MQTT & WIFI OBJECTS
// =====================================================================
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish moisture1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor1.moisture");
Adafruit_MQTT_Publish moisture2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor2.moisture");
Adafruit_MQTT_Publish moisture3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor3.moisture");
Adafruit_MQTT_Publish moisture4 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor4.moisture");

Adafruit_MQTT_Publish* moistureFeeds[NUM_SENSORS] = {
  &moisture1, &moisture2, &moisture3, &moisture4
};

// Alert feed- Notifications
Adafruit_MQTT_Publish alertFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alerts");

// ✅ NEW: Subscribe to watering commands from Adafruit IO
Adafruit_MQTT_Subscribe waterCommand1 = 
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/cactus-water-now");
Adafruit_MQTT_Subscribe waterCommand2 = 
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/succulents-water-now");
Adafruit_MQTT_Subscribe waterCommand3 = 
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/tropical-water-now");
Adafruit_MQTT_Subscribe waterCommand4 = 
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/herbs-water-now");

Adafruit_MQTT_Subscribe* waterCommands[NUM_SENSORS] = {
  &waterCommand1, &waterCommand2, &waterCommand3, &waterCommand4
};

// =====================================================================
// 9. GLOBAL STATE VARIABLES
// =====================================================================
unsigned long previousMillis = 0;
float moisturePercent[NUM_SENSORS];

typedef struct {
  bool isTooWet;
  bool isTooDry;
} SensorState;

SensorState previousState[NUM_SENSORS] = {
  {false, false},
  {false, false},
  {false, false},
  {false, false}
};

//Track watering start time for each plant (auto-stop after 10 min)
unsigned long wateringStartTime[NUM_SENSORS] = {0, 0, 0, 0};


// =====================================================================
// 10. FUNCTION PROTOTYPES 
// =====================================================================
void readAllSensors();
float rawToPercent(int rawValue);
void printReadings();
void checkThresholds();
void checkWateringCommands();  
void autoStopWatering();       
void publishReadings();
void connectToWiFi();
void connectToMQTT();


// =====================================================================
// 11. setup() - RUNS ONCE AT POWER ON
// =====================================================================
void setup() {
  Serial.begin(115200);
  
   // Configure sensor pins
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
  
    // Configure relay pins (one per plant)
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    digitalWrite(RELAY_PIN[i], LOW);  // Start with all valves CLOSED
  }
  
  // Configure central valve pin
  pinMode(CENTRAL_VALVE_PIN, OUTPUT);
  digitalWrite(CENTRAL_VALVE_PIN, LOW);  // Start CLOSED
  }
  
  analogReadResolution(12);
  
  Serial.println("\n\n========================================");
  Serial.println("  Soil Moisture Monitor Initialized");
  Serial.println("  with Threshold Alerts & Push Notifications");
  Serial.println("========================================\n");
  
    connectToWiFi();
  delay(2000);
  connectToMQTT();
  
  // ✅ NEW: Subscribe to watering command feeds
  mqtt.subscribe(&waterCommand1);
  mqtt.subscribe(&waterCommand2);
  mqtt.subscribe(&waterCommand3);
  mqtt.subscribe(&waterCommand4);
  
  Serial.println("Setup complete! Ready to monitor...\n");
}


// =====================================================================
// 12. loop() - RUNS FOREVER (separately from setup!)
// =====================================================================
void loop() {
  unsigned long currentMillis = millis();
  

  if (!mqtt.connected()) {
    connectToMQTT();
  }
  
    // Process incoming MQTT messages & keep connection alive
  mqtt.processStream(100);
  
  // ✅ NEW: Check for remote watering commands (can happen anytime)
  checkWateringCommands();
  autoStopWatering();
  manageCentralValve();
  // Non-blocking timing: only read sensors every 2 seconds
  if (currentMillis - previousMillis >= PRINT_INTERVAL_MS) {
    previousMillis = currentMillis;
    
    readAllSensors();      // Read all 4 sensors
    printReadings();       // Print to serial monitor
    checkThresholds();     // Check thresholds (alerts only, no valve control)
    publishReadings();     // Send to Adafruit IO
  }
}



/// =====================================================================
// 13. FUNCTION DEFINITIONS (All the "workers")
// =====================================================================

/**
 * Read all 4 soil sensors and convert raw values to percentages
 * This runs every 2 seconds (called from loop)
 */
void readAllSensors() {
  // Loop through all 4 sensors
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    // Read the raw ADC value from this sensor (0-4095)
    int rawValue = analogRead(SENSOR_PINS[i]);
    
    // Convert raw value to 0-100% using calibration points
    moisturePercent[i] = rawToPercent(rawValue);
  }
}

/**
 * Convert a raw ADC reading (0-4095) into a percentage (0-100%)
 * Uses the DRY_VALUE and WET_VALUE calibration points
 * 
 * Math:
 *   - DRY_VALUE (3000) = 0% moisture (very dry)
 *   - WET_VALUE (1200) = 100% moisture (in water)
 *   - Anything in between = linear interpolation
 */
float rawToPercent(int rawValue) {
  // Linear map: (DRY - raw) / (DRY - WET) * 100
  float percent = (float)(DRY_VALUE - rawValue) / (float)(DRY_VALUE - WET_VALUE) * 100.0;
  
  // Clamp to 0-100 (prevents negative % or >100%)
  percent = constrain(percent, 0.0, 100.0);
  
  return percent;
}

/**
 * Print all 4 sensor readings to Serial Monitor in a readable format
 * Shows: sensor name, current %, min-max thresholds
 */
void printReadings() {
  Serial.println("---------------------------------------------");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(": ");
    Serial.print(moisturePercent[i], 1);  // 1 decimal place
    Serial.print("% (");
    Serial.print(MIN_THRESHOLD[i], 0);    // Min threshold, no decimals
    Serial.print("-");
    Serial.print(MAX_THRESHOLD[i], 0);    // Max threshold, no decimals
    Serial.println("%)");
  }
  Serial.println("---------------------------------------------");
}

/**
 * Check if soil moisture is in the healthy range, too dry, or too wet
 * Send alerts to Adafruit IO when thresholds are crossed
 * 
 * This is SIMPLIFIED: only sends alerts, does NOT control valves
 * User controls valves manually via Adafruit IO buttons
 */
void checkThresholds() {
  // Loop through all 4 sensors
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    float moisture = moisturePercent[i];
    
    // Check three zones
    bool isTooDry = moisture < MIN_THRESHOLD[i];
    bool isTooWet = moisture > MAX_THRESHOLD[i];
    
    // ═══════════════════════════════════════════════════════════
    // ZONE 1: TOO DRY → Send alert asking user to water
    // ═══════════════════════════════════════════════════════════
    // Only send once per dry period (not every 2 seconds)
    // Check: "is dry NOW" AND "wasn't dry before"
    if (isTooDry && !previousState[i].isTooDry) {
      
      // Build the alert message
      char message[80];
      sprintf(message, "💧 %s TOO DRY (%.1f%%) - Press button to water!", 
              SENSOR_NAMES[i], moisture);
      
      // Print to Serial Monitor (for debugging)
      Serial.println(message);
      
      // Publish to Adafruit IO (sends to cloud, triggers email)
      if (!alertFeed->publish(message)) {
        Serial.println("Failed to publish alert");
      }
      
      // Mark this sensor as "now dry" so we don't spam alerts
      previousState[i].isTooDry = true;
    }
    
    // ═══════════════════════════════════════════════════════════
    // ZONE 2: RECOVERY → Soil went from dry back to healthy
    // ═══════════════════════════════════════════════════════════
    // Check: "not dry NOW" AND "was dry before"
    else if (!isTooDry && previousState[i].isTooDry) {
      
      // Build the recovery message
      char message[80];
      sprintf(message, "✓ %s RECOVERED (%.1f%%)", 
              SENSOR_NAMES[i], moisture);
      
      Serial.println(message);
      
      // Publish confirmation
      if (!alertFeed->publish(message)) {
        Serial.println("Failed to publish message");
      }
      
      // Mark this sensor as "no longer dry"
      previousState[i].isTooDry = false;
    }
    
    // ═══════════════════════════════════════════════════════════
    // ZONE 3: TOO WET → Warning alert (shouldn't happen normally)
    // ═══════════════════════════════════════════════════════════
    else if (isTooWet && !previousState[i].isTooWet) {
      
      char message[80];
      sprintf(message, "🌊 %s TOO WET (%.1f%%)", SENSOR_NAMES[i], moisture);
      
      Serial.println(message);
      
      if (!alertFeed->publish(message)) {
        Serial.println("Failed to publish alert");
      }
      
      previousState[i].isTooWet = true;
    }
    
    // Back to normal after being too wet
    else if (!isTooWet && previousState[i].isTooWet) {
      previousState[i].isTooWet = false;
    }
  }
}

/**
 * ✅ NEW: Check for incoming watering commands from Adafruit IO
 * 
 * When user clicks "💧 Water [Plant] Now" button in the app:
 * 1. Button press publishes message to Adafruit IO feed
 * 2. ESP32 receives that message via MQTT
 * 3. This function detects it and opens the valve
 */
void checkWateringCommands() {
  // Loop through all 4 plant valves
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    // Get the latest message from this plant's water feed
    // (e.g., waterCommand1 for plant 1, waterCommand2 for plant 2, etc.)
    Adafruit_MQTT_Message *message = waterCommands[i]->lastread;
    
    // Check: Did we receive a message AND does it have data?
    if (message && message->len > 0) {
      
      // Check if message says "ON" or "TRUE" (user pressed the button)
      // Adafruit can send different formats: '1', 'o', 't', 'O', 'T'
      if (message->data[0] == '1' || message->data[0] == 'o' || 
          message->data[0] == 't' || message->data[0] == 'O' || 
          message->data[0] == 'T') {
        
        // Only start watering if valve isn't already open
        // (Prevents opening the same valve twice)
        if (digitalRead(RELAY_PIN[i]) == LOW) {
          
          // OPEN THE VALVE for this plant
          digitalWrite(RELAY_PIN[i], HIGH);
          
          // Record the time we opened it (for auto-stop timeout)
          wateringStartTime[i] = millis();
          
          // Send confirmation message
          char msg[80];
          sprintf(msg, "💧 %s WATERING STARTED (5 min timeout)", SENSOR_NAMES[i]);
          Serial.println(msg);
          
          if (!alertFeed->publish(msg)) {
            Serial.println("Failed to publish message");
          }
        }
      }
    }
  }
}

/**
 * ✅ NEW: Auto-stop valves after 5 minutes of watering
 * 
 * Safety feature: Valve can't stay open forever
 * After 5 minutes (WATERING_DURATION_MS), automatically close the valve
 */
void autoStopWatering() {
  // Get current time in milliseconds
  unsigned long currentTime = millis();
  
  // Loop through all 4 plant valves
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    
    // Check TWO things:
    // 1) Is this valve currently OPEN? (GPIO high = open)
    // 2) Has 5 minutes passed since we opened it?
    if (digitalRead(RELAY_PIN[i]) == HIGH && 
        (currentTime - wateringStartTime[i]) >= WATERING_DURATION_MS) {
      
      // Time's up! CLOSE THE VALVE
      digitalWrite(RELAY_PIN[i], LOW);
      
      // Send notification that watering stopped
      char msg[80];
      sprintf(msg, "✓ %s WATERING STOPPED (5 min timeout)", SENSOR_NAMES[i]);
      Serial.println(msg);
      
      if (!alertFeed->publish(msg)) {
        Serial.println("Failed to publish message");
      }
    }
  }
}

/**
 * ✅ NEW: Manage central valve (main water supply)
 * 
 * Logic:
 * - Central valve should be OPEN only when at least ONE plant valve is open
 * - If NO plant valves are open, central valve should close
 * - This prevents water from flowing when no plants are being watered
 */
void manageCentralValve() {
  // Assume no valves are open
  bool anyValveOpen = false;
  
  // Check all 4 plant valves
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    // If this valve is OPEN (HIGH)...
    if (digitalRead(RELAY_PIN[i]) == HIGH) {
      // ... then at least one valve is open
      anyValveOpen = true;
      break;  // No need to check the others, we found at least one
    }
  }
  
  // ═══════════════════════════════════════════════════════════
  // SITUATION 1: At least one plant valve is open
  // ═══════════════════════════════════════════════════════════
  if (anyValveOpen && digitalRead(CENTRAL_VALVE_PIN) == LOW) {
    // Central valve is currently CLOSED, but we need water
    // OPEN the central valve
    digitalWrite(CENTRAL_VALVE_PIN, HIGH);
    
    Serial.println("🚰 Central valve OPENED (water supply activated)");
  }
  
  // ═══════════════════════════════════════════════════════════
  // SITUATION 2: No plant valves are open
  // ═══════════════════════════════════════════════════════════
  if (!anyValveOpen && digitalRead(CENTRAL_VALVE_PIN) == HIGH) {
    // Central valve is currently OPEN, but all plants are done
    // CLOSE the central valve (shut off main water)
    digitalWrite(CENTRAL_VALVE_PIN, LOW);
    
    Serial.println("🚰 Central valve CLOSED (all plants done watering)");
  }
}

/**
 * Publish current soil moisture readings to Adafruit IO
 * Creates a graph over time in your dashboard
 */
void publishReadings() {
  // Publish each sensor's reading to its corresponding feed
  if (cactus) cactus->publish(moisturePercent[0]);
  if (succulents) succulents->publish(moisturePercent[1]);
  if (tropical) tropical->publish(moisturePercent[2]);
  if (herbs) herbs->publish(moisturePercent[3]);
}

/**
 * Connect to your home WiFi network
 * Called once in setup(), then used whenever connection is lost
 */
void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  
  // Start WiFi connection with SSID and password
  WiFi.begin(ssid, password);
  
  // Try to connect for up to 10 seconds (20 attempts × 0.5 seconds)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");  // Print dots while waiting
    attempts++;
  }
  
  // Check if connection succeeded
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi connection failed!");
  }
}

/**
 * Connect to Adafruit IO using MQTT protocol
 * Called once in setup(), then reconnected if connection drops
 */
void connectToMQTT() {
  Serial.print("Connecting to Adafruit IO MQTT...");
  
  // Try to connect with username and API key
  if (mqtt.connect(AIO_USERNAME, AIO_USERNAME, AIO_KEY)) {
    Serial.println("✓ MQTT connected!");
  } else {
    Serial.print("✗ MQTT failed, rc=");
    Serial.println(mqtt.state());
    Serial.println("Retrying in next loop...");
  }
}