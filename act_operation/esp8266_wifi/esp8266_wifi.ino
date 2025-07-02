#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Arduino_JSON.h>
#include <WiFiManager.h>

// Firebase Credentials
#define API_KEY ""
#define DATABASE_URL ""
#define FIREBASE_EMAIL ""
#define FIREBASE_PASSWORD ""

// Firebase Objects
FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig config;

// Local state
int heaterValue = 0;
int inlineValue = 0;
int exhaustValue = 0;
int augerValue = 0; // 0 = off, >0 = seconds remaining
JSONVar JSON_All_Data_Received;

// Add these global variables to track last uploaded state
int lastHeaterValue = -1;
int lastInlineValue = -1;
int lastExhaustValue = -1;
int lastAugerValue = -1;

// Error handling
int firebaseErrorCount = 0;
const int firebaseErrorThreshold = 5;

// Stream callbacks
void streamCallback(FirebaseStream data);
void streamTimeoutCallback(bool timeout);
void sendCommandToArduino(const String& command);
void uploadStateToFirebase(const String& jsonStr);
void connectToWiFi();
String getCurrentDateTime();

unsigned long lastFetchTime = 0;
const unsigned long fetchInterval = 5000; // 5 seconds

// --- Firebase Connection Setup (like gabung.ino) ---
void setup() {
  Serial.begin(115200);
  Serial.println("Memulai ESP8266...");
  delay(2000);
  connectToWiFi();

  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  auth.user.email = FIREBASE_EMAIL;
  auth.user.password = FIREBASE_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  ESP.wdtEnable(30000); // Enable hardware watchdog, 30s timeout
}

void loop() {
  ESP.wdtFeed(); // Feed the watchdog
  ensureWiFiAndFirebase();
  yield(); // Allow background tasks
  // Listen for Arduino feedback
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (data.startsWith("STATE:")) {
        String jsonStr = data.substring(6);
        JSONVar doc = JSON.parse(jsonStr);

        if (doc.hasOwnProperty("Heater")) heaterValue = (int)doc["Heater"];
        if (doc.hasOwnProperty("Inline")) inlineValue = (int)doc["Inline"];
        if (doc.hasOwnProperty("Exhaust")) exhaustValue = (int)doc["Exhaust"];
        if (doc.hasOwnProperty("Auger")) augerValue = (int)doc["Auger"];

        Serial.println("Received STATE from Arduino:");
        Serial.println(jsonStr);

        // Only upload if state changed
        if ((doc.hasOwnProperty("Heater") && heaterValue != lastHeaterValue) ||
            (doc.hasOwnProperty("Inline") && inlineValue != lastInlineValue) ||
            (doc.hasOwnProperty("Exhaust") && exhaustValue != lastExhaustValue) ||
            (doc.hasOwnProperty("Auger") && augerValue != lastAugerValue)) {
            uploadStateToFirebase(jsonStr);
            lastHeaterValue = heaterValue;
            lastInlineValue = inlineValue;
            lastExhaustValue = exhaustValue;
            lastAugerValue = augerValue;
        }
    }
  }

  // Periodically fetch latest data from Firebase
  if (millis() - lastFetchTime > fetchInterval) {
    fetchLatestActuatorData();
    lastFetchTime = millis();
  }
}

void streamCallback(FirebaseStream data) {
  Serial.println("\nStream Data Received:");
  Serial.println(data.dataPath());
  Serial.println(data.dataType());

  if (data.dataType() == "json") {
    Serial.println("Actuator JSON from stream:");
    Serial.println(data.jsonString());

    FirebaseJson json;
    json.setJsonData(data.jsonString());
    FirebaseJsonData result;

    json.get(result, "Heater");
    if (result.success) {
      int HeaterValue = result.to<int>();
      sendCommandToArduino("heater:" + String(HeaterValue));
    }

    json.get(result, "Inline");
    if (result.success) {
      int speed = result.to<int>();
      sendCommandToArduino("inline:" + String(speed));
    }

    json.get(result, "Exhaust");
    if (result.success) {
      int speed = result.to<int>();
      sendCommandToArduino("exhaust:" + String(speed));
    }

    json.get(result, "Auger");
    if (result.success) {
      int augerValue = 0;
      bool found = false;
      if (String(result.type) == "int" || String(result.type) == "double") {
        augerValue = result.to<int>();
        found = true;
      } else if (String(result.type) == "object") {
        FirebaseJson augerObj;
        augerObj.setJsonData(result.to<const char*>());
        FirebaseJsonData valueField;
        if (augerObj.get(valueField, "value") && valueField.success) {
          augerValue = valueField.to<int>();
          found = true;
        }
      }
      if (found && augerValue > 0) {
        sendCommandToArduino("auger:" + String(augerValue));
      }
    }
  }
}

void streamTimeoutCallback(bool timeout) {
  if (timeout) {
    Serial.println("Stream timeout. Attempting to reconnect...");
  }
}

void sendCommandToArduino(const String& command) {
  Serial.println("CMD:" + command);
}

void connectToWiFi() {
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180); // 3 minutes timeout
  if (!wifiManager.autoConnect("ESP8266_AP")) {
    Serial.println("WiFiManager timeout or failed! Restarting ESP8266...");
    delay(2000);
    ESP.restart();
  }
  Serial.println("Connected! IP: " + WiFi.localIP().toString());
}

void uploadStateToFirebase(const String& jsonStr) {
  if (!Firebase.ready()) return;

  FirebaseJson json;
  json.setJsonData(jsonStr);

  // Store current state in new format: /actuator-state
  String dateTimeStr = getCurrentDateTime();
  FirebaseJson currentStateJson;
  FirebaseJsonData result;

  // Heater
  json.get(result, "Heater");
  if (result.success) currentStateJson.set("Heater", result.to<int>());
  // Inline
  json.get(result, "Inline");
  if (result.success) currentStateJson.set("Inline", result.to<int>());
  // Exhaust
  json.get(result, "Exhaust");
  if (result.success) currentStateJson.set("Exhaust", result.to<int>());
  // Auger
  json.get(result, "Auger");
  if (result.success) currentStateJson.set("Auger", result.to<int>());
  // Timestamp
  currentStateJson.set("timestamp", dateTimeStr);

  if (Firebase.RTDB.setJSON(&fbdo, "/actuator-state", &currentStateJson)) {
    Serial.println("Current state uploaded to /actuator-state");
  } else {
    Serial.println("Upload to /actuator-state failed: " + fbdo.errorReason());
  }

  // Store timeline (timeseries) data for each actuator separately
  // Heater
  json.get(result, "Heater");
  if (result.success) {
    Firebase.RTDB.setInt(&fbdo, "/actuator-state/Heater/" + dateTimeStr, result.to<int>());
  }
  // Inline
  json.get(result, "Inline");
  if (result.success) {
    Firebase.RTDB.setInt(&fbdo, "/actuator-state/Inline/" + dateTimeStr, result.to<int>());
  }
  // Exhaust
  json.get(result, "Exhaust");
  if (result.success) {
    Firebase.RTDB.setInt(&fbdo, "/actuator-state/Exhaust/" + dateTimeStr, result.to<int>());
  }
  // Auger
  json.get(result, "Auger");
  if (result.success) {
    Firebase.RTDB.setInt(&fbdo, "/actuator-state/Auger/" + dateTimeStr, result.to<int>());
  }
}

String getCurrentDateTime() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(buffer);
}

void fetchLatestActuatorData() {
  if (!Firebase.ready()) return;
  if (Firebase.RTDB.getJSON(&fbdo, "/actuators-current")) {
    FirebaseJson json;
    json.setJsonData(fbdo.jsonString());
    FirebaseJsonData result;

    // Heater
    json.get(result, "Heater");
    if (result.success) {
      String heaterState = String(result.to<const char*>());
      if (heaterState == "on" || heaterState == "ON" || heaterState == "1") {
        Serial.println("CMD:heater:on");
      } else {
        Serial.println("CMD:heater:off");
      }
    }

    // Inline
    json.get(result, "Inline");
    if (result.success) {
      int inlineValue = 0;
      if (result.type == "int" || result.type == "double") {
        inlineValue = result.to<int>();
      } else if (result.type == "string") {
        inlineValue = String(result.to<const char*>()).toInt();
      }
      Serial.println("CMD:inline:" + String(inlineValue));
    }

    // Exhaust
    json.get(result, "Exhaust");
    if (result.success) {
      int exhaustValue = 0;
      if (result.type == "int" || result.type == "double") {
        exhaustValue = result.to<int>();
      } else if (result.type == "string") {
        exhaustValue = String(result.to<const char*>()).toInt();
      }
      Serial.println("CMD:exhaust:" + String(exhaustValue));
    }

    // Auger
    json.get(result, "Auger");
    if (result.success) {
      int augerValue = 0;
      if (result.type == "int" || result.type == "double") {
        augerValue = result.to<int>();
      } else if (result.type == "string") {
        augerValue = String(result.to<const char*>()).toInt();
      }
      if (augerValue > 0) {
        Serial.println("CMD:auger:" + String(augerValue));
      }
    }
  }
}

// Token status callback for Firebase authentication
void tokenStatusCallback(TokenInfo info) {
  // Print basic token status info (compatible with ESP8266/ESP32 Firebase library)
  Serial.print("Token info: type = ");
  Serial.print(info.type);
  Serial.print(", status = ");
  Serial.println(info.status);
}

void ensureWiFiAndFirebase() {
  // Ensure WiFi is always connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost! Attempting reconnect...");
    connectToWiFi();
    delay(2000);
  }
  // Ensure Firebase is always connected
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready! Re-initializing...");
    // Removed Firebase.end(&fbdo); as it does not exist in Firebase_ESP_Client
    delay(500);
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    config.token_status_callback = tokenStatusCallback;
    auth.user.email = FIREBASE_EMAIL;
    auth.user.password = FIREBASE_PASSWORD;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    delay(2000);
    firebaseErrorCount++;
    if (firebaseErrorCount >= firebaseErrorThreshold) {
      Serial.println("Too many Firebase errors, restarting ESP8266...");
      delay(2000);
      ESP.restart();
    }
  } else {
    firebaseErrorCount = 0; // Reset on success
  }
}