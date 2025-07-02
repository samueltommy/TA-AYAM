#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Adafruit_MAX31865.h>
#include <time.h>
#include <math.h>
#include <WiFiManager.h>

// Firebase
#define API_KEY ""
#define DATABASE_URL ""
#define FIREBASE_EMAIL ""
#define FIREBASE_PASSWORD ""

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
#define RNOMINAL  100.0

// Analytics definitions
#define STATISTICS_SAMPLES 10
#define MOVING_AVERAGE_WINDOW 5

// Firebase Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Constants
const unsigned long INTERVAL = 600000; // 10 minute
const unsigned long DATA_COLLECTION_INTERVAL = 600000; // 10 minutes
unsigned long sendDataPrevMillis = 0;
unsigned long lastDataCollectionMillis = 0;
unsigned long lastMemoryReport = 0;
unsigned long lastNetworkReport = 0;
const unsigned long MEMORY_REPORT_INTERVAL = 600000; // 10 minutes

// Network statistics
unsigned long totalBytesSent = 0;
unsigned long totalBytesReceived = 0;

// Statistical analysis
float tempSamples[STATISTICS_SAMPLES];
float windSpeedSamples[STATISTICS_SAMPLES];
int statSampleIndex = 0;
bool statsCollectionActive = false;

// Variables for wind speed - EXACTLY as in kecepatan_angin.ino
volatile byte rpmcount; // hitung signals
volatile unsigned long last_micros;
unsigned long timeold;
unsigned long timemeasure = 10.00; // detik
int timetoSleep = 1;               // menit
unsigned long sleepTime = 15;      // menit
unsigned long timeNow;
int countThing = 0;
int GPIO_pulse = 4;               // ESP32 = D14
float rpm, rotasi_per_detik;       // rotasi/detik
float kecepatan_kilometer_per_jam; // kilometer/jam
float kecepatan_meter_per_detik;   //meter/detik
float windSpeedMps = 0.0;   // Wind speed in meters per second
float windSpeedKph = 0.0;   // Wind speed in kilometers per hour
volatile boolean flag = false;

// RTD Sensor - EXACTLY as in suhu-lingkungan.ino
Adafruit_MAX31865 rtdSensor = Adafruit_MAX31865(20, 21, 47, 48); // CS, DI (MOSI), DO (MISO), CLK

// Function Prototypes
void connectToWiFi();
void initializeFirebase();
void initializeRTDSensor();
void calculateWindSpeed();
float readRTDTemperature();
void sendToFirebase(const String &path, float value);
void ICACHE_RAM_ATTR rpm_anemometer();
String getCurrentDateTime();
void trackDataSent(size_t bytes);
void trackDataReceived(size_t bytes);
void collectStatisticsSample(float temperature, float windSpeed);
void reportMemoryUsage();
void reportNetworkUsage();
float calculateStdDev(float values[], int size);
void calculateAndSubmitStatistics();
void saveUsageMetricsToFirebase();
void sendCurrentStateToFirebase(float temperature, float windSpeed);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting ESP32-S3...");

    // Initialize wind speed sensor
    pinMode(GPIO_pulse, INPUT_PULLUP);
    digitalWrite(GPIO_pulse, LOW);
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse));
    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING);
    rpmcount = 0;
    rpm = 0;
    timeold = 0;
    timeNow = 0;

    // Initialize RTD sensor
    Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
    rtdSensor.begin(MAX31865_3WIRE);

    connectToWiFi();
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    initializeFirebase();

    // Hitung wind speed
    calculateWindSpeed();

    // Baca suhu RTD
    float rtdTemp = readRTDTemperature();
    Serial.printf("RTD Temperature: %.2f °C\n", rtdTemp);
    sendToFirebase("/IoT-Env/RTD_Temp", rtdTemp); // timeseries

    // Kirim wind speed
    Serial.printf("Wind Speed: %.2f m/s", windSpeedMps);
    sendToFirebase("/IoT-Env/WindSpeed", windSpeedMps); // timeseries

    // Send all current sensor values collectively to /IoT-Env-Current
    sendCurrentStateToFirebase(rtdTemp, windSpeedMps);

    // Statistik
    collectStatisticsSample(rtdTemp, windSpeedMps);
    calculateAndSubmitStatistics();

    // Laporan memory & network
    reportMemoryUsage();
    reportNetworkUsage();
    saveUsageMetricsToFirebase();

    // Masuk deep sleep selama 10 menit
    Serial.println("Entering deep sleep for 10 minutes...");
    esp_sleep_enable_timer_wakeup(DATA_COLLECTION_INTERVAL * 1000);
    esp_deep_sleep_start();
}

void loop() {
    // Kosong, tidak digunakan
}

void connectToWiFi() {
    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(180); // 5 minutes timeout
    if (!wifiManager.autoConnect("ESP32_ENV_AP")) {
        Serial.println("WiFiManager timeout or failed! Restarting ESP32...");
        delay(1000);
        ESP.restart();
    }
    Serial.println("Connected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("------------");
}

void initializeFirebase() {
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    config.token_status_callback = tokenStatusCallback;
    
    // Sign up with email and password
    auth.user.email = FIREBASE_EMAIL;
    auth.user.password = FIREBASE_PASSWORD;
    
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

float readRTDTemperature() {
    // Implementation EXACTLY as in suhu-lingkungan.ino
    uint16_t rtd = rtdSensor.readRTD();
    float ratio = rtd;
    ratio /= 32768;
    float temperature = rtdSensor.temperature(RNOMINAL, RREF);

    // Check and print any faults - exactly as in suhu-lingkungan.ino
    uint8_t fault = rtdSensor.readFault();
    if (fault) {
      Serial.print("Fault 0x"); Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
      }
      rtdSensor.clearFault();
      return -999.0; // Return error value
    }
    return temperature;
}

void sendToFirebase(const String &path, float value) {
    // Format time
    String dateTimeStr = getCurrentDateTime();

    // Create JSON
    FirebaseJson jsonData;
    jsonData.set(dateTimeStr, value);
    
    // Get size of data being sent (approximate)
    String jsonStr;
    jsonData.toString(jsonStr);
    size_t dataSizeBytes = path.length() + jsonStr.length() + 20; // Extra bytes for headers, etc.
    
    // Track bytes being sent
    trackDataSent(dataSizeBytes);

    // Send data to Firebase
    if (Firebase.RTDB.updateNode(&fbdo, path, &jsonData)) {
        Serial.printf("Data sent to Firebase: %s = %.2f\n", path.c_str(), value);
        Serial.println("Time: " + dateTimeStr);
        
        // Approximate received data size (usually just status/confirmation)
        trackDataReceived(100); // Typical response size
    } else {
        Serial.printf("Failed to send data to Firebase: %s\nError: %s\n", path.c_str(), fbdo.errorReason().c_str());
    }
    
    delay(200); // Small delay between Firebase sends (200 ms)
}

void calculateWindSpeed() {
    // Implementation EXACTLY as in kecepatan_angin.ino
    if (flag == true)
    {
      if (long(micros() - last_micros) >= 5000)
      {
        rpmcount++;
        last_micros = micros();
      }
      flag = false; // reset flag
    }

    if ((millis() - timeold) >= timemeasure * 1000)
    {
      countThing++;
      detachInterrupt(digitalPinToInterrupt(GPIO_pulse));      // Menonaktifkan interrupt saat menghitung
      rotasi_per_detik = float(rpmcount) / float(timemeasure); // rotasi per detik
      //kecepatan_meter_per_detik = rotasi_per_detik; // rotasi/detik sebelum dikalibrasi untuk dijadikan meter per detik
      kecepatan_meter_per_detik = ((-0.0181 * (rotasi_per_detik * rotasi_per_detik)) + (1.3859 * rotasi_per_detik) + 1.4055); // meter/detik sesudah dikalibrasi dan sudah dijadikan meter per detik
      if (kecepatan_meter_per_detik <= 1.5)
      { // Minimum pembacaan sensor kecepatan angin adalah 1.5 meter/detik
        kecepatan_meter_per_detik = 0.0;
      }
      kecepatan_kilometer_per_jam = kecepatan_meter_per_detik * 3.6; // kilometer/jam
      
      // Update global variables for use in the rest of the program
      windSpeedMps = kecepatan_meter_per_detik;
      windSpeedKph = kecepatan_kilometer_per_jam;
      
      Serial.print("kecepatan meter per detik="); // Minimal kecepatan angin yang dapat dibaca sensor adalah 4 meter/detik dan maksimum 30 meter/detik.
      Serial.print(kecepatan_meter_per_detik);
     
      timeold = millis();
      rpmcount = 0;
      attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // enable interrupt
    }
}

void ICACHE_RAM_ATTR rpm_anemometer() {
    // Implementation EXACTLY as in kecepatan_angin.ino
    flag = true;
}

String getCurrentDateTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return "";
    }
    char buffer[20];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
}

void trackDataSent(size_t bytes) {
    totalBytesSent += bytes;
    Serial.printf("Total bytes sent: %lu\n", totalBytesSent);
}

void trackDataReceived(size_t bytes) {
    totalBytesReceived += bytes;
    Serial.printf("Total bytes received: %lu\n", totalBytesReceived);
}

void collectStatisticsSample(float temperature, float windSpeed) {
    tempSamples[statSampleIndex] = temperature;
    windSpeedSamples[statSampleIndex] = windSpeed;
    statSampleIndex = (statSampleIndex + 1) % STATISTICS_SAMPLES;
}

void reportMemoryUsage() {
    Serial.printf("Free heap memory: %u bytes\n", ESP.getFreeHeap());
}

void reportNetworkUsage() {
    Serial.printf("Total bytes sent: %lu\n", totalBytesSent);
    Serial.printf("Total bytes received: %lu\n", totalBytesReceived);
}

// Calculate standard deviation from array of values
float calculateStdDev(float values[], int size) {
  float sum = 0.0;
  float mean;
  float stdDev = 0.0;
  
  // Calculate mean
  for (int i = 0; i < size; i++) {
    sum += values[i];
  }
  mean = sum / size;
  
  // Calculate standard deviation
  for (int i = 0; i < size; i++) {
    stdDev += pow(values[i] - mean, 2);
  }
  
  return sqrt(stdDev / size);
}

// Calculate and submit statistics to Firebase
void calculateAndSubmitStatistics() {
  // Skip if we don't have enough samples yet
  if (!statsCollectionActive && statSampleIndex < STATISTICS_SAMPLES) {
    Serial.println("Not enough samples for statistics yet");
    return;
  }

  Serial.println("\n----- SENSOR STATISTICS -----");
  
  // Calculate means
  float tempMean = 0.0;
  float windSpeedMean = 0.0;
  
  for (int i = 0; i < STATISTICS_SAMPLES; i++) {
    tempMean += tempSamples[i];
    windSpeedMean += windSpeedSamples[i];
  }
  
  tempMean /= STATISTICS_SAMPLES;
  windSpeedMean /= STATISTICS_SAMPLES;
  
  // Calculate standard deviations
  float tempStdDev = calculateStdDev(tempSamples, STATISTICS_SAMPLES);
  float windSpeedStdDev = calculateStdDev(windSpeedSamples, STATISTICS_SAMPLES);
  
  // Print statistics
  Serial.printf("Temperature: Mean=%.2f°C, StdDev=%.2f°C, CV=%.2f%%\n", 
               tempMean, tempStdDev, (tempStdDev/tempMean)*100);
  Serial.printf("Wind Speed: Mean=%.2fm/s, StdDev=%.2fm/s, CV=%.2f%%\n", 
               windSpeedMean, windSpeedStdDev, (windSpeedStdDev/windSpeedMean)*100);
  
  // Send to Firebase
  FirebaseJson statJson;
  String statsPath = "/IoT-Env/Statistics";
  String timestamp = getCurrentDateTime();
  
  statJson.set("timestamp", timestamp);
  statJson.set("temperature/mean", tempMean);
  statJson.set("temperature/stddev", tempStdDev);
  statJson.set("temperature/cv_percent", (tempStdDev/tempMean)*100);
  
  statJson.set("windSpeed/mean", windSpeedMean);
  statJson.set("windSpeed/stddev", windSpeedStdDev);
  statJson.set("windSpeed/cv_percent", (windSpeedStdDev/windSpeedMean)*100);
  
  if (Firebase.RTDB.setJSON(&fbdo, statsPath, &statJson)) {
    Serial.println("Statistics sent to Firebase successfully");
  } else {
    Serial.printf("Failed to send statistics to Firebase: %s\n", fbdo.errorReason().c_str());
  }
  
  statsCollectionActive = true;
  Serial.println("----- END STATISTICS -----\n");
}

void saveUsageMetricsToFirebase() {
    String dateTimeStr = getCurrentDateTime();
    // Memory
    sendToFirebase("/IoT-Env-Usage/free_heap", ESP.getFreeHeap());
    sendToFirebase("/IoT-Env-Usage/total_heap", ESP.getHeapSize());
    sendToFirebase("/IoT-Env-Usage/heap_usage_percent", 100.0 * (1.0 - ((float)ESP.getFreeHeap() / (float)ESP.getHeapSize())));
    sendToFirebase("/IoT-Env-Usage/min_free_heap", ESP.getMinFreeHeap());
    sendToFirebase("/IoT-Env-Usage/max_alloc_heap", ESP.getMaxAllocHeap());
    // Network
    sendToFirebase("/IoT-Env-Usage/total_bytes_sent", totalBytesSent);
    sendToFirebase("/IoT-Env-Usage/total_bytes_received", totalBytesReceived);
    sendToFirebase("/IoT-Env-Usage/wifi_rssi", WiFi.RSSI());
    sendToFirebase("/IoT-Env-Usage/wifi_channel", WiFi.channel());
    // CPU & uptime
    sendToFirebase("/IoT-Env-Usage/cpu_freq_mhz", ESP.getCpuFreqMHz());
    sendToFirebase("/IoT-Env-Usage/uptime_seconds", millis() / 1000);
}

// Send all current sensor values collectively to /IoT-Env-Current
void sendCurrentStateToFirebase(float temperature, float windSpeed) {
    String dateTimeStr = getCurrentDateTime();
    FirebaseJson jsonData;
    jsonData.set("timestamp", dateTimeStr);
    jsonData.set("RTD_Temp", temperature);
    jsonData.set("WindSpeed", windSpeed);

    // Estimate data size for network tracking
    String jsonStr;
    jsonData.toString(jsonStr);
    size_t dataSizeBytes = String("/IoT-Env-Current").length() + jsonStr.length() + 20;
    trackDataSent(dataSizeBytes);

    if (Firebase.RTDB.setJSON(&fbdo, "/IoT-Env-Current", &jsonData)) {
        Serial.println("Current state sent to Firebase successfully");
        trackDataReceived(100);
    } else {
        Serial.printf("Failed to send current state to Firebase: %s\n", fbdo.errorReason().c_str());
    }
    delay(200);
}

