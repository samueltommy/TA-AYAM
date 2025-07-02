#include <Wire.h>
#include <WiFi.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
// #include "DFRobot_AirQualitySensor.h"
#include <MHZ19.h>
#include <Adafruit_MAX31865.h>
#include <WiFiManager.h>
#include "esp_task_wdt.h"

#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Firebase
#define API_KEY ""
#define DATABASE_URL ""
#define FIREBASE_EMAIL ""
#define FIREBASE_PASSWORD ""

// Firebase Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
const unsigned long interval = 600000; // 10 menit
bool signupOK = false;

// RS485 Modbus (SHT20)
#define RX_PIN 17  // Sesuaikan dengan pin RX ESP32
#define TX_PIN 16  // Sesuaikan dengan pin TX ESP32
#define DE_PIN 5   // Data Enable (RS485)
#define RE_PIN 4   // Receive Enable (RS485)

// TSL2561 I2C address and registers
#define TSL2561_ADDR 0x39
#define CMD  0x80
#define CTRL 0x00
#define TIMING 0x01
#define CH0_LOW  0x0C
#define CH0_HIGH 0x0D
#define CH1_LOW  0x0E
#define CH1_HIGH 0x0F

#define I2C_LUX_SDA 13
#define I2C_LUX_SCL 12

// #define I2C_PM10_ADDRESS    0x19
// #define I2C_PM10_SDA        39
// #define I2C_PM10_SCL        38

// TwoWire I2C_Second = TwoWire(1); // Create second I2C for PM sensor

#define MHZ19B_RX_PIN 35
#define MHZ19B_TX_PIN 36
#define BAUDRATE 9600 

#define RREF 430.0
#define RNOMINAL 100.0

#define MQ135_PIN 1 // Hubungkan A0 dari Flying Fish ke GPIO34 ESP32-S3
#define RL 10000.0   // Flying Fish pakai RL 10K Ohm
#define CALIBRATION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_TIMES 10
#define READ_SAMPLE_INTERVAL 200

#define MOVING_AVERAGE_WINDOW 10  // Jumlah nilai yang digunakan untuk menghitung rata-rata

HardwareSerial RS485(1);
ModbusMaster node;

// DFRobot_AirQualitySensor particle(&I2C_Second, I2C_PM10_ADDRESS);

MHZ19 myMHZ19;                                             // Constructor for library
HardwareSerial mySerial(2);                                // Create device to MH-Z19 serial (using HardwareSerial 1)

unsigned long lastSensorReadTime = 0;
unsigned long lastFirebaseSendTime = 0;
unsigned long currentMillis;

float temperatureBuffer[MOVING_AVERAGE_WINDOW];
float humidityBuffer[MOVING_AVERAGE_WINDOW];
int bufferIndex = 0;

// Adafruit_MAX31865 rtdSensor = Adafruit_MAX31865(20, 21, 47, 48);  // CS, DI (MOSI), DO (MISO), CLK

float R0 = 5011.24; // Ganti setelah kalibrasi!

float movingAverage(float *buffer) {
    float sum = 0;
    for (int i = 0; i < MOVING_AVERAGE_WINDOW; i++) {
        sum += buffer[i];
    }
    return sum / MOVING_AVERAGE_WINDOW;
}

void addToBuffer(float *buffer, float value) {
    buffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_WINDOW;
}

// Fungsi RS485 Pre/Post Transmission
void preTransmission() {
    digitalWrite(DE_PIN, HIGH);
    digitalWrite(RE_PIN, HIGH);
}

void postTransmission() {
    digitalWrite(DE_PIN, LOW);
    digitalWrite(RE_PIN, LOW);
}

void tsl2561_init() {
  // Power ON
  Wire.beginTransmission(TSL2561_ADDR);
  Wire.write(CMD | CTRL);
  Wire.write(0x03); // Power ON
  Wire.endTransmission();

  // Set integration time = 101ms, gain = 1x
  Wire.beginTransmission(TSL2561_ADDR);
  Wire.write(CMD | TIMING);
  Wire.write(0x01); // 101ms
  Wire.endTransmission();
}

uint16_t read_channel(uint8_t reg_low, uint8_t reg_high) {
  Wire.beginTransmission(TSL2561_ADDR);
  Wire.write(CMD | reg_low);
  Wire.endTransmission();
  Wire.requestFrom(TSL2561_ADDR, 2);
  uint8_t low = Wire.read();

  Wire.beginTransmission(TSL2561_ADDR);
  Wire.write(CMD | reg_high);
  Wire.endTransmission();
  Wire.requestFrom(TSL2561_ADDR, 1);
  uint8_t high = Wire.read();

  return (high << 8) | low;
}

// Convert raw data to approximate lux (simplified formula)
float calculateLux(uint16_t ch0, uint16_t ch1) {
  if (ch0 == 0) return 0;

  float ratio = (float)ch1 / (float)ch0;
  float lux = 0;

  if (ratio <= 0.5)
    lux = 0.0304 * ch0 - 0.062 * ch0 * pow(ratio, 1.4);
  else if (ratio <= 0.61)
    lux = 0.0224 * ch0 - 0.031 * ch1;
  else if (ratio <= 0.80)
    lux = 0.0128 * ch0 - 0.0153 * ch1;
  else if (ratio <= 1.30)
    lux = 0.00146 * ch0 - 0.00112 * ch1;

  return lux;
}

float readTemperature() {
    // Read temperature from SHT20 sensor over RS485 Modbus
    uint8_t result;
    uint16_t data[2];
    result = node.readInputRegisters(0x0001, 2); // Read 2 registers for temperature and humidity
    if (result == node.ku8MBSuccess) {
        data[0] = node.getResponseBuffer(0);  // Temperature in tenths of degrees
        return data[0] / 10.0;  // Convert to Celsius
    } else {
        Serial.println("Failed to read temperature from SHT20.");
        return 0.0;  // Return 0 if reading fails
    }
}

float readHumidity() {
    // Read humidity from SHT20 sensor over RS485 Modbus
    uint8_t result;
    uint16_t data[2];
    result = node.readInputRegisters(0x0001, 2); // Read 2 registers for temperature and humidity
    if (result == node.ku8MBSuccess) {
        data[1] = node.getResponseBuffer(1);  // Humidity in tenths of percentage
        return data[1] / 10.0;  // Convert to percentage
    } else {
        Serial.println("Failed to read humidity from SHT20.");   return 0.0;  // Return 0 if reading fails
    }
}

float readCO2() {
    int CO2 = myMHZ19.getCO2();

    Serial.print("MH-Z19B CO2 OK: ");
    Serial.print(CO2);
    Serial.println(" ppm");

    return CO2;
}

float readLux() {
    // Read lux from TSL2561 sensor
    uint16_t ch0 = read_channel(CH0_LOW, CH0_HIGH);  // Visible + IR light
    uint16_t ch1 = read_channel(CH1_LOW, CH1_HIGH);  // IR light
    return calculateLux(ch0, ch1);  // Calculate lux based on the channels
}

// float readPM1_0() {
//     // Read PM1.0 concentration from the DFRobot Air Quality Sensor
//     return particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE);  // Get PM1.0 concentration in micrograms per cubic meter
// }

// float readPM2_5() {
//     // Read PM2.5 concentration from the DFRobot Air Quality Sensor
//     return particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_ATMOSPHERE);  // Get PM2.5 concentration in micrograms per cubic meter
// }

// float readPM10() {
//     // Read PM10 concentration from the DFRobot Air Quality Sensor
//     return particle.gainParticleConcentration_ugm3(PARTICLE_PM10_ATMOSPHERE);  // Get PM10 concentration in micrograms per cubic meter
// }

// float readRTDTemperature() {
//   uint16_t rtd = rtdSensor.readRTD();

//   Serial.print("RTD value: "); Serial.println(rtd);
//   float ratio = rtd / 32768.0;
//   Serial.print("Ratio = "); Serial.println(ratio, 8);
//   Serial.print("Resistance = "); Serial.println(RREF * ratio, 8);
//   float temp = rtdSensor.temperature(RNOMINAL, RREF);
//   Serial.print("RTD Temperature = "); Serial.println(temp);

//   // Cek error
//   uint8_t fault = rtdSensor.readFault();
//   if (fault) {
//     Serial.print("Fault 0x"); Serial.println(fault, HEX);
//     if (fault & MAX31865_FAULT_HIGHTHRESH) Serial.println("RTD High Threshold");
//     if (fault & MAX31865_FAULT_LOWTHRESH) Serial.println("RTD Low Threshold");
//     if (fault & MAX31865_FAULT_REFINLOW) Serial.println("REFIN- > 0.85 x Bias");
//     if (fault & MAX31865_FAULT_REFINHIGH) Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
//     if (fault & MAX31865_FAULT_RTDINLOW) Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
//     if (fault & MAX31865_FAULT_OVUV) Serial.println("Under/Over voltage");
//     rtdSensor.clearFault();
//     return -999.0; // nilai error
//   }

//   return temp;
// }

float calibrateSensor() {
  float val = 0.0;
  for (int i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {
    int adc = analogRead(MQ135_PIN);
    float voltage = adc * 3.3 / 4095.0;
    float rs = ((3.3 * RL) / voltage) - RL;
    val += rs;
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }

  float rs_air = val / CALIBRATION_SAMPLE_TIMES;
  float r0 = rs_air / 3.6; // Rasio udara bersih Rs/R0 ≈ 3.6
  return r0;
}

float getAmmoniaPPM() {
  float rs = 0.0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    int adc = analogRead(MQ135_PIN);
    float voltage = adc * 3.3 / 4095.0;
    float rs_reading = ((3.3 * RL) / voltage) - RL;
    rs += rs_reading;
    delay(READ_SAMPLE_INTERVAL);
  }

  rs /= READ_SAMPLE_TIMES;
  float ratio = rs / R0;

  float ppm_log = (log10(ratio) - 0.86) / -0.47;
  float ppm = pow(10, ppm_log);

  return ppm;
}

// Fungsi mendapatkan waktu saat ini dalam format YYYY-MM-DD HH:MM:SS
String getCurrentDateTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Gagal mendapatkan waktu");
        return "0000-00-00 00:00:00"; // Format default jika gagal
    }
    char buffer[20]; // YYYY-MM-DD HH:MM:SS
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
}

// Memory usage tracking
void reportMemoryUsage() {
  Serial.println("\n----- MEMORY USAGE REPORT -----");
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.print("Total Heap: ");
  Serial.print(ESP.getHeapSize());
  Serial.println(" bytes");
  
  Serial.print("Heap Usage: ");
  float heapUsagePercent = 100.0 * (1.0 - ((float)ESP.getFreeHeap() / (float)ESP.getHeapSize()));
  Serial.print(heapUsagePercent);
  Serial.println("%");
  
  Serial.print("Minimum Free Heap: ");
  Serial.print(ESP.getMinFreeHeap());
  Serial.println(" bytes");
  
  Serial.print("Maximum Allocated Heap: ");
  Serial.print(ESP.getMaxAllocHeap());
  Serial.println(" bytes");
  
  // Log in CSV format for easier parsing
  Serial.print("MEM,");
  Serial.print(getCurrentDateTime());
  Serial.print(",");
  Serial.print(ESP.getFreeHeap());
  Serial.print(",");
  Serial.print(ESP.getHeapSize());
  Serial.print(",");
  Serial.print(heapUsagePercent);
  Serial.print(",");
  Serial.print(ESP.getMinFreeHeap());
  Serial.print(",");
  Serial.println(ESP.getMaxAllocHeap());
  
  Serial.println("----- END MEMORY REPORT -----\n");
}

// Network usage tracking variables
unsigned long totalBytesSent = 0;
unsigned long totalBytesReceived = 0;
unsigned long lastNetworkReportTime = 0;
const unsigned long networkReportInterval = 600000; // 10 minutes

// Function to track bytes sent in Firebase operations
void trackDataSent(size_t bytesSent) {
  totalBytesSent += bytesSent;
}

// Function to track bytes received in Firebase operations
void trackDataReceived(size_t bytesReceived) {
  totalBytesReceived += bytesReceived;
}

// Report network usage
void reportNetworkUsage() {
  Serial.println("\n----- NETWORK USAGE REPORT -----");
  Serial.print("Total Bytes Sent: ");
  Serial.print(totalBytesSent);
  Serial.println(" bytes");
  
  Serial.print("Total Bytes Received: ");
  Serial.print(totalBytesReceived);
  Serial.println(" bytes");
  
  Serial.print("Total Network Usage: ");
  Serial.print(totalBytesSent + totalBytesReceived);
  Serial.println(" bytes");
  
  // Average bandwidth usage per ten minutes since last report
  unsigned long timeElapsedMinutes = (millis() - lastNetworkReportTime) / 600000;
  if (timeElapsedMinutes > 0) {
    Serial.print("Average Bandwidth: ");
    Serial.print((totalBytesSent + totalBytesReceived) / timeElapsedMinutes);
    Serial.println(" bytes/minute");
  }
  
  // Log in CSV format for easier parsing
  Serial.print("NET,");
  Serial.print(getCurrentDateTime());
  Serial.print(",");
  Serial.print(totalBytesSent);
  Serial.print(",");
  Serial.print(totalBytesReceived);
  Serial.print(",");
  if (timeElapsedMinutes > 0) {
    Serial.print((totalBytesSent + totalBytesReceived) / timeElapsedMinutes);
  } else {
    Serial.print("0");
  }
  Serial.println();
  
  Serial.println("----- END NETWORK REPORT -----\n");
  
  lastNetworkReportTime = millis();
}

// Modified sendToFirebase function to track network usage
void sendToFirebase(const String& path, float value) {
    // Format waktu
    String dateTimeStr = getCurrentDateTime();

    // Membuat JSON
    FirebaseJson jsonData;
    jsonData.set(dateTimeStr, value);
    
    // Get size of data being sent (approximate)
    String jsonStr;
    jsonData.toString(jsonStr);
    size_t dataSizeBytes = path.length() + jsonStr.length() + 20; // Extra bytes for headers, etc.
    
    // Track bytes being sent
    trackDataSent(dataSizeBytes);

    // Mengirim data ke Firebase
    if (Firebase.RTDB.updateNode(&fbdo, path, &jsonData)) {
        Serial.printf("Data sent to Firebase: %s = %.2f\n", path.c_str(), value);
        Serial.print("Time: ");
        Serial.println(dateTimeStr);
        // Approximate received data size (usually just status/confirmation)
        trackDataReceived(100); // Typical response size
    } else {
        Serial.println("Gagal mengirim " + path + " ke Firebase!");
        Serial.println("Error: " + fbdo.errorReason());
    }

    delay(200); // Mikro delay antar kiriman Firebase (200 ms)
}

// Send all current sensor values collectively to /IoT-Current/
void sendCurrentToFirebase(float temperature, float humidity, float CO2, float lux, float ppm) {
    String dateTimeStr = getCurrentDateTime();
    FirebaseJson jsonData;
    jsonData.set("timestamp", dateTimeStr);
    jsonData.set("Temperature", temperature);
    jsonData.set("Humidity", humidity);
    jsonData.set("CO2", CO2);
    jsonData.set("Light", lux);
    // jsonData.set("PM2_5", PM2_5);
    // jsonData.set("PM10", PM10);
    jsonData.set("NH3", ppm);

    // Estimate data size for network tracking
    String jsonStr;
    jsonData.toString(jsonStr);
    size_t dataSizeBytes = String("/IoT-Current").length() + jsonStr.length() + 20;
    trackDataSent(dataSizeBytes);

    if (Firebase.RTDB.setJSON(&fbdo, "/IoT-Current", &jsonData)) {
        Serial.println("Current sensor data sent to /IoT-Current");
        trackDataReceived(100); // Typical response size
    } else {
        Serial.println("Failed to send current sensor data to /IoT-Current");
        Serial.println("Error: " + fbdo.errorReason());
    }
    delay(200);
}

void enterDeepSleep() {
    Serial.println("Memasuki mode tidur untuk menghemat daya...");
    esp_deep_sleep(10 * 1000000);  // 10 detik
}

// Global variables for sensor evaluation
unsigned long latencyStartTime = 0;
bool isWaitingForLatencyResponse = false;
float latencyTimes[10]; // Store last 10 latency measurements
int latencyIndex = 0;

// Variables for standard deviation calculation
#define STATISTICS_SAMPLES 10
float tempSamples[STATISTICS_SAMPLES];
float humiditySamples[STATISTICS_SAMPLES];
float co2Samples[STATISTICS_SAMPLES];
float luxSamples[STATISTICS_SAMPLES];
// float pm1_0Samples[STATISTICS_SAMPLES];
// float pm2_5Samples[STATISTICS_SAMPLES];
// float pm10Samples[STATISTICS_SAMPLES];
// float rtdTempSamples[STATISTICS_SAMPLES];
float nh3Samples[STATISTICS_SAMPLES];
int statIndex = 0;
bool statisticsComplete = false;

// Variables for response time analysis
bool responseTimeTestActive = false;
unsigned long responseTestStartTime = 0;
#define RESPONSE_TEST_SAMPLES 30
float responseValues[RESPONSE_TEST_SAMPLES];
unsigned long responseTimes[RESPONSE_TEST_SAMPLES];
int responseIndex = 0;

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

// Function to export data to serial in CSV format
void exportDataToSerial(float temp, float humidity, float co2, float lux, float nh3) {
  // Print CSV headers if it's the first time
  static bool headersPrinted = false;
  if (!headersPrinted) {
    Serial.println("timestamp,temperature,humidity,co2,lux,nh3");
    headersPrinted = true;
  }
  
  // Get timestamp
  String timestamp = getCurrentDateTime();
  
  // Print CSV format
  Serial.print(timestamp); Serial.print(",");
  Serial.print(temp); Serial.print(",");
  Serial.print(humidity); Serial.print(",");
  Serial.print(co2); Serial.print(",");
  Serial.print(lux); Serial.print(",");
  // Serial.print(pm1_0); Serial.print(",");
  // Serial.print(pm2_5); Serial.print(",");
  // Serial.print(pm10); Serial.print(",");
  // Serial.print(rtdTemp); Serial.print(",");
  Serial.println(nh3);
}

// Collect samples for precision/consistency evaluation
void collectStatisticsSample(float temp, float humidity, float co2, float lux, float nh3) {
  tempSamples[statIndex] = temp;
  humiditySamples[statIndex] = humidity;
  co2Samples[statIndex] = co2;
  luxSamples[statIndex] = lux;
  // pm1_0Samples[statIndex] = pm1_0;
  // pm2_5Samples[statIndex] = pm2_5;
  // pm10Samples[statIndex] = pm10;
  // rtdTempSamples[statIndex] = rtdTemp;
  nh3Samples[statIndex] = nh3;
  
  statIndex++;
  
  Serial.print("Collecting statistics sample ");
  Serial.print(statIndex);
  Serial.print(" of ");
  Serial.println(STATISTICS_SAMPLES);
  
  if (statIndex >= STATISTICS_SAMPLES) {
    calculateAndPrintStatistics();
    statIndex = 0;
    statisticsComplete = true;
  }
}

// Calculate and print statistics
void calculateAndPrintStatistics() {
  Serial.println("\n----- SENSOR PRECISION STATISTICS -----");
  Serial.println("Sensor,Mean,StdDev,StdDev%");
  
  // Calculate for each sensor
  float tempMean = 0, humidityMean = 0, co2Mean = 0, luxMean = 0, nh3Mean = 0;
  
  // Calculate means
  for (int i = 0; i < STATISTICS_SAMPLES; i++) {
    tempMean += tempSamples[i];
    humidityMean += humiditySamples[i];
    co2Mean += co2Samples[i];
    luxMean += luxSamples[i];
    // pm1_0Mean += pm1_0Samples[i];
    // pm2_5Mean += pm2_5Samples[i];
    // pm10Mean += pm10Samples[i];
    // rtdTempMean += rtdTempSamples[i];
    nh3Mean += nh3Samples[i];
  }
  
  tempMean /= STATISTICS_SAMPLES;
  humidityMean /= STATISTICS_SAMPLES;
  co2Mean /= STATISTICS_SAMPLES;
  luxMean /= STATISTICS_SAMPLES;
  // pm1_0Mean /= STATISTICS_SAMPLES;
  // pm2_5Mean /= STATISTICS_SAMPLES;
  // pm10Mean /= STATISTICS_SAMPLES;
  // rtdTempMean /= STATISTICS_SAMPLES;
  nh3Mean /= STATISTICS_SAMPLES;
  
  // Calculate standard deviations
  float tempStdDev = calculateStdDev(tempSamples, STATISTICS_SAMPLES);
  float humidityStdDev = calculateStdDev(humiditySamples, STATISTICS_SAMPLES);
  float co2StdDev = calculateStdDev(co2Samples, STATISTICS_SAMPLES);
  float luxStdDev = calculateStdDev(luxSamples, STATISTICS_SAMPLES);
  // float pm1_0StdDev = calculateStdDev(pm1_0Samples, STATISTICS_SAMPLES);
  // float pm2_5StdDev = calculateStdDev(pm2_5Samples, STATISTICS_SAMPLES);
  // float pm10StdDev = calculateStdDev(pm10Samples, STATISTICS_SAMPLES);
  // float rtdTempStdDev = calculateStdDev(rtdTempSamples, STATISTICS_SAMPLES);
  float nh3StdDev = calculateStdDev(nh3Samples, STATISTICS_SAMPLES);
  
  // Print results
  Serial.print("Temperature,"); Serial.print(tempMean); Serial.print(","); Serial.print(tempStdDev); 
  Serial.print(","); Serial.println((tempStdDev/tempMean)*100);
  
  Serial.print("Humidity,"); Serial.print(humidityMean); Serial.print(","); Serial.print(humidityStdDev);
  Serial.print(","); Serial.println((humidityStdDev/humidityMean)*100);
  
  Serial.print("CO2,"); Serial.print(co2Mean); Serial.print(","); Serial.print(co2StdDev);
  Serial.print(","); Serial.println((co2StdDev/co2Mean)*100);
  
  Serial.print("Light,"); Serial.print(luxMean); Serial.print(","); Serial.print(luxStdDev);
  Serial.print(","); Serial.println((luxStdDev/luxMean)*100);
  
  // Serial.print("PM1.0,"); Serial.print(pm1_0Mean); Serial.print(","); Serial.print(pm1_0StdDev);
  // Serial.print(","); Serial.println((pm1_0StdDev/pm1_0Mean)*100);

  // Serial.print("PM2.5,"); Serial.print(pm2_5Mean); Serial.print(","); Serial.print(pm2_5StdDev);
  // Serial.print(","); Serial.println((pm2_5StdDev/pm2_5Mean)*100);

  // Serial.print("PM10,"); Serial.print(pm10Mean); Serial.print(","); Serial.print(pm10StdDev);
  // Serial.print(","); Serial.println((pm10StdDev/pm10Mean)*100);
  
  // Serial.print("RTD Temp,"); Serial.print(rtdTempMean); Serial.print(","); Serial.print(rtdTempStdDev);
  // Serial.print(","); Serial.println((rtdTempStdDev/rtdTempMean)*100);
  
  Serial.print("NH3,"); Serial.print(nh3Mean); Serial.print(","); Serial.print(nh3StdDev);
  Serial.print(","); Serial.println((nh3StdDev/nh3Mean)*100);
  
  Serial.println("----- END STATISTICS -----\n");
  
  // Also send to Firebase
  String statsPath = "/IoT/statistics/";
  FirebaseJson statJson;
  
  statJson.set("temperature/mean", tempMean);
  statJson.set("temperature/stddev", tempStdDev);
  statJson.set("temperature/stddev_percent", (tempStdDev/tempMean)*100);
  
  statJson.set("humidity/mean", humidityMean);
  statJson.set("humidity/stddev", humidityStdDev);
  statJson.set("humidity/stddev_percent", (humidityStdDev/humidityMean)*100);
  
  statJson.set("co2/mean", co2Mean);
  statJson.set("co2/stddev", co2StdDev);
  statJson.set("co2/stddev_percent", (co2StdDev/co2Mean)*100);
  
  statJson.set("lux/mean", luxMean);
  statJson.set("lux/stddev", luxStdDev);
  statJson.set("lux/stddev_percent", (luxStdDev/luxMean)*100);

  // statJson.set("pm1_0/mean", pm1_0Mean);
  // statJson.set("pm1_0/stddev", pm1_0StdDev);
  // statJson.set("pm1_0/stddev_percent", (pm1_0StdDev/pm1_0Mean)*100);

  // statJson.set("pm2_5/mean", pm2_5Mean);
  // statJson.set("pm2_5/stddev", pm2_5StdDev);
  // statJson.set("pm2_5/stddev_percent", (pm2_5StdDev/pm2_5Mean)*100);
  
  // statJson.set("pm10/mean", pm10Mean);
  // statJson.set("pm10/stddev", pm10StdDev);
  // statJson.set("pm10/stddev_percent", (pm10StdDev/pm10Mean)*100);
  
  // statJson.set("rtdTemp/mean", rtdTempMean);
  // statJson.set("rtdTemp/stddev", rtdTempStdDev);
  // statJson.set("rtdTemp/stddev_percent", (rtdTempStdDev/rtdTempMean)*100);
  
  statJson.set("nh3/mean", nh3Mean);
  statJson.set("nh3/stddev", nh3StdDev);
  statJson.set("nh3/stddev_percent", (nh3StdDev/nh3Mean)*100);
  
  if (Firebase.RTDB.setJSON(&fbdo, statsPath, &statJson)) {
    Serial.println("Statistics data sent to Firebase!");
  } else {
    Serial.println("Failed to send statistics to Firebase");
  }
}

// Start response time test (for a specific sensor)
void startResponseTimeTest(String sensorType) {
  responseTimeTestActive = true;
  responseTestStartTime = millis();
  responseIndex = 0;
  
  Serial.print("Starting response time test for ");
  Serial.println(sensorType);
}

// Record response data point during a test
void recordResponseDataPoint(float value) {
  if (responseTimeTestActive && responseIndex < RESPONSE_TEST_SAMPLES) {
    unsigned long timestamp = millis() - responseTestStartTime;
    
    responseTimes[responseIndex] = timestamp;
    responseValues[responseIndex] = value;
    
    Serial.print("Response test sample ");
    Serial.print(responseIndex + 1);
    Serial.print(" at ");
    Serial.print(timestamp);
    Serial.print("ms: ");
    Serial.println(value);
    
    responseIndex++;
    
    // End test if we've collected enough samples
    if (responseIndex >= RESPONSE_TEST_SAMPLES) {
      completeResponseTimeTest();
    }
  }
}

// Complete response time test and send data to Firebase
void completeResponseTimeTest() {
  responseTimeTestActive = false;
  Serial.println("Response time test complete!");
  Serial.println("Time(ms),Value");
  for (int i = 0; i < responseIndex; i++) {
    Serial.print(responseTimes[i]);
    Serial.print(",");
    Serial.println(responseValues[i]);
  }
  // Perbaikan: deklarasi dataJson dan testPath
  String testPath = "/IoT/responseTest/data";
  FirebaseJson dataJson;
  for (int i = 0; i < responseIndex; i++) {
    String timeKey = String(responseTimes[i]);
    dataJson.set(timeKey, responseValues[i]);
  }
  if (Firebase.RTDB.setJSON(&fbdo, testPath, &dataJson)) {
    Serial.println("Response time data sent to Firebase!");
  } else {
    Serial.println("Failed to send response data to Firebase");
  }
}

// Fungsi untuk menyimpan pengujian latency, memory usage, dan network usage ke Firebase
void saveUsageChickenToFirebase(float lastLatency) {
    String dateTimeStr;
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        char buffer[20]; // YYYYMMDD_HHMMSS
        strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &timeinfo);
        dateTimeStr = String(buffer);
    } else {
        dateTimeStr = String(millis()); // fallback
    }

    // Kirim setiap metric ke path /IoT/<metric> dengan field timestamp
    sendToFirebase("/IoT-Usage/latency", lastLatency);
    sendToFirebase("/IoT-Usage/free_heap", ESP.getFreeHeap());
    sendToFirebase("/IoT-Usage/total_heap", ESP.getHeapSize());
    sendToFirebase("/IoT-Usage/heap_usage_percent", 100.0 * (1.0 - ((float)ESP.getFreeHeap() / (float)ESP.getHeapSize())));
    sendToFirebase("/IoT-Usage/min_free_heap", ESP.getMinFreeHeap());
    sendToFirebase("/IoT-Usage/max_alloc_heap", ESP.getMaxAllocHeap());
    sendToFirebase("/IoT-Usage/total_bytes_sent", totalBytesSent);
    sendToFirebase("/IoT-Usage/total_bytes_received", totalBytesReceived);
    sendToFirebase("/IoT-Usage/wifi_rssi", WiFi.RSSI());
    sendToFirebase("/IoT-Usage/wifi_channel", WiFi.channel());
    sendToFirebase("/IoT-Usage/cpu_freq_mhz", ESP.getCpuFreqMHz());
    sendToFirebase("/IoT-Usage/uptime_seconds", millis() / 1000);
}

// Fungsi untuk mengukur dan menyimpan latency round-trip ke Firebase
void measureAndSaveLatency() {
    // Start latency timer
    unsigned long startMillis = millis();
    String testPath = "/IoT-Usage/latencyTest";
    unsigned long testValue = startMillis;
    String dateTimeStr = getCurrentDateTime();

    // Kirim nilai test ke Firebase
    if (Firebase.RTDB.setInt(&fbdo, testPath, testValue)) {
        // Ambil kembali nilai test dari Firebase
        if (Firebase.RTDB.getInt(&fbdo, testPath)) {
            unsigned long returnedValue = fbdo.intData();
            if (returnedValue == testValue) {
                unsigned long latency = millis() - startMillis;
                Serial.print("Measured latency: ");
                Serial.print(latency);
                Serial.println(" ms");
                // Simpan latency ke Firebase dengan timestamp
                sendToFirebase("/IoT-Usage/latency", (float)latency);
            } else {
                Serial.println("Returned value does not match test value!");
            }
        } else {
            Serial.println("Failed to get latency test value from Firebase");
        }
    } else {
        Serial.println("Failed to send latency test value to Firebase");
    }
}

void connectToWiFi() {
    int retryCount = 0;
    const int maxRetries = 5;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true); // Ensure clean state
    delay(1000);
    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(180); // 3 minutes timeout
    while (!wifiManager.autoConnect("ESP32_AP")) {
        Serial.println("WiFiManager timeout or failed! Restarting ESP32...");
        retryCount++;
        if (retryCount >= maxRetries) {
            Serial.println("Max WiFi retries reached. Restarting...");
            delay(2000);
            ESP.restart();
        }
        delay(2000);
    }
    Serial.println("Connected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("------------");
    Firebase.reconnectWiFi(true); // Always notify Firebase
}

void ensureWiFiAndFirebase() {
    // Try to reconnect WiFi if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi lost! Attempting reconnect...");
        connectToWiFi();
        delay(2000);
    }
    // Try to reconnect Firebase if not ready
    if (!Firebase.ready()) {
        Serial.println("Firebase not ready! Re-initializing...");
        config.api_key = API_KEY;
        config.database_url = DATABASE_URL;
        config.token_status_callback = tokenStatusCallback;
        auth.user.email = FIREBASE_EMAIL;
        auth.user.password = FIREBASE_PASSWORD;
        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true);
        delay(2000);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Memulai ESP32-S3...");
    delay(2000);
    connectToWiFi();

    // Konfigurasi waktu NTP
    // Konfigurasi waktu ke GMT+7
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7

    // Konfigurasi Firebase
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    config.token_status_callback = tokenStatusCallback;
    // Login Firebase dengan Email & Password
    auth.user.email = FIREBASE_EMAIL;
    auth.user.password = FIREBASE_PASSWORD;

    // Inisialisasi Firebase
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    // Inisialisasi UART1 dengan RX, TX untuk RS485
    RS485.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    // Atur pin DE & RE sebagai output
    pinMode(DE_PIN, OUTPUT);
    pinMode(RE_PIN, OUTPUT);
    digitalWrite(DE_PIN, LOW);
    digitalWrite(RE_PIN, LOW);

    // Inisialisasi ModbusMaster
    node.begin(1, RS485); // ID 1 untuk sensor SHT20
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // Init I2C (SDA = 3, SCL = 2)
    Wire.begin(I2C_LUX_SDA, I2C_LUX_SCL);
    tsl2561_init();

    // I2C_Second.begin(I2C_PM10_SDA, I2C_PM10_SCL);
    // Serial.println("Initializing air quality sensor...");
    // while(!particle.begin())
    // {
    //   Serial.println("No Devices ! Check connections for PM sensor");
    //   delay(1000);
    // }
    // Serial.println("Air quality sensor begin success!");

    // uint8_t version = particle.gainVersion();
    // Serial.print("PM sensor version is : ");
    // Serial.println(version);

    // Start Co2
    mySerial.begin(BAUDRATE);                               // Inisialisasi MH-Z19 pada Serial2 (ESP32)
    myMHZ19.begin(mySerial);                                // *Serial(Stream) reference must be passed to library begin().
    myMHZ19.autoCalibration(false);   

    // rtdSensor.begin(MAX31865_3WIRE);

    analogReadResolution(12);

    // Enable watchdog: 30 seconds timeout, panic on trigger
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL); // Add current thread to WDT
}

void loop() {
    esp_task_wdt_reset(); // Feed the watchdog

    currentMillis = millis();

    ensureWiFiAndFirebase();

    // Cek koneksi Wi-Fi setiap 10 detik
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi terputus! Mencoba menyambung kembali...");
        connectToWiFi();
        delay(5000); // Tunggu 5 detik sebelum mencoba kembali
    }

    //KODE UNTUK MEMASTIKAN FIREBASE READY
    if (Firebase.ready() && (millis() - sendDataPrevMillis > interval || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();

        // Read sensor data
        float temperature = readTemperature();
        float humidity = readHumidity();
        float CO2 = readCO2();
        float lux = readLux();
        // float PM1_0 = readPM1_0();
        // float PM2_5 = readPM2_5();
        // float PM10 = readPM10();
        float ppm = getAmmoniaPPM();

        Serial.println("=======================");
        Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
        Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
        Serial.println("=======================");
        Serial.print("Lux: "); Serial.println(lux);
        // Serial.println("=======================");
        // Serial.print("PM1.0 concentration: "); Serial.print(PM1_0); Serial.println(" ug/m3");
        // Serial.println("=======================");
        // Serial.print("PM2.5 concentration: "); Serial.print(PM2_5); Serial.println(" ug/m3");
        // Serial.println("=======================");
        // Serial.print("PM10 concentration: "); Serial.print(PM10); Serial.println(" ug/m3");
        Serial.println("=======================");
        Serial.print("CO2: "); Serial.print(CO2); Serial.println(" ppm");
        Serial.println("=======================");
        Serial.print("NH3: "); Serial.print(ppm); Serial.println(" ppm");
        Serial.println("=======================");

        // Simpan setiap nilai sensor ke path Firebase yang berbeda sesuai sensor
        sendToFirebase("/IoT/Temperature", temperature);
        sendToFirebase("/IoT/Humidity", humidity);
        sendToFirebase("/IoT/CO2", CO2);
        sendToFirebase("/IoT/Light", lux);
        // sendToFirebase("/IoT/PM2_5", PM2_5);
        // sendToFirebase("/IoT/PM10", PM10);
        sendToFirebase("/IoT/NH3", ppm);

        // Send all current sensor values collectively to /IoT-Current
        sendCurrentToFirebase(temperature, humidity, CO2, lux, ppm);

        // Export data to serial in CSV format
        exportDataToSerial(temperature, humidity, CO2, lux, ppm);

        // Collect statistics samples
        collectStatisticsSample(temperature, humidity, CO2, lux, ppm);

        // Measure latency
        measureAndSaveLatency();

        // Report memory usage
        reportMemoryUsage();

        // Report network usage every 10 minutes
        if (millis() - lastNetworkReportTime > networkReportInterval) {
            reportNetworkUsage();
        }
        // Simpan pengujian memory usage, network usage, dan metric teknis ke Firebase
        saveUsageChickenToFirebase(0); // Latency sudah dicatat terpisah

        // Enter deep sleep for energy efficiency (10 minutes)
        esp_sleep_enable_timer_wakeup(interval * 1000); // interval is in ms, convert to us
        Serial.println("Entering deep sleep for 10 minutes...");
        esp_deep_sleep_start();
    }
}
