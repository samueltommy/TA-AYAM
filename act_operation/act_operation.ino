#include <Arduino.h>
#include <avr/wdt.h> // Include AVR watchdog header

// Define pins for Fan Control (BTS7960)
const int RPWM_PIN1 = 4;  // RPWM pin of BTS7960 for Fan 1
const int LPWM_PIN1 = 5;  // LPWM pin of BTS7960 for Fan 1
const int RPWM_PIN2 = 8;  // RPWM pin of BTS7960 for Fan 2
const int LPWM_PIN2 = 9;  // LPWM pin of BTS7960 for Fan 2

// Define pin for Heater Relay
#define HEATER_RELAY_PIN 2  // Relay control pin for heater

// Auger relay pin and state
#define AUGER_RELAY_PIN 7

// System state variables
int heaterValue = 0;
int inlineValue = 0;
int exhaustValue = 0;
int augerValue = 0;

// Timer variables
unsigned long previousMillis = 0;
const long updateInterval = 100; // Reduced to 100ms for near real-time updates

// Add these global variables to track last sent state
int lastHeaterValue = -1;
int lastInlineValue = -1;
int lastExhaustValue = -1;
int lastAugerValue = -1; // Track auger state

// Auger control variables
bool augerOn = false;
int augerDuration = 0;
unsigned long augerStartTime = 0;

String simpleInput = "";

// Function to control fans based on speed only (one direction)
void controlFan(const String& fanName, int speed) {
    // Map speed percentage (0-100) to PWM values (0-255)
    speed = constrain(speed, 0, 100);
    int pwmValue = map(speed, 0, 100, 0, 255);
    Serial.print("Controlling Fan ");
    Serial.print(fanName);
    Serial.print(", Speed: ");
    Serial.print(speed);
    Serial.print("%, PWM Value: ");
    Serial.println(pwmValue);
    if (fanName == "inline") {
        analogWrite(LPWM_PIN1, pwmValue);
        analogWrite(RPWM_PIN1, 0); // Always off
    } else if (fanName == "exhaust") {
        analogWrite(LPWM_PIN2, pwmValue);
        analogWrite(RPWM_PIN2, 0); // Always off
    }
}

// Function to send current state to Serial Monitor
void sendStateUpdate() {
    // Only send if state changed
    if (heaterValue != lastHeaterValue || inlineValue != lastInlineValue || exhaustValue != lastExhaustValue || augerValue != lastAugerValue) {
        String stateMsg = "STATE:";
        stateMsg += "{\"Heater\":";
        stateMsg += heaterValue;
        stateMsg += ",\"Inline\":";
        stateMsg += inlineValue;
        stateMsg += ",\"Exhaust\":";
        stateMsg += exhaustValue;
        stateMsg += ",\"Auger\":";
        stateMsg += augerValue;
        stateMsg += "}";

        Serial.println(stateMsg);
        lastHeaterValue = heaterValue;
        lastInlineValue = inlineValue;
        lastExhaustValue = exhaustValue;
        lastAugerValue = augerValue;
    }
}

void processCommand(String command) {
    int colonIndex = command.indexOf(':');
    if (colonIndex == -1) return;
    String device = command.substring(0, colonIndex);
    String value = command.substring(colonIndex + 1);
    value.trim();
    value.toLowerCase();
    if (device == "heater") {
        // Only ON or OFF, no value
        if (value == "on") {
            digitalWrite(HEATER_RELAY_PIN, LOW); // Turn relay ON (correct logic)
            heaterValue = 1;
            Serial.println("Heater relay ON");
        } else if (value == "off") {
            digitalWrite(HEATER_RELAY_PIN, HIGH); // Turn relay OFF (correct logic)
            heaterValue = 0;
            Serial.println("Heater relay OFF");
        } else {
            Serial.println("Invalid heater command. Use 'on' or 'off'.");
        }
    }    
    else if (device == "inline") {
        int val = value.toInt();
        inlineValue = constrain(val, 0, 100);
        Serial.print("Inline speed set to ");
        Serial.print(inlineValue);
        Serial.println("%");
        controlFan("inline", inlineValue);
    }
    else if (device == "exhaust") {
        int val = value.toInt();
        exhaustValue = constrain(val, 0, 100);
        Serial.print("Exhaust speed set to ");
        Serial.print(exhaustValue);
        Serial.println("%");
        controlFan("exhaust", exhaustValue);
    }
    else if (device == "auger") {
        int val = value.toInt();
        if (val > 0) {
            augerDuration = val;
            augerOn = true;
            augerStartTime = millis();
            digitalWrite(AUGER_RELAY_PIN, LOW); // Turn ON auger (correct logic)
            augerValue = val; // Set auger state for reporting
            Serial.print("Auger ON for ");
            Serial.print(augerDuration);
            Serial.println(" seconds (CMD)");
        } else {
            Serial.println("Invalid auger command. Use a positive number of seconds.");
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("Arduino Ready");

    // Setup pins for Fan Control
    pinMode(RPWM_PIN1, OUTPUT);
    pinMode(LPWM_PIN1, OUTPUT);
    pinMode(RPWM_PIN2, OUTPUT);
    pinMode(LPWM_PIN2, OUTPUT);
    
    // Initialize fans to stopped state
    analogWrite(RPWM_PIN1, 0);
    analogWrite(LPWM_PIN1, 0);
    analogWrite(RPWM_PIN2, 0);
    analogWrite(LPWM_PIN2, 0);

    // Setup relay pin for heater
    pinMode(HEATER_RELAY_PIN, OUTPUT);
    digitalWrite(HEATER_RELAY_PIN, HIGH); // Ensure relay is OFF at startup
    heaterValue = 0; // Track heater state as OFF

    // Setup pin for auger relay
    pinMode(AUGER_RELAY_PIN, OUTPUT);
    digitalWrite(AUGER_RELAY_PIN, HIGH); // OFF by default

    wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout
}

void loop() {
    wdt_reset(); // Feed the watchdog

    // Simulating command processing via Serial Monitor
    while (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        data.trim();
        if (data.startsWith("CMD:")) {
            String cmd = data.substring(4);
            // If structured command, process as structured
            processCommand(cmd);
        } else if (data == "heateron") {
            digitalWrite(HEATER_RELAY_PIN, LOW); // Corrected to HIGH for turning on
            heaterValue = 1;
            Serial.println("Heater relay ON (simple)");
        } else if (data == "heateroff") {
            digitalWrite(HEATER_RELAY_PIN, HIGH); // Corrected to LOW for turning off
            heaterValue = 0;
            Serial.println("Heater relay OFF (simple)");
        }
    }

    // Auger timer logic
    if (augerOn) {
        if (millis() - augerStartTime >= augerDuration * 1000UL) {
            digitalWrite(AUGER_RELAY_PIN, HIGH); // Turn OFF auger after specified duration
            Serial.println("Auger OFF (CMD)");
            augerOn = false;
            augerValue = 0; // Set auger state to 0 when off
        }
    } else {
        digitalWrite(AUGER_RELAY_PIN, HIGH); // Always keep OFF when not active
        augerValue = 0; // Ensure auger state is 0 when not running
    }

    // Send periodic state updates
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= updateInterval) {
        previousMillis = currentMillis;
        sendStateUpdate();
    }
}
