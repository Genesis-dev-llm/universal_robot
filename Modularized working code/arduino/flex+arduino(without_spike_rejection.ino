#include <WiFi.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// WiFi credentials
const char* ssid = "ESP32_IMU";
const char* password = "12345678";

// Network settings
WiFiServer server(3333);
WiFiClient client;
bool wifiEnabled = false;

// Button pins
#define BASE_ARM_PIN 13        // Base/Arm movement (extend/retract, left/right base rotation)
#define VERTICAL_PIN 12        // Vertical movement (up/down using elbow)
#define TCP_MODE_PIN 27        // TCP/Wrist mode (fine positioning and orientation)

// Flex sensor pin - MUST use ADC1 pin when WiFi is enabled!
// ADC2 pins (GPIO0,2,4,12-15,25-27) don't work with WiFi active
#define FLEX_PIN 33            // GPIO33 - safe with WiFi

BNO080 myIMU;

// Timing intervals
const long SENSOR_READ_INTERVAL = 20;    // 50 Hz for IMU and flex internal calculation
const long FLEX_SEND_INTERVAL = 50;     // 20 Hz for flex transmission update

unsigned long previousMillis = 0;
unsigned long lastFlexUpdate = 0;

// Home positions for IMU
float homeRoll = 0, homePitch = 0, homeYaw = 0;
float homeQI = 0, homeQJ = 0, homeQK = 0, homeQR = 1;
bool isHomed = false;

// Button debouncing for all three buttons
struct ButtonState {
  unsigned long lastChangeTime;
  bool lastState;
  bool currentState;
  bool stableState;
};

ButtonState baseArmButton = {0, HIGH, HIGH, HIGH};
ButtonState verticalButton = {0, HIGH, HIGH, HIGH};
ButtonState tcpModeButton = {0, HIGH, HIGH, HIGH};

const unsigned long debounceDelay = 50;

enum ControlMode {
  IDLE = 0,
  COARSE_TCP_XY = 1,      // Pin 13: Mode 1
  COARSE_TCP_Z = 2,       // Pin 12: Mode 2
  GLOBAL_SWING = 3,       // Pin 27: Mode 3
  PRECISION_TCP_XY = 4,   // Pin 13 + 12: Mode 4
  ACTION_SCREW_GRIP = 5,  // Pin 12 + 27: Mode 5
  TOOL_MIMIC = 6,         // All pins (13+12+27): Mode 6
  GRIPPER_ONLY = 7        // NEW: Mode 7
};

// ============================================================================
// FLEX SENSOR CONFIGURATION
// ============================================================================

// Calibration values (your specific sensor)
int FLAT_ADC = 450;      // ADC value when flex sensor is flat
int BENT_ADC = 1000;     // ADC value when flex sensor is fully bent

// Flex sensor state
int currentFlexPercent = 0;              // Updated every 20ms
int sentFlexPercent = 0;                 // Updated every 50ms (for transmission)

// ============================================================================
// FLEX SENSOR PROCESSING FUNCTION
// ============================================================================

int readAndProcessFlex() {
  // 1. Read raw ADC value
  int raw = analogRead(FLEX_PIN);
  
  // 2. Constrain to calibrated range
  raw = constrain(raw, FLAT_ADC, BENT_ADC);
  
  // 3. Normalize to 0.0 - 1.0
  float normalized = (float)(raw - FLAT_ADC) / (float)(BENT_ADC - FLAT_ADC);
  
  // 4. Apply exponential curve (power of 2.5)
  float curved = pow(normalized, 2.5);
  
  // 5. Convert to percentage (0-100)
  int percentage = (int)(curved * 100.0);
  percentage = constrain(percentage, 0, 100);
  
  return percentage;
}

// ============================================================================
// QUATERNION HELPER FUNCTIONS
// ============================================================================

void quaternionMultiply(float q1w, float q1x, float q1y, float q1z,
                       float q2w, float q2x, float q2y, float q2z,
                       float& outw, float& outx, float& outy, float& outz) {
  outw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z;
  outx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y;
  outy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x;
  outz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w;
}

bool debounceButton(ButtonState& button, int pin) {
  bool reading = (digitalRead(pin) == LOW);
  
  if (reading != button.lastState) {
    button.lastChangeTime = millis();
  }
  
  if ((millis() - button.lastChangeTime) > debounceDelay) {
    if (reading != button.currentState) {
      button.currentState = reading;
      button.stableState = reading;
    }
  }
  
  button.lastState = reading;
  return button.stableState;
}

ControlMode determineControlMode(bool baseArm, bool vertical, bool tcpMode) {
  // 3-button combos (Highest priority)
  if (baseArm && vertical && tcpMode) return TOOL_MIMIC;
  
  // 2-button combos
  if (baseArm && tcpMode) return GRIPPER_ONLY;         // NEW: Mode 7
  if (vertical && tcpMode) return ACTION_SCREW_GRIP;
  if (baseArm && vertical) return PRECISION_TCP_XY;
  
  // 1-button modes
  if (tcpMode) return GLOBAL_SWING;
  if (baseArm) return COARSE_TCP_XY;
  if (vertical) return COARSE_TCP_Z;
  
  return IDLE;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize button pins
  pinMode(BASE_ARM_PIN, INPUT_PULLUP);
  pinMode(VERTICAL_PIN, INPUT_PULLUP);
  pinMode(TCP_MODE_PIN, INPUT_PULLUP);
  
  // Initialize flex sensor pin
  pinMode(FLEX_PIN, INPUT);
  
  sentFlexPercent = 0;  // Start with gripper open
  
  // Initialize I2C for IMU
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  
  // Initialize BNO085 IMU
  if (!myIMU.begin(0x4A, Wire)) {
    Serial.println("# FATAL ERROR: BNO085 not detected.");
    while (1);
  }
  
  myIMU.enableRotationVector(20);
  Serial.println("# BNO085 initialized successfully");
  Serial.println("# Flex sensor initialized (ADC range: " + String(FLAT_ADC) + "-" + String(BENT_ADC) + ")");
  
  // Initialize WiFi Access Point
  if (strlen(ssid) > 0) {
    Serial.print("# Starting WiFi Access Point: ");
    Serial.println(ssid);
    
    WiFi.softAP(ssid, password);
    Serial.print("# AP IP address: ");
    Serial.println(WiFi.softAPIP());
    
    server.begin();
    wifiEnabled = true;
    Serial.println("# System Ready: 3-Button Unified Tool-Centric Control + Flex Sensor");
  } else {
    Serial.println("# System Ready: USB Only Mode");
  }
}

void loop() {
  // Check for new TCP clients
  if (wifiEnabled && (!client || !client.connected())) {
    client = server.available();
    if (client) {
      Serial.println("# Client connected via WiFi TCP");
    }
  }
  
  if (millis() - previousMillis >= SENSOR_READ_INTERVAL) {
    previousMillis = millis();
    
    // ========================================================================
    // ALWAYS read flex sensor (every 20ms)
    // ========================================================================
    currentFlexPercent = readAndProcessFlex();
    
    // Update flex transmission value at 20 Hz (every 50ms)
    if (millis() - lastFlexUpdate >= FLEX_SEND_INTERVAL) {
      sentFlexPercent = currentFlexPercent;
      lastFlexUpdate = millis();
    }
    
    // ========================================================================
    // Debounce all buttons
    // ========================================================================
    bool baseArmPressed = debounceButton(baseArmButton, BASE_ARM_PIN);
    bool verticalPressed = debounceButton(verticalButton, VERTICAL_PIN);
    bool tcpModePressed = debounceButton(tcpModeButton, TCP_MODE_PIN);
    
    // Determine control mode
    ControlMode mode = determineControlMode(baseArmPressed, verticalPressed, tcpModePressed);
    
    // ========================================================================
    // Only process IMU data if any button is pressed
    // ========================================================================
    if (mode != IDLE) {
      if (myIMU.dataAvailable()) {
        
        // Homing logic - Updated for Consistent Data Persistence
        if (!isHomed) {
          // Home EVERYTHING (Euler & Quaternions) on first button press
          // This ensures visualizer has orientation data even in button 1/2/3 modes
          
          // Euler homing
          homeRoll = myIMU.getRoll() * 180.0 / PI;
          homePitch = myIMU.getPitch() * 180.0 / PI;
          homeYaw = myIMU.getYaw() * 180.0 / PI;
          
          // Quaternion homing
          homeQI = myIMU.getQuatI();
          homeQJ = myIMU.getQuatJ();
          homeQK = myIMU.getQuatK();
          homeQR = myIMU.getQuatReal();
          
          isHomed = true;
          Serial.println("# System homed (Euler & Quat)");
        }

        // Get current IMU data
        float currentQI = myIMU.getQuatI();
        float currentQJ = myIMU.getQuatJ();
        float currentQK = myIMU.getQuatK();
        float currentQR = myIMU.getQuatReal();
        
        float currentRoll = myIMU.getRoll() * 180.0 / PI;
        float currentPitch = myIMU.getPitch() * 180.0 / PI;
        float currentYaw = myIMU.getYaw() * 180.0 / PI;
        
        // Calculate relative quaternion (for TCP movements)
        float relQI = 0, relQJ = 0, relQK = 0, relQR = 1;
        // Calculate relative quaternion (for Visualizer & Mode 6)
        if (isHomed) {
          quaternionMultiply(currentQR, currentQI, currentQJ, currentQK,
                            homeQR, -homeQI, -homeQJ, -homeQK,
                            relQR, relQI, relQJ, relQK);
        }
        
        // Calculate relative Euler angles (for base movements)
        float relRoll = 0, relPitch = 0, relYaw = 0;
        // Calculate relative Euler angles (for Modes 1-5)
        if (isHomed) {
          relRoll = currentRoll - homeRoll;
          relPitch = currentPitch - homePitch;
          relYaw = currentYaw - homeYaw;
        }
        
        // ====================================================================
        // Create data string: relQI,relQJ,relQK,relQR,relRoll,relPitch,relYaw,mode,flex
        // ====================================================================
        String dataString = String(relQI, 4) + "," + String(relQJ, 4) + "," + 
                           String(relQK, 4) + "," + String(relQR, 4) + "," +
                           String(relRoll, 2) + "," + String(relPitch, 2) + "," + 
                           String(relYaw, 2) + "," + String(mode) + "," +
                           String(sentFlexPercent);  // ← NEW: 9th value (flex sensor)
        
        // Send data via both USB and WiFi
        Serial.println(dataString);
        if (wifiEnabled && client && client.connected()) {
          client.println(dataString);
        }
      }
    } else {
      // No buttons pressed - reset homing for next activation
      isHomed = false;
    }
  }
}