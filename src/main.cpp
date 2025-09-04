#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Add FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Servo control parameters
#define SERVO_PIN 1         // GPIO 1 for servo control
#define LEDC_CHANNEL 0      // Using LEDC channel 0
#define LEDC_TIMER_BIT 12   // 12-bit resolution (0-4095)
#define LEDC_BASE_FREQ 50   // 50Hz PWM frequency

// LED status indicators
#define LED_PIN LED_BUILTIN // Built-in LED on XIAO-ESP32S3
#define LED_AP_MODE 1000    // AP mode - slow blink (1000ms)
#define LED_WIFI_CONNECTING 250 // WiFi connecting - fast blink (250ms)
#define LED_SERVO_ACTIVE 50 // Servo movement - very short blink (50ms)

// Servo angle PWM values
#define SERVO_MIN_PULSE 103   // 0.5ms/20ms * 4095 ≈ 103 (0 degrees)
#define SERVO_MID_PULSE 307   // 1.5ms/20ms * 4095 ≈ 307 (90 degrees)
#define SERVO_MAX_PULSE 512   // 2.5ms/20ms * 4095 ≈ 512 (180 degrees)

// I2C pins for MPU6050
#define SDA_PIN 5  // GPIO5 for SDA according to XIAO ESP32S3 pinout
#define SCL_PIN 6  // GPIO6 for SCL according to XIAO ESP32S3 pinout

// WiFi configuration
#define EEPROM_SIZE 512
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64

// FreeRTOS task definitions
#define WEBSERVER_TASK_PRIORITY 3
#define MPU_TASK_PRIORITY 2
#define LED_TASK_PRIORITY 1
#define STACK_SIZE 8192

// Web server
WebServer server(80);

// Global variables
int currentServoAngle = 90;  // Current servo angle
String apSSID = "ServoControl_AP";
String apPassword = "12345678";
bool wifiConfigured = false;
unsigned long lastLedToggle = 0;
int ledBlinkInterval = 1000; // Default blink interval
bool ledState = false;       // LED state
bool restartPending = false;
unsigned long restartTime = 0;

// MPU6050 variables
Adafruit_MPU6050 mpu;
float measuredAngle = 90;           // Initial value matching default servo angle
const float FILTER_FACTOR = 0.25;    // Low-pass filter coefficient (0-1)
bool mpuInitialized = false;        // MPU6050 initialization status

// FreeRTOS synchronization
SemaphoreHandle_t angleMutex;
SemaphoreHandle_t wifiMutex;
SemaphoreHandle_t ledMutex;

// Structure to store WiFi configuration
struct {
  char ssid[MAX_SSID_LENGTH] = {0};
  char password[MAX_PASSWORD_LENGTH] = {0};
  bool configured = false;
} wifiConfig;

// Function prototypes
void webServerTask(void *parameter);
void mpuTask(void *parameter);
void ledControlTask(void *parameter);
void setServoAngle(int angle);
float readMPUAngle();
bool initMPU6050();
float mapFloat();

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to read a file from SPIFFS
String readFile(const char* path) {
  File file = SPIFFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while (file.available()) {
    fileContent += (char)file.read();
  }
  file.close();
  return fileContent;
}

// LED control functions
void setLedMode(int blinkInterval) {
  if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
    ledBlinkInterval = blinkInterval;
    lastLedToggle = millis();
    xSemaphoreGive(ledMutex);
  }
}

void turnLedOn() {
  if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
    digitalWrite(LED_PIN, LOW); // LOW turns on the LED on most ESP32 boards
    ledState = true;
    ledBlinkInterval = 0;
    xSemaphoreGive(ledMutex);
  }
}

void turnLedOff() {
  if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
    digitalWrite(LED_PIN, HIGH); // HIGH turns off the LED on most ESP32 boards
    ledState = false;
    ledBlinkInterval = 0;
    xSemaphoreGive(ledMutex);
  }
}

// Convert angle to PWM value
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Set servo angle with thread safety
void setServoAngle(int angle) {
  // Take mutex to ensure thread safety
  if (xSemaphoreTake(angleMutex, portMAX_DELAY)) {
    // Limit angle to valid range
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Convert angle to PWM value and write
    int pulse = angleToPulse(angle);
    ledcWrite(LEDC_CHANNEL, pulse);
    currentServoAngle = angle;
    
    // Indicate servo movement with LED (non-blocking)
    digitalWrite(LED_PIN, HIGH); // Briefly turn off
    vTaskDelay(pdMS_TO_TICKS(10)); // Non-blocking delay
    digitalWrite(LED_PIN, LOW);  // Turn back on
    
    Serial.print("Servo angle set to: ");
    Serial.print(angle);
    Serial.print(" (PWM value: ");
    Serial.print(pulse);
    Serial.println(")");
    
    xSemaphoreGive(angleMutex);
  }
}

// MPU6050 initialization function
bool initMPU6050() {
  // Initialize I2C with specified SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  
  Serial.println("MPU6050 Found!");
  
  // Configure accelerometer range (±2, 4, 8, or 16 G)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Configure gyroscope range (250, 500, 1000, or 2000 degrees/second)
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  // Configure filter bandwidth (5, 10, 21, 44, 94, 184, 260 Hz)
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  
  vTaskDelay(pdMS_TO_TICKS(100));
  return true;
}

// Read MPU6050 angle data
float readMPUAngle() {
  if (!mpuInitialized) return measuredAngle;
  
  float result = 0;
  
  if (xSemaphoreTake(angleMutex, portMAX_DELAY)) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calculate tilt angle based on accelerometer data
    float angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    
    // Apply low-pass filter for smoother readings
    measuredAngle = measuredAngle * (1.0 - FILTER_FACTOR) + angleX * FILTER_FACTOR;
    
    // Map to 0-180 degree range
    result = mapFloat(constrain(measuredAngle, -90.0f, 90.0f), -90.0f, 90.0f, 0.0f, 180.0f);
    
    xSemaphoreGive(angleMutex);
  }
  
  return result;
}

// Load WiFi configuration from EEPROM
void loadWiFiConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, wifiConfig);
  EEPROM.end();
  
  if (wifiConfig.configured) {
    Serial.println("WiFi configuration loaded from EEPROM");
    Serial.print("SSID: ");
    Serial.println(wifiConfig.ssid);
    wifiConfigured = true;
  } else {
    Serial.println("No WiFi configuration found in EEPROM");
    wifiConfigured = false;
  }
}

// Save WiFi configuration to EEPROM
void saveWiFiConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, wifiConfig);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("WiFi configuration saved to EEPROM");
}

// Start AP mode
void startAPMode() {
  WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP mode started. IP address: ");
  Serial.println(IP);
  
  // Set LED to slow blink for AP mode
  setLedMode(LED_AP_MODE);
}

// Try to connect to WiFi
bool connectToWiFi() {
  if (!wifiConfigured) return false;
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifiConfig.ssid);
  
  // Set LED to fast blink for WiFi connecting
  setLedMode(LED_WIFI_CONNECTING);
  
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
    
    // Set LED to solid ON for connected state
    turnLedOn();
    
    return true;
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi");
    return false;
  }
}

// Handle root page (Servo control page)
void handleRoot() {
  String html = readFile("/index.html");
  // Replace angle placeholder with current value
  html.replace("%ANGLE%", String(currentServoAngle));
  server.send(200, "text/html", html);
}

// WiFi configuration page
void handleWiFiConfig() {
  String html = readFile("/wifi_config.html");
  server.send(200, "text/html", html);
}

// Handle measured angle requests
void handleGetMeasuredAngle() {
  float angle = readMPUAngle();
  server.send(200, "application/json", "{\"angle\":" + String(angle, 2) + ", \"status\":" + String(mpuInitialized ? "true" : "false") + "}");
}

// Save WiFi configuration
void handleSaveWiFi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  
  if (xSemaphoreTake(wifiMutex, portMAX_DELAY)) {
    // Save configuration with proper null termination
    strncpy(wifiConfig.ssid, ssid.c_str(), MAX_SSID_LENGTH - 1);
    wifiConfig.ssid[MAX_SSID_LENGTH - 1] = '\0';
    
    strncpy(wifiConfig.password, password.c_str(), MAX_PASSWORD_LENGTH - 1);
    wifiConfig.password[MAX_PASSWORD_LENGTH - 1] = '\0';
    
    wifiConfig.configured = true;
    saveWiFiConfig();
    
    xSemaphoreGive(wifiMutex);
  }
  
  String html = readFile("/wifi_save.html");
  server.send(200, "text/html", html);
  
  // Schedule restart with LED indication
  setLedMode(100);
  restartPending = true;
  restartTime = millis();
}

// Set servo angle handler
void handleSetAngle() {
  if (server.hasArg("angle")) {
    int angle = server.arg("angle").toInt();
    setServoAngle(angle);
    server.send(200, "text/plain", "Angle set to " + String(angle));
  } else {
    server.send(400, "text/plain", "Missing angle parameter");
  }
}

// Web server task
void webServerTask(void *parameter) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent watchdog triggers
  }
}

// MPU sensor reading task
void mpuTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(20); // 100ms reading interval
  
  for (;;) {
    if (mpuInitialized) {
      readMPUAngle(); // Read and update angle measurement
    }
    vTaskDelay(xDelay);
  }
}

// LED control task
void ledControlTask(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(5000); // 50ms for LED control
  
  for (;;) {
    if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
      // Check if blinking is required
      if (ledBlinkInterval > 0) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastLedToggle >= ledBlinkInterval) {
          lastLedToggle = currentMillis;
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        }
      }
      
      // Check for restart pending
      if (restartPending && (millis() - restartTime >= 5000)) {
        ESP.restart();
      }
      
      xSemaphoreGive(ledMutex);
    }
    vTaskDelay(xDelay);
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Servo Control System Initializing with FreeRTOS...");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Start with LED off
  
  // Create mutexes
  angleMutex = xSemaphoreCreateMutex();
  wifiMutex = xSemaphoreCreateMutex();
  ledMutex = xSemaphoreCreateMutex();
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    // Error pattern - very fast blink
    ledBlinkInterval = 50;
    return;
  }
  Serial.println("SPIFFS mounted successfully");
  
  // Configure LEDC channel
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  
  // Attach LEDC channel to GPIO pin
  ledcAttachPin(SERVO_PIN, LEDC_CHANNEL);
  
  // Set servo to center position (90 degrees)
  setServoAngle(90);
  
  // Initialize MPU6050 sensor
  mpuInitialized = initMPU6050();
  if (mpuInitialized) {
    Serial.println("MPU6050 initialized successfully");
  } else {
    Serial.println("MPU6050 initialization failed - continuing without angle measurement");
  }
  
  // Load WiFi configuration
  loadWiFiConfig();
  
  // Try to connect to saved WiFi
  bool connected = false;
  if (wifiConfigured) {
    connected = connectToWiFi();
  }
  
  // If cannot connect, start AP mode
  if (!connected) {
    startAPMode();
    
    // Set web server routes for AP mode
    server.on("/", HTTP_GET, handleWiFiConfig);
    server.on("/saveWiFi", HTTP_POST, handleSaveWiFi);
  } else {
    // Set web server routes for STA mode
    server.on("/", HTTP_GET, handleRoot);
    server.on("/setAngle", HTTP_GET, handleSetAngle);
    server.on("/getMeasuredAngle", HTTP_GET, handleGetMeasuredAngle);
    
    // Add static file handler for all other files
    server.serveStatic("/", SPIFFS, "/");
  }
  
  // Start web server
  server.begin();
  Serial.println("Web server started");
  
  if (connected) {
    Serial.println("System ready in STA mode");
    Serial.print("Access control panel at: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("System ready in AP mode");
    Serial.print("Connect to WiFi network: ");
    Serial.println(apSSID);
    Serial.print("Password: ");
    Serial.println(apPassword);
    Serial.print("Then navigate to: http://");
    Serial.println(WiFi.softAPIP());
  }
  
  // Create FreeRTOS tasks
  xTaskCreate(
    webServerTask,    // Task function
    "WebServerTask",  // Name for debugging
    STACK_SIZE,       // Stack size (bytes)
    NULL,             // Task parameters
    WEBSERVER_TASK_PRIORITY,  // Priority (higher number = higher priority)
    NULL              // Task handle
  );
  
  xTaskCreate(
    mpuTask,
    "MPUTask",
    STACK_SIZE,
    NULL,
    MPU_TASK_PRIORITY,
    NULL
  );
  
  xTaskCreate(
    ledControlTask,
    "LEDTask",
    STACK_SIZE,
    NULL,
    LED_TASK_PRIORITY,
    NULL
  );
  
  // FreeRTOS is now running, main loop will not be used
}

void loop() {
  // Nothing to do here - FreeRTOS tasks are handling everything
  vTaskDelay(portMAX_DELAY); // Prevent watchdog issues
}