#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPIFFS.h>
// 添加 MPU6050 相关库
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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
unsigned long restartTime = 0;
bool restartPending = false;

// 添加 MPU6050 相关全局变量
Adafruit_MPU6050 mpu;
unsigned long lastMpuReadTime = 0;
const int MPU_READ_INTERVAL = 100;  // 读取间隔 (毫秒)
float measuredAngle = 90;           // 初始值与舵机默认角度相同
const float FILTER_FACTOR = 0.1;    // 平滑角度数据的滤波系数 (0-1)
bool mpuInitialized = false;        // MPU6050 初始化状态

// Structure to store WiFi configuration
struct {
  char ssid[MAX_SSID_LENGTH] = {0};
  char password[MAX_PASSWORD_LENGTH] = {0};
  bool configured = false;
} wifiConfig;

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
  ledBlinkInterval = blinkInterval;
  lastLedToggle = millis();
}

void turnLedOn() {
  digitalWrite(LED_PIN, LOW); // LOW turns on the LED on most ESP32 boards
  ledState = true;
}

void turnLedOff() {
  digitalWrite(LED_PIN, HIGH); // HIGH turns off the LED on most ESP32 boards
  ledState = false;
}

void blinkLed() {
  if (ledBlinkInterval == 0) {
    // Solid on
    turnLedOn();
    return;
  }
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastLedToggle >= ledBlinkInterval) {
    lastLedToggle = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH);
  }
}

void pulseServoLed() {
  // Quick pulse to indicate servo movement
  turnLedOff();
  delay(LED_SERVO_ACTIVE);
  turnLedOn();
}

// Convert angle to PWM value
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Set servo angle
void setServoAngle(int angle) {
  // Limit angle to valid range
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  // Convert angle to PWM value and write
  int pulse = angleToPulse(angle);
  ledcWrite(LEDC_CHANNEL, pulse);
  currentServoAngle = angle;
  
  // Flash LED to indicate servo movement
  pulseServoLed();
  
  Serial.print("Servo angle set to: ");
  Serial.print(angle);
  Serial.print(" (PWM value: ");
  Serial.print(pulse);
  Serial.println(")");
}

// MPU6050 初始化函数
bool initMPU6050() {
  // 使用指定的 SDA 和 SCL 引脚初始化 I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  
  Serial.println("MPU6050 Found!");
  
  // 设置加速度计范围 (±2, 4, 8, or 16 G)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // 设置陀螺仪范围 (250, 500, 1000, or 2000 度/秒)
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // 设置滤波带宽 (5, 10, 21, 44, 94, 184, 260 Hz)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);
  return true;
}

// 读取 MPU6050 角度数据
float readMPUAngle() {
  if (!mpuInitialized) return measuredAngle;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // 计算基于加速度计的倾角（假设安装方向与舵面方向匹配）
  // 这里假设 MPU6050 的 X 轴与舵面旋转轴垂直
  float angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  
  // 应用低通滤波平滑数据
  measuredAngle = measuredAngle * (1.0 - FILTER_FACTOR) + angleX * FILTER_FACTOR;
  
  // 映射到 0-180 度范围
  float mappedAngle = map(constrain(measuredAngle, -90, 90), -90, 90, 0, 180);
  
  return mappedAngle;
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
    delay(500);
    Serial.print(".");
    blinkLed(); // Update LED while waiting
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
    
    // Set LED to solid ON for connected state
    turnLedOn();
    ledBlinkInterval = 0; // Solid on
    
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

// 处理获取实际角度的请求
void handleGetMeasuredAngle() {
  float angle = readMPUAngle();
  server.send(200, "application/json", "{\"angle\":" + String(angle, 1) + ", \"status\":" + String(mpuInitialized ? "true" : "false") + "}");
}

// Save WiFi configuration and try to connect
void handleSaveWiFi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  
  // Save configuration (添加安全保障：确保字符串以null结尾)
  strncpy(wifiConfig.ssid, ssid.c_str(), MAX_SSID_LENGTH - 1);
  wifiConfig.ssid[MAX_SSID_LENGTH - 1] = '\0';
  
  strncpy(wifiConfig.password, password.c_str(), MAX_PASSWORD_LENGTH - 1);
  wifiConfig.password[MAX_PASSWORD_LENGTH - 1] = '\0';
  
  wifiConfig.configured = true;
  saveWiFiConfig();
  
  String html = readFile("/wifi_save.html");
  server.send(200, "text/html", html);
  
  // 设置重启标志而不是立即重启
  restartTime = millis();
  restartPending = true;
  setLedMode(100);
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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Servo Control System Initializing...");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  turnLedOff(); // Start with LED off
  
  // Initial startup LED pattern - fast blink
  setLedMode(100);
  for (int i = 0; i < 10; i++) {
    turnLedOn();
    delay(100);
    turnLedOff();
    delay(100);
  }
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    // Error pattern - very fast blink
    setLedMode(50);
    return;
  }
  Serial.println("SPIFFS mounted successfully");
  
  // Configure LEDC channel
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  
  // Attach LEDC channel to GPIO pin
  ledcAttachPin(SERVO_PIN, LEDC_CHANNEL);
  
  // Set servo to center position (90 degrees)
  setServoAngle(90);
  
  // 初始化 MPU6050 传感器
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
}

void loop() {
  server.handleClient();
  // blinkLed(); // Update LED state
    // 非阻塞重启检查
  if (restartPending && millis() - restartTime >= 5000) {
    ESP.restart();
  }
  // 周期性读取 MPU6050 数据
  unsigned long currentMillis = millis();
  if (mpuInitialized && (currentMillis - lastMpuReadTime >= MPU_READ_INTERVAL)) {
    lastMpuReadTime = currentMillis;
    readMPUAngle();
  }
}