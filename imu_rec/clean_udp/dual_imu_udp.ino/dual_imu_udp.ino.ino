#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// ==================== WiFi Settings ====================
const char* ssid = "Redmi Note 12 Pro 5G";
const char* password = "football";

// ==================== UDP Settings ====================
WiFiUDP udp;
const char* remoteIP = "192.168.17.173"; // Receiver's IP Address
unsigned int remoteUdpPort1 = 8888; // IMU1 Data
unsigned int remoteUdpPort2 = 8889; // IMU2 Data

// ==================== MPU6050 Sensors ====================
MPU6050 mpu1(0x68); // IMU1 (Default address)
MPU6050 mpu2(0x69); // IMU2 (Alternative address)

// ---------- Sensor 1 Variables ----------
int16_t ax1, ay1, az1, gx1, gy1, gz1;
float kalmanPitch1 = 0, kalmanRoll1 = 0;
float biasPitch1 = 0, biasRoll1 = 0;
float P1[2][2] = {{1, 0}, {0, 1}};
float filteredPitch1, filteredRoll1;

// ---------- Sensor 2 Variables ----------
int16_t ax2, ay2, az2, gx2, gy2, gz2;
float kalmanPitch2 = 0, kalmanRoll2 = 0;
float biasPitch2 = 0, biasRoll2 = 0;
float P2[2][2] = {{1, 0}, {0, 1}};
float filteredPitch2, filteredRoll2;

// Common Kalman Filter Constants
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
const float alpha = 0.98;

// ==================== Kalman Filter ====================
float kalmanFilter(float measured, float rate, float dt, float &state, float &bias, float P[2][2]) {
  state += dt * (rate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  float S = P[0][0] + R_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  float y = measured - state;
  state += K[0] * y;
  bias += K[1] * y;
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
  return state;
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  EEPROM.begin(512);

  // WiFi Connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected. IP: " + WiFi.localIP().toString());

  // Initialize MPU6050 Sensors
  mpu1.initialize();
  mpu2.initialize();
  if (!mpu1.testConnection() || !mpu2.testConnection()) {
    Serial.println("MPU6050 Connection Failed!");
    while (1) delay(100);
  }

  Serial.println("IMUs Initialized.");
}

// ==================== Main Loop ====================
void loop() {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.01;

  // --------- Sensor 1 Data ---------
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  float gyroX_deg1 = gx1 / 131.0;
  float gyroY_deg1 = gy1 / 131.0;
  float accPitch1 = atan2(-ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
  float accRoll1  = atan2(ay1, az1) * 180.0 / PI;
  filteredPitch1 = kalmanFilter(accPitch1, gyroX_deg1, dt, kalmanPitch1, biasPitch1, P1);
  filteredRoll1  = kalmanFilter(accRoll1,  gyroY_deg1, dt, kalmanRoll1, biasRoll1, P1);

  // --------- Sensor 2 Data ---------
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  float gyroX_deg2 = gx2 / 131.0;
  float gyroY_deg2 = gy2 / 131.0;
  float accPitch2 = atan2(-ax2, sqrt(ay2 * ay2 + az2 * az2)) * 180.0 / PI;
  float accRoll2  = atan2(ay2, az2) * 180.0 / PI;
  filteredPitch2 = kalmanFilter(accPitch2, gyroX_deg2, dt, kalmanPitch2, biasPitch2, P2);
  filteredRoll2  = kalmanFilter(accRoll2,  gyroY_deg2, dt, kalmanRoll2, biasRoll2, P2);

  // --------- UDP Transmission ---------
  // Sensor 1 Data
  StaticJsonDocument<128> json1;
  json1["pitch"] = filteredPitch1;
  json1["roll"] = filteredRoll1;
  char buffer1[128];
  serializeJson(json1, buffer1);
  udp.beginPacket(remoteIP, remoteUdpPort1);
  udp.write(buffer1);
  udp.endPacket();

  // Sensor 2 Data
  StaticJsonDocument<128> json2;
  json2["pitch"] = filteredPitch2;
  json2["roll"] = filteredRoll2;
  char buffer2[128];
  serializeJson(json2, buffer2);
  udp.beginPacket(remoteIP, remoteUdpPort2);
  udp.write(buffer2);
  udp.endPacket();

  // Debug Output
  Serial.print("IMU1 - Pitch: "); Serial.print(filteredPitch1);
  Serial.print(", Roll: "); Serial.println(filteredRoll1);
  Serial.print("IMU2 - Pitch: "); Serial.print(filteredPitch2);
  Serial.print(", Roll: "); Serial.println(filteredRoll2);

  delay(20);
}
