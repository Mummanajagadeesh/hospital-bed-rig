#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>


// ==================== WiFi Settings ====================
const char* ssid = "Raghul";
const char* password = "raghul*2803";

// ==================== HTTP & UDP Settings ====================
ESP8266WebServer httpServer(80);
WiFiUDP udp;
unsigned int localUdpPort = 8888;  // (optional, if you want to receive UDP replies)
const char* remoteIP = "192.168.1.100"; // Remote IP for UDP packets
unsigned int remoteUdpPort1 = 8888; // UDP port for sensor 1 data
unsigned int remoteUdpPort2 = 8889; // UDP port for sensor 2 data

// ==================== Sensor Declarations ====================
// Create two MPU6050 objects
MPU6050 mpu1; // Sensor 1 (should be at address 0x68)
MPU6050 mpu2; // Sensor 2 (should be at address 0x69)

// ---------- Sensor 1 Variables ----------
int16_t gx1, gy1, gz1, ax1, ay1, az1;
float gyroX_offset1, gyroY_offset1, gyroZ_offset1;
float kalmanPitch1 = 0, kalmanRoll1 = 0;
float biasPitch1 = 0, biasRoll1 = 0;
float P1[2][2] = { {1, 0}, {0, 1} };
float yaw1 = 0;

// ---------- Sensor 2 Variables ----------
int16_t gx2, gy2, gz2, ax2, ay2, az2;
float gyroX_offset2, gyroY_offset2, gyroZ_offset2;
float kalmanPitch2 = 0, kalmanRoll2 = 0;
float biasPitch2 = 0, biasRoll2 = 0;
float P2[2][2] = { {1, 0}, {0, 1} };
float yaw2 = 0;

// Common filter constants
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
const float alpha = 0.98;  

// Variables to store filtered values (for HTTP and UDP)
float filteredPitch1, filteredRoll1;
float filteredPitch2, filteredRoll2;

// ==================== Utility Functions ====================

// Calibrates a given sensor and stores gyro offsets in EEPROM.
// For sensor 1, offsets are stored at EEPROM addresses 0,4,8
// For sensor 2, offsets are stored at EEPROM addresses 12,16,20
void calibrateGyro(MPU6050 &sensor, float &offsetX, float &offsetY, float &offsetZ, int baseEEPROMAddr) {
  const int samples = 2000;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    sensor.getRotation(&gx, &gy, &gz);
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(2);
  }
  offsetX = gx_sum / (float)samples;
  offsetY = gy_sum / (float)samples;
  offsetZ = gz_sum / (float)samples;
  EEPROM.put(baseEEPROMAddr, offsetX);
  EEPROM.put(baseEEPROMAddr + 4, offsetY);
  EEPROM.put(baseEEPROMAddr + 8, offsetZ);
  EEPROM.commit();
}

// Kalman Filter function
float kalmanFilter(float measured, float rate, float dt, float &state, float &bias, float P[2][2]) {
  // Predict
  state += dt * (rate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // Update
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

// ==================== HTTP Server Handlers ====================

// HTML page to display live sensor data
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <meta charset="utf-8">
    <title>Dual MPU6050 Live Data</title>
  </head>
  <body>
    <h1>Dual MPU6050 Live Data</h1>
    <h2>Sensor 1</h2>
    <p>Pitch: <span id="pitch1">0</span></p>
    <p>Roll: <span id="roll1">0</span></p>
    <p>Yaw: <span id="yaw1">0</span></p>
    <h2>Sensor 2</h2>
    <p>Pitch: <span id="pitch2">0</span></p>
    <p>Roll: <span id="roll2">0</span></p>
    <p>Yaw: <span id="yaw2">0</span></p>
    <script>
      setInterval(function() {
        fetch('/data')
          .then(response => response.json())
          .then(data => {
            document.getElementById('pitch1').innerText = data.sensor1.pitch;
            document.getElementById('roll1').innerText  = data.sensor1.roll;
            document.getElementById('yaw1').innerText   = data.sensor1.yaw;
            document.getElementById('pitch2').innerText = data.sensor2.pitch;
            document.getElementById('roll2').innerText  = data.sensor2.roll;
            document.getElementById('yaw2').innerText   = data.sensor2.yaw;
          });
      }, 1000);
    </script>
  </body>
</html>
)rawliteral";

// Serve the main HTML page
void handleRoot() {
  httpServer.send_P(200, "text/html", webpage);
}

// Serve sensor data in JSON format
void handleData() {
  String json = "{";
  json += "\"sensor1\":{\"pitch\":" + String(filteredPitch1, 2);
  json += ",\"roll\":"  + String(filteredRoll1, 2);
  json += ",\"yaw\":"   + String(yaw1, 2) + "},";
  json += "\"sensor2\":{\"pitch\":" + String(filteredPitch2, 2);
  json += ",\"roll\":"  + String(filteredRoll2, 2);
  json += ",\"yaw\":"   + String(yaw2, 2) + "}}";
  httpServer.send(200, "application/json", json);
}

// ==================== Global Timing ====================
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Use default I2C pins (adjust if needed)
  EEPROM.begin(512);

  // Initialize both sensors
  mpu1.initialize();
  mpu2.initialize();

  // Test sensor connections
  if (!mpu1.testConnection()) {
    Serial.println("MPU6050 sensor 1 connection failed!");
    while (1) { delay(10); }
  }
  if (!mpu2.testConnection()) {
    Serial.println("MPU6050 sensor 2 connection failed!");
    while (1) { delay(10); }
  }

  // Calibrate gyroscopes
  Serial.println("Calibrating Gyroscopes...");
  delay(3000);
  calibrateGyro(mpu1, gyroX_offset1, gyroY_offset1, gyroZ_offset1, 0);
  calibrateGyro(mpu2, gyroX_offset2, gyroY_offset2, gyroZ_offset2, 12);
  Serial.println("Calibration complete");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  // Start the HTTP server
  httpServer.on("/", handleRoot);
  httpServer.on("/data", handleData);
  httpServer.begin();
  Serial.println("HTTP server started");

  // Start UDP (optional: for receiving replies, etc.)
  udp.begin(localUdpPort);
}

void loop() {
  // Calculate dt (delta time)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.01;

  // --------- Sensor 1 Update ---------
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  float gyroX_deg1 = (gx1 - gyroX_offset1) / 131.0;
  float gyroY_deg1 = (gy1 - gyroY_offset1) / 131.0;
  float gyroZ_deg1 = (gz1 - gyroZ_offset1) / 131.0;
  float accPitch1 = atan2(-ax1, sqrt((float)ay1 * ay1 + (float)az1 * az1)) * 180.0 / PI;
  float accRoll1  = atan2(ay1, az1) * 180.0 / PI;
  filteredPitch1 = kalmanFilter(accPitch1, gyroX_deg1, dt, kalmanPitch1, biasPitch1, P1);
  filteredRoll1  = kalmanFilter(accRoll1,  gyroY_deg1, dt, kalmanRoll1, biasRoll1, P1);
  yaw1 = alpha * (yaw1 + gyroZ_deg1 * dt) + (1 - alpha) * atan2(ay1, ax1) * 180.0 / PI;
  if (yaw1 > 360) yaw1 -= 360;
  if (yaw1 < 0) yaw1 += 360;
  filteredPitch1 = constrain(filteredPitch1, -90, 90);
  filteredRoll1  = constrain(filteredRoll1, -90, 90);

  // --------- Sensor 2 Update ---------
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  float gyroX_deg2 = (gx2 - gyroX_offset2) / 131.0;
  float gyroY_deg2 = (gy2 - gyroY_offset2) / 131.0;
  float gyroZ_deg2 = (gz2 - gyroZ_offset2) / 131.0;
  float accPitch2 = atan2(-ax2, sqrt((float)ay2 * ay2 + (float)az2 * az2)) * 180.0 / PI;
  float accRoll2  = atan2(ay2, az2) * 180.0 / PI;
  filteredPitch2 = kalmanFilter(accPitch2, gyroX_deg2, dt, kalmanPitch2, biasPitch2, P2);
  filteredRoll2  = kalmanFilter(accRoll2,  gyroY_deg2, dt, kalmanRoll2, biasRoll2, P2);
  yaw2 = alpha * (yaw2 + gyroZ_deg2 * dt) + (1 - alpha) * atan2(ay2, ax2) * 180.0 / PI;
  if (yaw2 > 360) yaw2 -= 360;
  if (yaw2 < 0) yaw2 += 360;
  filteredPitch2 = constrain(filteredPitch2, -90, 90);
  filteredRoll2  = constrain(filteredRoll2, -90, 90);

  // --------- UDP Transmission ---------
  // Build JSON for Sensor 1
  String json1 = "{";
  json1 += "\"pitch\":" + String(filteredPitch1, 2);
  json1 += ",\"roll\":"  + String(filteredRoll1, 2);
  json1 += ",\"yaw\":"   + String(yaw1, 2);
  json1 += "}";
  // Build JSON for Sensor 2
  String json2 = "{";
  json2 += "\"pitch\":" + String(filteredPitch2, 2);
  json2 += ",\"roll\":"  + String(filteredRoll2, 2);
  json2 += ",\"yaw\":"   + String(yaw2, 2);
  json2 += "}";

  // Send UDP packet for Sensor 1
  udp.beginPacket(remoteIP, remoteUdpPort1);
  udp.write(json1.c_str());
  udp.endPacket();

  // Send UDP packet for Sensor 2
  udp.beginPacket(remoteIP, remoteUdpPort2);
  udp.write(json2.c_str());
  udp.endPacket();

  // --------- Handle HTTP Requests ---------
  httpServer.handleClient();

  delay(20); // Adjust loop timing as needed
}