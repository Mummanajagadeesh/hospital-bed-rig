#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>

// Create two MPU6050 objects
MPU6050 mpu1;
MPU6050 mpu2;

// --- Sensor 1 Variables ---
// Raw sensor data for sensor 1
int16_t gx1, gy1, gz1, ax1, ay1, az1;
// Gyro offsets for sensor 1
float gyroX_offset1, gyroY_offset1, gyroZ_offset1;
// Kalman filter state variables for sensor 1
float kalmanPitch1 = 0, kalmanRoll1 = 0;
float biasPitch1 = 0, biasRoll1 = 0;
float P1[2][2] = { {1, 0}, {0, 1} };
// Yaw angle for sensor 1
float yaw1 = 0;

// --- Sensor 2 Variables ---
// Raw sensor data for sensor 2
int16_t gx2, gy2, gz2, ax2, ay2, az2;
// Gyro offsets for sensor 2
float gyroX_offset2, gyroY_offset2, gyroZ_offset2;
// Kalman filter state variables for sensor 2
float kalmanPitch2 = 0, kalmanRoll2 = 0;
float biasPitch2 = 0, biasRoll2 = 0;
float P2[2][2] = { {1, 0}, {0, 1} };
// Yaw angle for sensor 2
float yaw2 = 0;

// Common filter constants
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
const float alpha = 0.98;  

// ----- Calibration function -----
// Calibrates a given MPU6050 sensor and saves the gyro offsets to EEPROM.
// For sensor 1, offsets are stored at addresses 0, 4, and 8.
// For sensor 2, offsets are stored at addresses 12, 16, and 20.
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

// ----- Kalman Filter function -----
// Updated to accept a covariance matrix "P" for each sensor.
float kalmanFilter(float measured, float rate, float dt, float &state, float &bias, float P[2][2]) {
  // Predict step
  state += dt * (rate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update step
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

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Use default I2C pins (change if needed)
  EEPROM.begin(512);

  // Initialize both sensors
  mpu1.initialize();
  mpu2.initialize();

  // Test connection for sensor 1
  if (!mpu1.testConnection()) {
    Serial.println("MPU6050 sensor 1 connection failed!");
    while (1) { delay(10); }
  }
  // Test connection for sensor 2
  if (!mpu2.testConnection()) {
    Serial.println("MPU6050 sensor 2 connection failed!");
    while (1) { delay(10); }
  }

  Serial.println("Calibrating Gyroscopes...");
  delay(3000);
  
  // Calibrate sensor 1 (store offsets at EEPROM addresses 0, 4, 8)
  calibrateGyro(mpu1, gyroX_offset1, gyroY_offset1, gyroZ_offset1, 0);
  // Calibrate sensor 2 (store offsets at EEPROM addresses 12, 16, 20)
  calibrateGyro(mpu2, gyroX_offset2, gyroY_offset2, gyroZ_offset2, 12);
  
  Serial.println("Calibration complete");
}

void loop() {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.01;

  // ----- Sensor 1 Readings -----
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  
  // Remove gyro offsets for sensor 1
  float gyroX_deg1 = (gx1 - gyroX_offset1) / 131.0;
  float gyroY_deg1 = (gy1 - gyroY_offset1) / 131.0;
  float gyroZ_deg1 = (gz1 - gyroZ_offset1) / 131.0;
  
  // Compute pitch & roll from accelerometer for sensor 1
  float accPitch1 = atan2(-ax1, sqrt((float)ay1 * ay1 + (float)az1 * az1)) * 180.0 / PI;
  float accRoll1  = atan2(ay1, az1) * 180.0 / PI;
  
  // Kalman Filter for sensor 1
  float filteredPitch1 = kalmanFilter(accPitch1, gyroX_deg1, dt, kalmanPitch1, biasPitch1, P1);
  float filteredRoll1  = kalmanFilter(accRoll1,  gyroY_deg1, dt, kalmanRoll1, biasRoll1, P1);
  
  // Complementary filter for Yaw for sensor 1
  yaw1 = alpha * (yaw1 + gyroZ_deg1 * dt) + (1 - alpha) * atan2(ay1, ax1) * 180.0 / PI;
  if (yaw1 > 360) yaw1 -= 360;
  if (yaw1 < 0) yaw1 += 360;

  // Constrain angles for sensor 1
  filteredPitch1 = constrain(filteredPitch1, -90, 90);
  filteredRoll1  = constrain(filteredRoll1, -90, 90);

  // ----- Sensor 2 Readings -----
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  
  // Remove gyro offsets for sensor 2
  float gyroX_deg2 = (gx2 - gyroX_offset2) / 131.0;
  float gyroY_deg2 = (gy2 - gyroY_offset2) / 131.0;
  float gyroZ_deg2 = (gz2 - gyroZ_offset2) / 131.0;
  
  // Compute pitch & roll from accelerometer for sensor 2
  float accPitch2 = atan2(-ax2, sqrt((float)ay2 * ay2 + (float)az2 * az2)) * 180.0 / PI;
  float accRoll2  = atan2(ay2, az2) * 180.0 / PI;
  
  // Kalman Filter for sensor 2
  float filteredPitch2 = kalmanFilter(accPitch2, gyroX_deg2, dt, kalmanPitch2, biasPitch2, P2);
  float filteredRoll2  = kalmanFilter(accRoll2,  gyroY_deg2, dt, kalmanRoll2, biasRoll2, P2);
  
  // Complementary filter for Yaw for sensor 2
  yaw2 = alpha * (yaw2 + gyroZ_deg2 * dt) + (1 - alpha) * atan2(ay2, ax2) * 180.0 / PI;
  if (yaw2 > 360) yaw2 -= 360;
  if (yaw2 < 0) yaw2 += 360;
  
  // Constrain angles for sensor 2
  filteredPitch2 = constrain(filteredPitch2, -90, 90);
  filteredRoll2  = constrain(filteredRoll2, -90, 90);

  // ----- Output Data -----
  Serial.print("Sensor1 -> Pitch: ");
  Serial.print(filteredPitch1);
  Serial.print(" | Roll: ");
  Serial.print(filteredRoll1);
  Serial.print(" | Yaw: ");
  Serial.println(yaw1);

  Serial.print("Sensor2 -> Pitch: ");
  Serial.print(filteredPitch2);
  Serial.print(" | Roll: ");
  Serial.print(filteredRoll2);
  Serial.print(" | Yaw: ");
  Serial.println(yaw2);

  Serial.println("");
  delay(20);
}