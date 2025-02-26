#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>

MPU6050 mpu;

// Raw sensor data
int16_t gx, gy, gz;
int16_t ax, ay, az;

// Gyro offsets
float gyroX_offset, gyroY_offset, gyroZ_offset;

// Kalman Filter Variables
float kalmanPitch = 0, kalmanRoll = 0;
float biasPitch = 0, biasRoll = 0;
float P[2][2] = {{1, 0}, {0, 1}}; // Kalman covariance matrix
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

// Complementary Filter Weight
const float alpha = 0.98;  

// Yaw angle (complementary filter)
float yaw = 0;

void calibrateGyro() {
  const int samples = 2000;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < samples; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(2);
  }
  
  gyroX_offset = gx_sum / samples;
  gyroY_offset = gy_sum / samples;
  gyroZ_offset = gz_sum / samples;

  EEPROM.put(0, gyroX_offset);
  EEPROM.put(4, gyroY_offset);
  EEPROM.put(8, gyroZ_offset);
  EEPROM.commit();
}

float kalmanFilter(float measured, float rate, float dt, float &state, float &bias) {
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
  Wire.begin();
  EEPROM.begin(512);
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  Serial.println("Calibrating Gyroscope...");
  delay(3000);
  calibrateGyro();
  Serial.println("Calibration complete");
}

void loop() {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.01;

  // Read IMU data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Remove gyro offsets
  float gyroX_deg = (gx - gyroX_offset) / 131.0;
  float gyroY_deg = (gy - gyroY_offset) / 131.0;
  float gyroZ_deg = (gz - gyroZ_offset) / 131.0;
  
  // Compute pitch & roll from accelerometer
  float accPitch = atan2(-ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;
  float accRoll = atan2(ay, az) * 180.0 / PI;

  // Kalman Filter for Pitch & Roll
  float filteredPitch = kalmanFilter(accPitch, gyroX_deg, dt, kalmanPitch, biasPitch);
  float filteredRoll = kalmanFilter(accRoll, gyroY_deg, dt, kalmanRoll, biasRoll);
  
  // Complementary filter for Yaw (reduces drift)
  yaw = alpha * (yaw + gyroZ_deg * dt) + (1 - alpha) * atan2(ay, ax) * 180.0 / PI;
  if (yaw > 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  // Constrain angles
  filteredPitch = constrain(filteredPitch, -90, 90);
  filteredRoll = constrain(filteredRoll, -90, 90);

  // Output data
  Serial.print("Pitch: "); Serial.print(filteredPitch);
  Serial.print(" | Roll: "); Serial.print(filteredRoll);
  Serial.print(" | Yaw: "); Serial.println(yaw);

  delay(20);
}