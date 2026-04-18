#include <DFRobot_BMI160.h>
#include <Wire.h>
#include <Math.h>

// IMUs
// DFRobot_BMI160 bmi160;
DFRobot_BMI160 imu1;
DFRobot_BMI160 imu2;

const int8_t i2c_addr = 0x68;
const int8_t TCAADDR = 0x70;

// Variables for orientation calculations
float ax1, ay1, az1, gx1, gy1, gz1;
float ax2, ay2, az2, gx2, gy2, gz2;
float pitch, roll, yaw;
unsigned long lastTime;
float dt = 0.01;  // Time interval in seconds (10 ms)
// add biessies
float gx_bias1 = 0, gy_bias1 = 0, gz_bias1 = 0;
float gx_bias2 = 0, gy_bias2 = 0, gz_bias2 = 0;
float vx = 0, vy = 0, vz = 0;
float px = 0, py = 0, pz = 0;

// Multiplexer code
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();

  // --- INIT IMU 1 ---
  tcaSelect(0);
  imu1.softReset();
  imu1.I2cInit(0x69);

  // --- INIT IMU 2 ---
  tcaSelect(1);
  imu2.softReset();
  imu2.I2cInit(0x69);


  // --- CALIBRATE IMU 1 ---
  for (int i = 0; i < 1000; i++) {
    int16_t d[6];
    tcaSelect(0);
    imu1.getAccelGyroData(d);

    gx_bias1 += d[0];
    gy_bias1 += d[1];
    gz_bias1 += d[2];
    delay(5);
  }
  gx_bias1 /= 1000;
  gy_bias1 /= 1000;
  gz_bias1 /= 1000;
  Serial.println("Test");
  // --- CALIBRATE IMU 2 ---
  for (int i = 0; i < 1000; i++) {
    int16_t d[6];
    tcaSelect(1);
    imu2.getAccelGyroData(d);

    gx_bias2 += d[0];
    gy_bias2 += d[1];
    gz_bias2 += d[2];
    delay(5);
  }
  gx_bias2 /= 1000;
  gy_bias2 /= 1000;
  gz_bias2 /= 1000;

  Serial.println("Calibration done");

  lastTime = millis();

  // Serial.println("Bias calculated");
  // Serial.print("gx_bias: ");
  // Serial.print(gx_bias);
  // Serial.print(" gy_bias: ");
  // Serial.print(gy_bias);
  // Serial.print(" gz_bias: ");
  // Serial.println(gz_bias);

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // Update dt
  lastTime = currentTime;

  int16_t d1[6], d2[6];

  // --- READ IMU 1 ---
  tcaSelect(0);
  int rslt1 = imu1.getAccelGyroData(d1);

  // --- READ IMU 2 ---
  tcaSelect(1);
  int rslt2 = imu2.getAccelGyroData(d2);

  if (rslt1 == 0 && rslt2 == 0) {
    // --- CONVERT ACCEL (m/s²) ---
    ax1 = (d1[3] / 16384.0) * 9.81;
    ay1 = (d1[4] / 16384.0) * 9.81;
    az1 = (d1[5] / 16384.0) * 9.81;

    ax2 = (d2[3] / 16384.0) * 9.81;
    ay2 = (d2[4] / 16384.0) * 9.81;
    az2 = (d2[5] / 16384.0) * 9.81;

    // --- CONVERT GYRO (rad/s) ---
    gx1 = (d1[0] - gx_bias1) * PI / 180.0;
    gy1 = (d1[1] - gy_bias1) * PI / 180.0;
    gz1 = (d1[2] - gz_bias1) * PI / 180.0;

    gx2 = (d2[0] - gx_bias2) * PI / 180.0;
    gy2 = (d2[1] - gy_bias2) * PI / 180.0;
    gz2 = (d2[2] - gz_bias2) * PI / 180.0;

    // FUSION of IMUs

    // Average gyro
    float gz_fused = (gz1 + gz2) / 2.0;

    // Average accel
    float ax = (ax1 + ax2) / 2.0;
    float ay = (ay1 + ay2) / 2.0;
    float az = (az1 + az2) / 2.0;

    // Calculate pitch and roll from accelerometer data
    // atan2() is a trigonometric function that calculates the angle between ay and az
    pitch = atan2(ay, az);                       // in radians (easier to calculate later with sin())
    roll = atan2(-ax, sqrt(ay * ay + az * az));  // still in radians

    // Integrate gyroscope data to get yaw
    yaw += gz_fused * dt;
    // --- REMOVE GRAVITY (using pitch & roll) ---
    float gx_comp = -9.81 * sin(pitch);
    float gy_comp = 9.81 * sin(roll) * cos(pitch);
    float gz_comp = 9.81 * cos(roll) * cos(pitch);

    float ax_lin = ax - gx_comp;
    float ay_lin = ay - gy_comp;
    float az_lin = az - gz_comp;

    // --- SIMPLE NOISE THRESHOLD (helps drift a bit) ---
    float threshold = 0.1;

    if (abs(ax_lin) < threshold) ax_lin = 0;
    if (abs(ay_lin) < threshold) ay_lin = 0;
    if (abs(az_lin) < threshold) az_lin = 0;

    // --- INTEGRATE TO VELOCITY ---
    vx += ax_lin * dt;
    vy += ay_lin * dt;
    vz += az_lin * dt;

    // --- ZERO VELOCITY UPDATE (if nearly stationary) ---
    if (ax_lin == 0 && ay_lin == 0 && az_lin == 0) {
      vx *= 0.9;
      vy *= 0.9;
      vz *= 0.9;
    }

    // --- INTEGRATE TO POSITION ---
    px += vx * dt;
    py += vy * dt;
    pz += vz * dt;

    // --- OUTPUT POSITION ---
    // Serial.print("px "); Serial.print(px); Serial.print(",");
    // Serial.print("py "); Serial.print(py); Serial.print(",");
    // Serial.print("pz "); Serial.println(pz);

    // Print pitch, roll, and yaw
    // Serial.print(pitch); Serial.print(",");
    // Serial.print(roll); Serial.print(",");
    // Serial.println(yaw);
    Serial.print("ax ");
    Serial.print(ax_lin);
    Serial.print(",");
    Serial.print("ay ");
    Serial.print(ay_lin);
    Serial.print(",");
    Serial.print("az ");
    Serial.println(az_lin);
    // Serial.print("gx "); Serial.print(gx); Serial.print(",");
    // Serial.print("gy "); Serial.print(gy); Serial.print(",");
    // Serial.print("gz "); Serial.println(gz);
    // Serial.print("vz "); Serial.print(vz); Serial.print(",");
    // Serial.print("pz "); Serial.println(pz);
    // float threshold = 0.1;
    // if (ax > threshold) {
    //   Serial.println("Forward");
    // }
    // else if (ax < -threshold) {
    //   Serial.println("Backward");
    // }
    // else {
    //   Serial.println("Not moving");
    // }
    //   } else {
    //     Serial.println("error");
    //   }

    // delay(10);
    delay(100);
  }
}
