#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <math.h>

#define TCAADDR 0x70
#define RAD_TO_DEG 57.2958

// -------- MULTIPLEXER SELECT --------
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// -------- IMU OBJECTS --------
DFRobot_BMI160 imu1;
DFRobot_BMI160 imu2;

// -------- VARIABLES --------
float ax1, ay1, az1, gx1, gy1, gz1;
float ax2, ay2, az2, gx2, gy2, gz2;

float gx_bias1 = 0, gy_bias1 = 0, gz_bias1 = 0;
float gx_bias2 = 0, gy_bias2 = 0, gz_bias2 = 0;

float yaw = 0;
unsigned long lastTime;
float dt;

// Position tracking (optional)
float vx = 0, vy = 0, vz = 0;
float px = 0, py = 0, pz = 0;

// -------- SETUP --------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // --- INIT IMU 1 ---
  tcaSelect(0);
  imu1.softReset();
  imu1.I2cInit(0x69);

  // --- INIT IMU 2 ---
  tcaSelect(1);
  imu2.softReset();
  imu2.I2cInit(0x69);

  Serial.println("Calibrating... KEEP IMU STILL");

  // --- CALIBRATE IMU 1 ---
  for (int i = 0; i < 500; i++) {
    int16_t d[6];
    tcaSelect(0);
    imu1.getAccelGyroData(d);

    gx_bias1 += d[0];
    gy_bias1 += d[1];
    gz_bias1 += d[2];
    delay(5);
  }
  gx_bias1 /= 500;
  gy_bias1 /= 500;
  gz_bias1 /= 500;

  // --- CALIBRATE IMU 2 ---
  for (int i = 0; i < 500; i++) {
    int16_t d[6];
    tcaSelect(1);
    imu2.getAccelGyroData(d);

    gx_bias2 += d[0];
    gy_bias2 += d[1];
    gz_bias2 += d[2];
    delay(5);
  }
  gx_bias2 /= 500;
  gy_bias2 /= 500;
  gz_bias2 /= 500;

  Serial.println("Calibration done");

  lastTime = millis();
}

// -------- LOOP --------
void loop() {

  // --- TIME STEP ---
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  int16_t d1[6], d2[6];

  // --- READ IMU 1 ---
  tcaSelect(0);
  imu1.getAccelGyroData(d1);

  // --- READ IMU 2 ---
  tcaSelect(1);
  imu2.getAccelGyroData(d2);

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

  // -------- FUSION --------

  // Average gyro
  float gz_fused = (gz1 + gz2) / 2.0;

  // Average accel
  float ax = (ax1 + ax2) / 2.0;
  float ay = (ay1 + ay2) / 2.0;
  float az = (az1 + az2) / 2.0;

  // --- ORIENTATION ---
  float pitch = atan2(ay, az);
  float roll  = atan2(-ax, sqrt(ay * ay + az * az));

  // --- YAW ---
  yaw += gz_fused * dt;

  // -------- GRAVITY REMOVAL --------
  float gx_comp = -9.81 * sin(pitch);
  float gy_comp =  9.81 * sin(roll) * cos(pitch);
  float gz_comp =  9.81 * cos(roll) * cos(pitch);

  float ax_lin = ax - gx_comp;
  float ay_lin = ay - gy_comp;
  float az_lin = az - gz_comp;

  // -------- NOISE FILTER --------
  float threshold = 0.1;
  if (abs(ax_lin) < threshold) ax_lin = 0;
  if (abs(ay_lin) < threshold) ay_lin = 0;
  if (abs(az_lin) < threshold) az_lin = 0;

  // -------- INTEGRATION --------
  vx += ax_lin * dt;
  vy += ay_lin * dt;
  vz += az_lin * dt;

  px += vx * dt;
  py += vy * dt;
  pz += vz * dt;

  // -------- OUTPUT --------
  Serial.print("Yaw: ");
  Serial.println(yaw * RAD_TO_DEG);
  // Serial.print(" | Pos: ");
  // Serial.print(px); Serial.print(",");
  // Serial.print(py); Serial.print(",");
  // Serial.println(pz);
  // Serial.print("ax "); Serial.print(ax_lin); Serial.print(",");
  // Serial.print("ay "); Serial.print(ay_lin); Serial.print(",");
  // Serial.print("az "); Serial.println(az_lin);
  

  delay(10);
}
