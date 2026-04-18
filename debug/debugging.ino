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
  for (int i = 0; i < 5; i++) {
    int16_t d[6];
    tcaSelect(0);
    imu1.getAccelGyroData(d);

    gx_bias1 += d[0];
    gy_bias1 += d[1];
    gz_bias1 += d[2];
    delay(5);
  }
  gx_bias1 /= 5;
  gy_bias1 /= 5;
  gz_bias1 /= 5;

  // --- CALIBRATE IMU 2 ---
  for (int i = 0; i < 5; i++) {
    int16_t d[6];
    tcaSelect(1);
    imu2.getAccelGyroData(d);

    gx_bias2 += d[0];
    gy_bias2 += d[1];
    gz_bias2 += d[2];
    delay(5);
  }
  gx_bias2 /= 5;
  gy_bias2 /= 5;
  gz_bias2 /= 5;

  Serial.println("Calibration done");

  lastTime = millis();
}

// -------- LOOP --------
void loop() {
  for (int i = 0; i < 2; i++) {
    tcaSelect(i);
    Serial.print("Channel "); Serial.println(i);

    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("Found at 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  int16_t d1[6], d2[6];

  tcaSelect(0);
  imu1.getAccelGyroData(d1);

  tcaSelect(1);
  imu2.getAccelGyroData(d2);

  Serial.print("IMU1 ax: ");
  Serial.print(d1[3]);
  Serial.print(" | IMU2 ax: ");
  Serial.println(d2[3]);

  delay(500);
  // -------- OUTPUT --------
  // Serial.print("Yaw: ");
  // Serial.println(yaw * RAD_TO_DEG);
  // Serial.print(" | Pos: ");
  // Serial.print(px); Serial.print(",");
  // Serial.print(py); Serial.print(",");
  // Serial.println(pz);
  // Serial.print("ax "); Serial.print(ax_lin); Serial.print(",");
  // Serial.print("ay "); Serial.print(ay_lin); Serial.print(",");
  // Serial.print("az "); Serial.println(az_lin);
  

  delay(10);
}
