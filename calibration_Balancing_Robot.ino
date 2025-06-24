// Fast Calibration Sketch for MPU6050
// Based on Luis Ródenas' original, optimized for speed

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// TUNING FOR SPEED
int buffersize     = 500;  // Reduced for faster averaging
int acel_deadzone  = 10;   // Looser accel tolerance
int giro_deadzone  = 2;    // Looser gyro tolerance

MPU6050 accelgyro(0x68); // Default I2C address

int16_t ax, ay, az, gx, gy, gz;
long mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int state = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  accelgyro.initialize();

  while (Serial.available() && Serial.read());
  while (!Serial.available()) {
    Serial.println(F("Send any key to start MPU6050 fast calibration..."));
    delay(1500);
  }
  while (Serial.available() && Serial.read());

  Serial.println(F("\nStarting fast calibration..."));
  delay(2000);
  Serial.println(F("Keep sensor flat and still."));
  delay(3000);

  Serial.println(accelgyro.testConnection() ? "Connection successful" : "Connection failed");

  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
}

void loop() {
  if (state == 0) {
    Serial.println("\nReading baseline...");
    meansensors();
    state++;
    delay(1000);
  } else if (state == 1) {
    Serial.println("Calibrating offsets...");
    calibration();
    state++;
    delay(1000);
  } else if (state == 2) {
    meansensors();
    Serial.println("\nDone!");
    Serial.print("Final sensor values:\t");
    Serial.print(mean_ax); Serial.print("\t");
    Serial.print(mean_ay); Serial.print("\t");
    Serial.print(mean_az); Serial.print("\t");
    Serial.print(mean_gx); Serial.print("\t");
    Serial.print(mean_gy); Serial.print("\t");
    Serial.println(mean_gz);

    Serial.print("Offsets to use:\t");
    Serial.print(ax_offset); Serial.print("\t");
    Serial.print(ay_offset); Serial.print("\t");
    Serial.print(az_offset); Serial.print("\t");
    Serial.print(gx_offset); Serial.print("\t");
    Serial.print(gy_offset); Serial.print("\t");
    Serial.println(gz_offset);

    Serial.println(F("\nIdeal values: accel = 0 0 16384, gyro = 0 0 0"));
    while (1); // halt
  }
}

void meansensors() {
  long buff_ax = 0, buff_ay = 0, buff_az = 0;
  long buff_gx = 0, buff_gy = 0, buff_gz = 0;

  for (long i = 0; i < buffersize + 100; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i >= 100) {
      buff_ax += ax; buff_ay += ay; buff_az += az;
      buff_gx += gx; buff_gy += gy; buff_gz += gz;
    }
    delay(2);
  }

  mean_ax = buff_ax / buffersize;
  mean_ay = buff_ay / buffersize;
  mean_az = buff_az / buffersize;
  mean_gx = buff_gx / buffersize;
  mean_gy = buff_gy / buffersize;
  mean_gz = buff_gz / buffersize;
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (true) {
    int ready = 0;

    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println(F("..."));

    if (abs(mean_ax) <= acel_deadzone) ready++; else ax_offset -= mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone) ready++; else ay_offset -= mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone) ready++; else az_offset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++; else gx_offset -= mean_gx / (giro_deadzone + 1);
    if (abs(mean_gy) <= giro_deadzone) ready++; else gy_offset -= mean_gy / (giro_deadzone + 1);
    if (abs(mean_gz) <= giro_deadzone) ready++; else gz_offset -= mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}
