#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>

LSM6 imu;
LIS3MDL mag;
LPS baro;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();

  // Init gyro + accelerometer
  if (!imu.init()) {
    Serial.println("LSM6DSO not found!");
    while (1);
  }
  imu.enableDefault();

  // Init magnetometer
  if (!mag.init()) {
    Serial.println("LIS3MDL not found!");
    while (1);
  }
  mag.enableDefault();

  // Init barometer
  if (!baro.init()) {
    Serial.println("LPS22DF not found!");
    while (1);
  }
  baro.enableDefault();

  Serial.println("All sensors ready.");
}

void loop() {
  // --- Gyro + Accelerometer ---
  imu.read();

  Serial.println("=== Accelerometer (raw) ===");
  Serial.print("X: "); Serial.print(imu.a.x);
  Serial.print("  Y: "); Serial.print(imu.a.y);
  Serial.print("  Z: "); Serial.println(imu.a.z);

  // Convert to g (±2g default, sensitivity = 0.061 mg/LSB)
  float ax = imu.a.x * 0.000061;
  float ay = imu.a.y * 0.000061;
  float az = imu.a.z * 0.000061;
  Serial.println("=== Accelerometer (g) ===");
  Serial.print("X: "); Serial.print(ax, 3);
  Serial.print("  Y: "); Serial.print(ay, 3);
  Serial.print("  Z: "); Serial.println(az, 3);

  Serial.println("=== Gyro (raw) ===");
  Serial.print("X: "); Serial.print(imu.g.x);
  Serial.print("  Y: "); Serial.print(imu.g.y);
  Serial.print("  Z: "); Serial.println(imu.g.z);

  // Convert to degrees/s (±245 dps default, sensitivity = 8.75 mdps/LSB)
  float gx = imu.g.x * 0.00875;
  float gy = imu.g.y * 0.00875;
  float gz = imu.g.z * 0.00875;
  Serial.println("=== Gyro (deg/s) ===");
  Serial.print("X: "); Serial.print(gx, 3);
  Serial.print("  Y: "); Serial.print(gy, 3);
  Serial.print("  Z: "); Serial.println(gz, 3);

  // --- Magnetometer ---
  mag.read();
  Serial.println("=== Magnetometer (raw) ===");
  Serial.print("X: "); Serial.print(mag.m.x);
  Serial.print("  Y: "); Serial.print(mag.m.y);
  Serial.print("  Z: "); Serial.println(mag.m.z);

  // Convert to gauss (±4 gauss default, sensitivity = 6842 LSB/gauss)
  float mx = mag.m.x / 6842.0;
  float my = mag.m.y / 6842.0;
  float mz = mag.m.z / 6842.0;
  Serial.println("=== Magnetometer (gauss) ===");
  Serial.print("X: "); Serial.print(mx, 4);
  Serial.print("  Y: "); Serial.print(my, 4);
  Serial.print("  Z: "); Serial.println(mz, 4);

  // --- Barometer ---
  float pressure = baro.readPressureMillibars();
  float altitude = baro.pressureToAltitudeMeters(pressure);
  float temperature = baro.readTemperatureC();

  Serial.println("=== Barometer ===");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" mbar");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");

  Serial.println("----------------------------");
  delay(500);
}