/*
  GY91 9-Axis IMU Basic Reading Example
  
  Demonstrates how to read acceleration, gyroscope, and temperature
  data from the MPU9250 in a GY91 sensor module.
  
  Author: Durvesh Pathak
  GitHub: https://github.com/durveshpathak1989/MPU-basics
  License: MIT
  
  Hardware:
  - Arduino board (Uno, Nano, Mega, etc.)
  - GY91 9-axis IMU module (MPU9250 + AK8963 magnetometer)
  - I2C connections (SDA, SCL)
  
  GY91 Wiring:
  VCC → 5V
  GND → GND
  SDA → A4 (or SDA pin)
  SCL → A5 (or SCL pin)
*/

#include <MPU.h>

// Create MPU object with default I2C address (0x68)
// This reads from the MPU9250 portion of the GY91 module
MPU mpu;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("=== GY91 9-Axis IMU Test ===");
  Serial.println("Initializing MPU9250...");
  
  if (mpu.initialize()) {
    Serial.println("✓ MPU9250 initialized successfully!");
  } else {
    Serial.println("✗ Failed to initialize MPU9250!");
    while (1);
  }
  
  Serial.println("\nStarting sensor readings...");
  Serial.println();
}

void loop() {
  // Read all data from MPU9250
  mpu.readAll();
  
  // Print accelerometer data (6-axis IMU)
  Serial.print("Accel (g): ");
  Serial.print("X="); Serial.print(mpu.getAccelX(), 2);
  Serial.print(" Y="); Serial.print(mpu.getAccelY(), 2);
  Serial.print(" Z="); Serial.print(mpu.getAccelZ(), 2);
  Serial.println();
  
  // Print gyroscope data (6-axis IMU)
  Serial.print("Gyro (°/s): ");
  Serial.print("X="); Serial.print(mpu.getGyroX(), 2);
  Serial.print(" Y="); Serial.print(mpu.getGyroY(), 2);
  Serial.print(" Z="); Serial.print(mpu.getGyroZ(), 2);
  Serial.println();
  
  // Print temperature (from MPU9250)
  Serial.print("Temperature: ");
  Serial.print(mpu.getTemp(), 1);
  Serial.println(" °C");
  
  // Note: For magnetometer (AK8963) readings, see the GY91 advanced examples
  Serial.println("---");
  delay(500);
}
