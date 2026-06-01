/*
  MPU Basic Reading Example
  
  Demonstrates how to read acceleration, gyroscope, and temperature
  data from an MPU sensor (MPU6050/MPU9250).
  
  Hardware:
  - Arduino board (Uno, Nano, Mega, etc.)
  - MPU6050 or MPU9250 sensor
  - I2C connections (SDA, SCL)
*/

#include <MPU.h>

// Create MPU object with default I2C address (0x68)
MPU mpu;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("Initializing MPU...");
  
  if (mpu.initialize()) {
    Serial.println("MPU initialized successfully!");
  } else {
    Serial.println("Failed to initialize MPU!");
    while (1);
  }
}

void loop() {
  // Read all data from the sensor
  mpu.readAll();
  
  // Print accelerometer data
  Serial.print("Accel (g): ");
  Serial.print("X="); Serial.print(mpu.getAccelX(), 2);
  Serial.print(" Y="); Serial.print(mpu.getAccelY(), 2);
  Serial.print(" Z="); Serial.println(mpu.getAccelZ(), 2);
  
  // Print gyroscope data
  Serial.print("Gyro (°/s): ");
  Serial.print("X="); Serial.print(mpu.getGyroX(), 2);
  Serial.print(" Y="); Serial.print(mpu.getGyroY(), 2);
  Serial.print(" Z="); Serial.println(mpu.getGyroZ(), 2);
  
  // Print temperature
  Serial.print("Temp: "); Serial.print(mpu.getTemp(), 2);
  Serial.println(" °C");
  
  Serial.println("---");
  delay(500);
}