#include "MPU.h"

// I2C Registers
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_TEMP_OUT_H   0x41
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_PWR_MGMT_1   0x6B

MPU::MPU(uint8_t address) : i2cAddress(address) {}

bool MPU::initialize() {
  Wire.begin();
  Wire.beginTransmission(i2cAddress);
  Wire.write(MPU_REG_PWR_MGMT_1);
  Wire.write(0x00);  // Wake up the sensor
  byte result = Wire.endTransmission();
  return result == 0;
}

void MPU::readAccel() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, (uint8_t)6);
  
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  
  accelX = ax / 16384.0f;
  accelY = ay / 16384.0f;
  accelZ = az / 16384.0f;
}

void MPU::readGyro() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(MPU_REG_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, (uint8_t)6);
  
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  
  gyroX = gx / 131.0f;
  gyroY = gy / 131.0f;
  gyroZ = gz / 131.0f;
}

void MPU::readTemp() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(MPU_REG_TEMP_OUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  temperature = (rawTemp / 340.0f) + 36.53f;
}

void MPU::readAll() {
  readAccel();
  readGyro();
  readTemp();
}