/*
  MPU.h
  Arduino library for MPU (Motion Processing Unit) sensors
  
  Author: Durvesh Pathak
  GitHub: https://github.com/durveshpathak1989/MPU-basics
  License: MIT
*/

#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Wire.h>

class MPU {
private:
  uint8_t i2cAddress;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;

public:
  MPU(uint8_t address = 0x68);
  bool initialize();
  void readAccel();
  void readGyro();
  void readTemp();
  void readAll();
  
  float getAccelX() { return accelX; }
  float getAccelY() { return accelY; }
  float getAccelZ() { return accelZ; }
  float getGyroX() { return gyroX; }
  float getGyroY() { return gyroY; }
  float getGyroZ() { return gyroZ; }
  float getTemp() { return temperature; }
};

#endif
