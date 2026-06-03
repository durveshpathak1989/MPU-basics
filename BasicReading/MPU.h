/*
  MPU.h
  Arduino library for GY-91 module:
  - MPU9250 / MPU9255 accelerometer + gyroscope
  - AK8963 magnetometer
  - BMP280 barometer

  Author: Durvesh Pathak
  GitHub: https://github.com/durveshpathak1989/MPU-basics
  License: MIT
*/

#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ============================================================
// IMU data: accelerometer + gyroscope + MPU temperature
// ============================================================
struct IMUData {
  float accelX_g;
  float accelY_g;
  float accelZ_g;

  float gyroX_dps;
  float gyroY_dps;
  float gyroZ_dps;

  float temperature_C;
  uint32_t timestamp_us;
};

// ============================================================
// Magnetometer data
// Unit: microtesla
// ============================================================
struct MAGData {
  float magX_uT;
  float magY_uT;
  float magZ_uT;

  uint32_t timestamp_us;
};

// ============================================================
// Barometer data
// ============================================================
struct BAROData {
  float pressure_hPa;
  float temperature_C;
  float altitude_m;

  uint32_t timestamp_us;
};

// ============================================================
// GY-91 device detection and health status
// ============================================================
struct GY91Status {
  bool mpu9250_ok;
  bool ak8963_ok;
  bool bmp280_ok;

  uint8_t mpu9250_whoami;
  uint8_t ak8963_whoami;
  uint8_t bmp280_chipid;
};

// ============================================================
// GY-91 driver class
// ============================================================
class GY91Mpu {

public:
  GY91Mpu(
    uint8_t mpuCsPin,
    uint8_t spiMosiPin,
    uint8_t spiMisoPin,
    uint8_t spiSclkPin,
    uint8_t i2cSdaPin,
    uint8_t i2cSclPin
  );

  bool begin();

  bool readImu(IMUData &data);
  bool readMag(MAGData &data);
  bool readBaro(BAROData &data);

  GY91Status getStatus() const;

private:
  // ----------------------------
  // Pin configuration
  // ----------------------------
  uint8_t _mpuCsPin;
  uint8_t _spiMosiPin;
  uint8_t _spiMisoPin;
  uint8_t _spiSclkPin;
  uint8_t _i2cSdaPin;
  uint8_t _i2cSclPin;

  // ----------------------------
  // Communication configuration
  // ----------------------------
  SPIClass _spi;
  SPISettings _mpuSpiSettings;

  // ----------------------------
  // Device status
  // ----------------------------
  GY91Status _status;

  // ----------------------------
  // MPU9250 / MPU9255 SPI helpers
  // ----------------------------
  void selectMpu();
  void deselectMpu();

  bool beginMpu();

  bool writeMpuRegister(uint8_t reg, uint8_t value);
  uint8_t readMpuRegister(uint8_t reg);
  bool readMpuRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length);

  // ----------------------------
  // AK8963 magnetometer helpers
  // ----------------------------
  bool beginMag();

  bool writeMagRegister(uint8_t reg, uint8_t value);
  bool readMagRegister(uint8_t reg, uint8_t &value);
  bool readMagRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length);

  // ----------------------------
  // BMP280 barometer helpers
  // ----------------------------
  bool beginBaro();

  bool writeBaroRegister(uint8_t reg, uint8_t value);
  bool readBaroRegister(uint8_t reg, uint8_t &value);
  bool readBaroRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length);

  // ----------------------------
  // Utility helpers
  // ----------------------------
  int16_t combineBytes(uint8_t highByte, uint8_t lowByte);
  int32_t combineThreeBytes(uint8_t byte1, uint8_t byte2, uint8_t byte3);
};

#endif