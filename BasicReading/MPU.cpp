/*
  MPU.cpp
  Arduino library for GY-91 module:
  - MPU9250 / MPU9255 accelerometer + gyroscope over SPI
  - AK8963 magnetometer over I2C path
  - BMP280 barometer over I2C

  Author: Durvesh Pathak
  GitHub: https://github.com/durveshpathak1989/MPU-basics
  License: MIT
*/

#include "MPU.h"

// ============================================================
// MPU9250 / MPU9255 register addresses
// ============================================================

static const uint8_t MPU_REG_WHO_AM_I       = 0x75;
static const uint8_t MPU_REG_PWR_MGMT_1     = 0x6B;
static const uint8_t MPU_REG_CONFIG         = 0x1A;
static const uint8_t MPU_REG_SMPLRT_DIV     = 0x19;
static const uint8_t MPU_REG_GYRO_CONFIG    = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG   = 0x1C;
static const uint8_t MPU_REG_ACCEL_XOUT_H   = 0x3B;
static const uint8_t MPU_REG_USER_CTRL      = 0x6A;
static const uint8_t MPU_REG_INT_PIN_CFG    = 0x37;

// ============================================================
// MPU SPI command masks
// ============================================================

static const uint8_t MPU_SPI_READ_BIT       = 0x80;
static const uint8_t MPU_SPI_ADDRESS_MASK   = 0x7F;

// ============================================================
// AK8963 magnetometer constants
// ============================================================

static const uint8_t AK8963_I2C_ADDR        = 0x0C;
static const uint8_t AK8963_REG_WHO_AM_I    = 0x00;
static const uint8_t AK8963_REG_CNTL1       = 0x0A;
static const uint8_t AK8963_EXPECTED_ID     = 0x48;

// ============================================================
// BMP280 constants
// ============================================================

static const uint8_t BMP280_I2C_ADDR_1      = 0x76;
static const uint8_t BMP280_I2C_ADDR_2      = 0x77;
static const uint8_t BMP280_REG_CHIP_ID     = 0xD0;
static const uint8_t BMP280_REG_CTRL_MEAS   = 0xF4;
static const uint8_t BMP280_REG_CONFIG      = 0xF5;
static const uint8_t BMP280_EXPECTED_ID     = 0x58;

// We will start with 0x76 and later make this dynamic if needed.
static uint8_t g_bmp280Address = BMP280_I2C_ADDR_1;

// ============================================================
// Constructor
// ============================================================

GY91Mpu::GY91Mpu(
  uint8_t mpuCsPin,
  uint8_t spiMosiPin,
  uint8_t spiMisoPin,
  uint8_t spiSclkPin,
  uint8_t i2cSdaPin,
  uint8_t i2cSclPin
)
  : _mpuCsPin(mpuCsPin),
    _spiMosiPin(spiMosiPin),
    _spiMisoPin(spiMisoPin),
    _spiSclkPin(spiSclkPin),
    _i2cSdaPin(i2cSdaPin),
    _i2cSclPin(i2cSclPin),
    _spi(VSPI),
    _mpuSpiSettings(1000000, MSBFIRST, SPI_MODE0)
{
  _status.mpu9250_ok = false;
  _status.ak8963_ok = false;
  _status.bmp280_ok = false;

  _status.mpu9250_whoami = 0;
  _status.ak8963_whoami = 0;
  _status.bmp280_chipid = 0;
}

// ============================================================
// Public begin()
// ============================================================

bool GY91Mpu::begin()
{
  pinMode(_mpuCsPin, OUTPUT);
  deselectMpu();

  // Start SPI for MPU9250 / MPU9255
  _spi.begin(_spiSclkPin, _spiMisoPin, _spiMosiPin, _mpuCsPin);

  // Start I2C for AK8963 and BMP280
  Wire.begin(_i2cSdaPin, _i2cSclPin);

  delay(100);

  bool mpuOk = beginMpu();

  // Magnetometer and barometer are not critical for basic attitude stabilization.
  bool magOk = beginMag();
  bool baroOk = beginBaro();

  _status.mpu9250_ok = mpuOk;
  _status.ak8963_ok = magOk;
  _status.bmp280_ok = baroOk;

  // For flight stabilization, MPU must work.
  // MAG and BARO can fail without blocking basic IMU operation.
  return mpuOk;
}

// ============================================================
// Status getter
// ============================================================

GY91Status GY91Mpu::getStatus() const
{
  return _status;
}

// ============================================================
// MPU chip select helpers
// ============================================================

void GY91Mpu::selectMpu()
{
  digitalWrite(_mpuCsPin, LOW);
}

void GY91Mpu::deselectMpu()
{
  digitalWrite(_mpuCsPin, HIGH);
}

// ============================================================
// MPU SPI register access
// ============================================================

bool GY91Mpu::writeMpuRegister(uint8_t reg, uint8_t value)
{
  _spi.beginTransaction(_mpuSpiSettings);
  selectMpu();

  _spi.transfer(reg & MPU_SPI_ADDRESS_MASK);
  _spi.transfer(value);

  deselectMpu();
  _spi.endTransaction();

  delayMicroseconds(10);

  return true;
}

uint8_t GY91Mpu::readMpuRegister(uint8_t reg)
{
  uint8_t value = 0;

  _spi.beginTransaction(_mpuSpiSettings);
  selectMpu();

  _spi.transfer(reg | MPU_SPI_READ_BIT);
  value = _spi.transfer(0x00);

  deselectMpu();
  _spi.endTransaction();

  delayMicroseconds(10);

  return value;
}

bool GY91Mpu::readMpuRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
  if (buffer == nullptr || length == 0)
  {
    return false;
  }

  _spi.beginTransaction(_mpuSpiSettings);
  selectMpu();

  _spi.transfer(startReg | MPU_SPI_READ_BIT);

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = _spi.transfer(0x00);
  }

  deselectMpu();
  _spi.endTransaction();

  return true;
}

// ============================================================
// MPU initialization
// ============================================================

bool GY91Mpu::beginMpu()
{
  delay(100);

  uint8_t whoami = readMpuRegister(MPU_REG_WHO_AM_I);
  _status.mpu9250_whoami = whoami;

  // Common IDs:
  // MPU9250 = 0x71
  // MPU9255 = 0x73
  if (whoami != 0x70 && whoami != 0x71 && whoami != 0x73)
  {
    return false;
  }

  // Wake up MPU
  writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Disable MPU internal I2C master for now.
  // We use ESP32 I2C bus separately for AK8963/BMP280 access.
  writeMpuRegister(MPU_REG_USER_CTRL, 0x00);
  delay(10);

  // Enable bypass mode so external I2C devices behind MPU can be visible.
  // This helps with AK8963 access on many MPU9250 boards.
  writeMpuRegister(MPU_REG_INT_PIN_CFG, 0x02);
  delay(10);

  // Digital low-pass filter.
  // 0x03 is a reasonable beginner setting.
  writeMpuRegister(MPU_REG_CONFIG, 0x03);

  // Sample rate divider.
  // Effective rate depends on gyro internal output rate.
  writeMpuRegister(MPU_REG_SMPLRT_DIV, 0x03);

  // Gyro full-scale range:
  // 0x00 = ±250 dps
  writeMpuRegister(MPU_REG_GYRO_CONFIG, 0x00);

  // Accel full-scale range:
  // 0x00 = ±2g
  writeMpuRegister(MPU_REG_ACCEL_CONFIG, 0x00);

  return true;
}

// ============================================================
// Read IMU data: accel + gyro + temperature
// ============================================================

bool GY91Mpu::readImu(IMUData &data)
{
  if (!_status.mpu9250_ok)
  {
    return false;
  }

  uint8_t buffer[14];

  bool ok = readMpuRegisters(MPU_REG_ACCEL_XOUT_H, buffer, 14);

  if (!ok)
  {
    return false;
  }

  int16_t rawAccelX = combineBytes(buffer[0], buffer[1]);
  int16_t rawAccelY = combineBytes(buffer[2], buffer[3]);
  int16_t rawAccelZ = combineBytes(buffer[4], buffer[5]);

  int16_t rawTemp   = combineBytes(buffer[6], buffer[7]);

  int16_t rawGyroX  = combineBytes(buffer[8], buffer[9]);
  int16_t rawGyroY  = combineBytes(buffer[10], buffer[11]);
  int16_t rawGyroZ  = combineBytes(buffer[12], buffer[13]);

  // Current config:
  // ACCEL_CONFIG = 0x00 => ±2g => 16384 LSB/g
  data.accelX_g = rawAccelX / 16384.0f;
  data.accelY_g = rawAccelY / 16384.0f;
  data.accelZ_g = rawAccelZ / 16384.0f;

  // Current config:
  // GYRO_CONFIG = 0x00 => ±250 dps => 131 LSB/(deg/s)
  data.gyroX_dps = rawGyroX / 131.0f;
  data.gyroY_dps = rawGyroY / 131.0f;
  data.gyroZ_dps = rawGyroZ / 131.0f;

  // MPU temperature formula
  data.temperature_C = (rawTemp / 333.87f) + 21.0f;

  data.timestamp_us = micros();

  return true;
}

// ============================================================
// Magnetometer initialization
// ============================================================

bool GY91Mpu::beginMag()
{
  uint8_t whoami = 0;

  bool ok = readMagRegister(AK8963_REG_WHO_AM_I, whoami);

  if (!ok)
  {
    _status.ak8963_whoami = 0;
    return false;
  }

  _status.ak8963_whoami = whoami;

  if (whoami != AK8963_EXPECTED_ID)
  {
    return false;
  }

  // Power down first
  writeMagRegister(AK8963_REG_CNTL1, 0x00);
  delay(10);

  // Continuous measurement mode 2, 16-bit output
  // 0x16 = 16-bit + 100 Hz continuous measurement
  writeMagRegister(AK8963_REG_CNTL1, 0x16);
  delay(10);

  return true;
}

// ============================================================
// Magnetometer I2C register access
// ============================================================

bool GY91Mpu::writeMagRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(AK8963_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool GY91Mpu::readMagRegister(uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(AK8963_I2C_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(AK8963_I2C_ADDR, (uint8_t)1) != 1)
  {
    return false;
  }

  value = Wire.read();

  return true;
}

bool GY91Mpu::readMagRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
  if (buffer == nullptr || length == 0)
  {
    return false;
  }

  Wire.beginTransmission(AK8963_I2C_ADDR);
  Wire.write(startReg);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(AK8963_I2C_ADDR, length) != length)
  {
    return false;
  }

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = Wire.read();
  }

  return true;
}

// ============================================================
// Read magnetometer data
// ============================================================

bool GY91Mpu::readMag(MAGData &data)
{
  if (!_status.ak8963_ok)
  {
    return false;
  }

  // AK8963 data registers start at 0x03:
  // HXL, HXH, HYL, HYH, HZL, HZH, ST2
  static const uint8_t AK8963_REG_HXL = 0x03;

  uint8_t buffer[7];

  bool ok = readMagRegisters(AK8963_REG_HXL, buffer, 7);

  if (!ok)
  {
    return false;
  }

  // AK8963 data is little-endian: low byte first, then high byte.
  int16_t rawMagX = combineBytes(buffer[1], buffer[0]);
  int16_t rawMagY = combineBytes(buffer[3], buffer[2]);
  int16_t rawMagZ = combineBytes(buffer[5], buffer[4]);

  // Very basic conversion for 16-bit mode:
  // 4912 uT full scale / 32760 counts ≈ 0.1499 uT/LSB.
  // Later we should apply factory sensitivity adjustment and calibration.
  const float magScale_uT_per_lsb = 4912.0f / 32760.0f;

  data.magX_uT = rawMagX * magScale_uT_per_lsb;
  data.magY_uT = rawMagY * magScale_uT_per_lsb;
  data.magZ_uT = rawMagZ * magScale_uT_per_lsb;

  data.timestamp_us = micros();

  return true;
}

// ============================================================
// BMP280 initialization
// ============================================================

bool GY91Mpu::beginBaro()
{
  uint8_t chipId = 0;

  // Try 0x76 first
  g_bmp280Address = BMP280_I2C_ADDR_1;

  bool ok = readBaroRegister(BMP280_REG_CHIP_ID, chipId);

  if (!ok || chipId != BMP280_EXPECTED_ID)
  {
    // Try 0x77
    g_bmp280Address = BMP280_I2C_ADDR_2;
    ok = readBaroRegister(BMP280_REG_CHIP_ID, chipId);
  }

  _status.bmp280_chipid = chipId;

  if (!ok || chipId != BMP280_EXPECTED_ID)
  {
    return false;
  }

  // Basic config:
  // ctrl_meas:
  // temp oversampling x1, pressure oversampling x1, normal mode
  writeBaroRegister(BMP280_REG_CTRL_MEAS, 0x27);

  // config:
  // standby 125 ms, filter off
  writeBaroRegister(BMP280_REG_CONFIG, 0x00);

  return true;
}

// ============================================================
// BMP280 I2C register access
// ============================================================

bool GY91Mpu::writeBaroRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(g_bmp280Address);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool GY91Mpu::readBaroRegister(uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(g_bmp280Address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(g_bmp280Address, (uint8_t)1) != 1)
  {
    return false;
  }

  value = Wire.read();

  return true;
}

bool GY91Mpu::readBaroRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
  if (buffer == nullptr || length == 0)
  {
    return false;
  }

  Wire.beginTransmission(g_bmp280Address);
  Wire.write(startReg);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(g_bmp280Address, length) != length)
  {
    return false;
  }

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = Wire.read();
  }

  return true;
}

// ============================================================
// Read BMP280 data
// ============================================================

bool GY91Mpu::readBaro(BAROData &data)
{
  if (!_status.bmp280_ok)
  {
    return false;
  }

  // This is a placeholder for now.
  // BMP280 requires compensation coefficients from calibration registers.
  // We will implement this properly in the next step.
  data.pressure_hPa = 0.0f;
  data.temperature_C = 0.0f;
  data.altitude_m = 0.0f;
  data.timestamp_us = micros();

  return true;
}

// ============================================================
// Utility helpers
// ============================================================

int16_t GY91Mpu::combineBytes(uint8_t highByte, uint8_t lowByte)
{
  return (int16_t)((highByte << 8) | lowByte);
}

int32_t GY91Mpu::combineThreeBytes(uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
  return ((int32_t)byte1 << 16) | ((int32_t)byte2 << 8) | byte3;
}