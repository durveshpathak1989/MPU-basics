#pragma once
#ifndef GY91_MPU9250_H
#define GY91_MPU9250_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================
// GY91_MPU9250.h
// Minimal high-speed MPU-9250 SPI driver for GY-91 + ESP32
// Reads accel, gyro, die temperature, and AK8963 magnetometer.
// ============================================================

struct GY91_IMUData {
    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float mx_uT;
    float my_uT;
    float mz_uT;

    float temp_c;

    bool magNew;
    bool magValid;
    uint32_t timestamp_us;
};

struct GY91_RawIMUData {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t mx;
    int16_t my;
    int16_t mz;
    bool magOk;
};

class GY91_MPU9250 {
public:
    explicit GY91_MPU9250(uint8_t csPin, SPIClass& spi = SPI);

    bool begin();
    bool isConnected();
    uint8_t whoAmI();
    bool isMagConnected();

    bool readRaw(GY91_RawIMUData& out);
    bool readAll(GY91_IMUData& out);

    // Basic runtime calibration offsets.
    // For first bring-up, leave these at zero.
    float gyroBiasX_dps = 0.0f;
    float gyroBiasY_dps = 0.0f;
    float gyroBiasZ_dps = 0.0f;

    float accelBiasX_g = 0.0f;
    float accelBiasY_g = 0.0f;
    float accelBiasZ_g = 0.0f;

    // Magnetometer factory sensitivity adjustment.
    float magAsaX = 1.0f;
    float magAsaY = 1.0f;
    float magAsaZ = 1.0f;

private:
    uint8_t _cs;
    SPIClass& _spi;

    bool _magConnected = false;
    bool _magValid = false;
    float _lastMx_uT = 0.0f;
    float _lastMy_uT = 0.0f;
    float _lastMz_uT = 0.0f;

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void burstRead(uint8_t reg, uint8_t* buf, uint8_t len);

    void akWrite(uint8_t akReg, uint8_t val);
    uint8_t akReadByte(uint8_t akReg);
    void akBurstRead(uint8_t akReg, uint8_t* buf, uint8_t len);
    void readMagASA();
    void setupContinuousMag();
};

#endif
