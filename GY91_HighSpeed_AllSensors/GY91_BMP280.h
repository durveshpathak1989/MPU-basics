#pragma once
#ifndef GY91_BMP280_H
#define GY91_BMP280_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================
// GY91_BMP280.h
// Minimal BMP280 SPI driver for GY-91 + ESP32
// Reads pressure, temperature, and altitude estimate.
// ============================================================

struct GY91_BMPData {
    float temperature_c;
    float pressure_pa;
    float pressure_hpa;
    float altitude_m;
    uint32_t timestamp_us;
};

class GY91_BMP280 {
public:
    explicit GY91_BMP280(uint8_t csPin, SPIClass& spi = SPI);

    bool begin();
    bool isConnected();
    uint8_t chipID();

    bool readAll(GY91_BMPData& out, float seaLevel_hPa = 1013.25f);

private:
    uint8_t _cs;
    SPIClass& _spi;

    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    int32_t t_fine;

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void burstRead(uint8_t reg, uint8_t* buf, uint8_t len);

    bool readCalibration();
    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
};

#endif
