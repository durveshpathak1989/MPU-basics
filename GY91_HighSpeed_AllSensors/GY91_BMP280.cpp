#include "GY91_BMP280.h"
#include <math.h>

// ============================================================
// BMP280 registers
// ============================================================
static constexpr uint8_t BMP_REG_ID          = 0xD0;
static constexpr uint8_t BMP_REG_RESET       = 0xE0;
static constexpr uint8_t BMP_REG_STATUS      = 0xF3;
static constexpr uint8_t BMP_REG_CTRL_MEAS   = 0xF4;
static constexpr uint8_t BMP_REG_CONFIG      = 0xF5;
static constexpr uint8_t BMP_REG_PRESS_MSB   = 0xF7;
static constexpr uint8_t BMP_REG_CALIB_START = 0x88;

static constexpr uint8_t BMP280_CHIP_ID      = 0x58;
static constexpr uint32_t BMP_SPI_HZ         = 10000000UL;

GY91_BMP280::GY91_BMP280(uint8_t csPin, SPIClass& spi)
    : _cs(csPin), _spi(spi), t_fine(0) {}

void GY91_BMP280::writeReg(uint8_t reg, uint8_t val) {
    _spi.beginTransaction(SPISettings(BMP_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg & 0x7F);
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t GY91_BMP280::readReg(uint8_t reg) {
    _spi.beginTransaction(SPISettings(BMP_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);
    uint8_t val = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void GY91_BMP280::burstRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    _spi.beginTransaction(SPISettings(BMP_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _spi.transfer(0x00);
    }
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t GY91_BMP280::chipID() {
    return readReg(BMP_REG_ID);
}

bool GY91_BMP280::isConnected() {
    return chipID() == BMP280_CHIP_ID;
}

bool GY91_BMP280::begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    delay(10);

    if (!isConnected()) {
        return false;
    }

    // Soft reset.
    writeReg(BMP_REG_RESET, 0xB6);
    delay(10);

    if (!readCalibration()) {
        return false;
    }

    // High-speed/basic configuration:
    // config: standby 0.5 ms, filter off, 3-wire SPI off.
    writeReg(BMP_REG_CONFIG, 0x00);

    // ctrl_meas: temp oversampling x1, pressure oversampling x1, normal mode.
    // osrs_t = 001, osrs_p = 001, mode = 11 => 0b00100111 = 0x27.
    writeReg(BMP_REG_CTRL_MEAS, 0x27);

    delay(10);
    return true;
}

bool GY91_BMP280::readCalibration() {
    uint8_t c[24];
    burstRead(BMP_REG_CALIB_START, c, 24);

    dig_T1 = (uint16_t)(c[0] | (c[1] << 8));
    dig_T2 = (int16_t)(c[2] | (c[3] << 8));
    dig_T3 = (int16_t)(c[4] | (c[5] << 8));

    dig_P1 = (uint16_t)(c[6] | (c[7] << 8));
    dig_P2 = (int16_t)(c[8] | (c[9] << 8));
    dig_P3 = (int16_t)(c[10] | (c[11] << 8));
    dig_P4 = (int16_t)(c[12] | (c[13] << 8));
    dig_P5 = (int16_t)(c[14] | (c[15] << 8));
    dig_P6 = (int16_t)(c[16] | (c[17] << 8));
    dig_P7 = (int16_t)(c[18] | (c[19] << 8));
    dig_P8 = (int16_t)(c[20] | (c[21] << 8));
    dig_P9 = (int16_t)(c[22] | (c[23] << 8));

    return dig_T1 != 0 && dig_P1 != 0;
}

int32_t GY91_BMP280::compensateTemperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8;
    return T; // temperature in 0.01 deg C
}

uint32_t GY91_BMP280::compensatePressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (uint32_t)p; // Q24.8 Pa
}

bool GY91_BMP280::readAll(GY91_BMPData& out, float seaLevel_hPa) {
    uint8_t data[6];
    burstRead(BMP_REG_PRESS_MSB, data, 6);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    int32_t temp_x100 = compensateTemperature(adc_T);
    uint32_t pressure_q24_8 = compensatePressure(adc_P);

    out.temperature_c = temp_x100 / 100.0f;
    out.pressure_pa = pressure_q24_8 / 256.0f;
    out.pressure_hpa = out.pressure_pa / 100.0f;

    if (out.pressure_hpa > 0.0f && seaLevel_hPa > 0.0f) {
        out.altitude_m = 44330.0f * (1.0f - powf(out.pressure_hpa / seaLevel_hPa, 0.1903f));
    } else {
        out.altitude_m = 0.0f;
    }

    out.timestamp_us = micros();
    return true;
}
