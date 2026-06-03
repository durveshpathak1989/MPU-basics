#include "GY91_MPU9250.h"

// ============================================================
// MPU-9250 register map
// ============================================================
static constexpr uint8_t MPU_REG_SMPLRT_DIV      = 0x19;
static constexpr uint8_t MPU_REG_CONFIG          = 0x1A;
static constexpr uint8_t MPU_REG_GYRO_CONFIG     = 0x1B;
static constexpr uint8_t MPU_REG_ACCEL_CONFIG    = 0x1C;
static constexpr uint8_t MPU_REG_ACCEL_CONFIG2   = 0x1D;
static constexpr uint8_t MPU_REG_INT_PIN_CFG     = 0x37;
static constexpr uint8_t MPU_REG_INT_ENABLE      = 0x38;
static constexpr uint8_t MPU_REG_ACCEL_XOUT_H    = 0x3B;
static constexpr uint8_t MPU_REG_EXT_SENS_DATA   = 0x49;
static constexpr uint8_t MPU_REG_USER_CTRL       = 0x6A;
static constexpr uint8_t MPU_REG_PWR_MGMT_1      = 0x6B;
static constexpr uint8_t MPU_REG_PWR_MGMT_2      = 0x6C;
static constexpr uint8_t MPU_REG_WHO_AM_I        = 0x75;

static constexpr uint8_t MPU_WHO_AM_I_9250       = 0x71;
static constexpr uint8_t MPU_WHO_AM_I_9255       = 0x73;
static constexpr uint8_t MPU_WHO_AM_I_6500       = 0x70;

// ============================================================
// MPU internal I2C master registers for AK8963 magnetometer
// ============================================================
static constexpr uint8_t AK8963_ADDR             = 0x0C;
static constexpr uint8_t AK8963_WHO_AM_I_EXPECT  = 0x48;

static constexpr uint8_t MPU_REG_I2C_MST_CTRL    = 0x24;
static constexpr uint8_t MPU_REG_I2C_SLV0_ADDR   = 0x25;
static constexpr uint8_t MPU_REG_I2C_SLV0_REG    = 0x26;
static constexpr uint8_t MPU_REG_I2C_SLV0_CTRL   = 0x27;
static constexpr uint8_t MPU_REG_I2C_SLV4_ADDR   = 0x31;
static constexpr uint8_t MPU_REG_I2C_SLV4_REG    = 0x32;
static constexpr uint8_t MPU_REG_I2C_SLV4_DO     = 0x33;
static constexpr uint8_t MPU_REG_I2C_SLV4_CTRL   = 0x34;
static constexpr uint8_t MPU_REG_I2C_MST_STATUS  = 0x36;

static constexpr uint8_t AK_REG_WIA              = 0x00;
static constexpr uint8_t AK_REG_ST1              = 0x02;
static constexpr uint8_t AK_REG_CNTL1            = 0x0A;
static constexpr uint8_t AK_REG_ASAX             = 0x10;

// ============================================================
// Scale factors for high-performance configuration
// Accel: +-16 g, Gyro: +-2000 dps, Mag: 16-bit AK8963
// ============================================================
static constexpr float ACCEL_SCALE_16G       = 16.0f / 32768.0f;
static constexpr float GYRO_SCALE_2000DPS    = 2000.0f / 32768.0f;
static constexpr float MAG_SCALE_16BIT_uT    = 4912.0f / 32760.0f;

static constexpr uint32_t MPU_SPI_SLOW_HZ    = 1000000UL;
static constexpr uint32_t MPU_SPI_FAST_HZ    = 20000000UL;

GY91_MPU9250::GY91_MPU9250(uint8_t csPin, SPIClass& spi)
    : _cs(csPin), _spi(spi) {}

void GY91_MPU9250::writeReg(uint8_t reg, uint8_t val) {
    _spi.beginTransaction(SPISettings(MPU_SPI_SLOW_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg & 0x7F);
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t GY91_MPU9250::readReg(uint8_t reg) {
    _spi.beginTransaction(SPISettings(MPU_SPI_FAST_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);
    uint8_t val = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void GY91_MPU9250::burstRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    _spi.beginTransaction(SPISettings(MPU_SPI_FAST_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _spi.transfer(0x00);
    }
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void GY91_MPU9250::akWrite(uint8_t akReg, uint8_t val) {
    // Use SLV4 one-shot write so SLV0 can later be dedicated to continuous reads.
    writeReg(MPU_REG_I2C_SLV4_ADDR, AK8963_ADDR);
    writeReg(MPU_REG_I2C_SLV4_REG, akReg);
    writeReg(MPU_REG_I2C_SLV4_DO, val);
    writeReg(MPU_REG_I2C_SLV4_CTRL, 0x80);

    uint32_t startMs = millis();
    while (((readReg(MPU_REG_I2C_MST_STATUS) & 0x40) == 0) && ((millis() - startMs) < 10)) {
        delayMicroseconds(100);
    }
    delay(1);
}

uint8_t GY91_MPU9250::akReadByte(uint8_t akReg) {
    writeReg(MPU_REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
    writeReg(MPU_REG_I2C_SLV0_REG, akReg);
    writeReg(MPU_REG_I2C_SLV0_CTRL, 0x81);
    delay(10);
    return readReg(MPU_REG_EXT_SENS_DATA);
}

void GY91_MPU9250::akBurstRead(uint8_t akReg, uint8_t* buf, uint8_t len) {
    writeReg(MPU_REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
    writeReg(MPU_REG_I2C_SLV0_REG, akReg);
    writeReg(MPU_REG_I2C_SLV0_CTRL, 0x80 | (len & 0x0F));
    delay(10);
    burstRead(MPU_REG_EXT_SENS_DATA, buf, len);
}

void GY91_MPU9250::readMagASA() {
    akWrite(AK_REG_CNTL1, 0x00); // power down
    delay(10);
    akWrite(AK_REG_CNTL1, 0x0F); // fuse ROM mode
    delay(10);

    uint8_t asa[3] = {0, 0, 0};
    akBurstRead(AK_REG_ASAX, asa, 3);

    magAsaX = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;
    magAsaY = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;
    magAsaZ = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;

    akWrite(AK_REG_CNTL1, 0x00); // power down before mode change
    delay(10);
}

void GY91_MPU9250::setupContinuousMag() {
    // Read 8 bytes every MPU sample: ST1 + HXL/HXH + HYL/HYH + HZL/HZH + ST2.
    // Reading ST2 is required to release the AK8963 data registers.
    writeReg(MPU_REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
    writeReg(MPU_REG_I2C_SLV0_REG, AK_REG_ST1);
    writeReg(MPU_REG_I2C_SLV0_CTRL, 0x88);
}

bool GY91_MPU9250::begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    delay(10);

    uint8_t who = whoAmI();

    if (who != MPU_WHO_AM_I_9250 &&
        who != MPU_WHO_AM_I_6500) {

        Serial.printf("[MPU] WHO_AM_I=0x%02X expected 0x70 or 0x71\n", who);
        return false;
    }

    Serial.printf("[MPU] WHO_AM_I=0x%02X OK\n", who);

    if (who == MPU_WHO_AM_I_6500) {
        Serial.println("[MPU] Detected 0x70 device. Accel/gyro should work; magnetometer may not be available.");
    }

    // Reset MPU.
    writeReg(MPU_REG_PWR_MGMT_1, 0x80);
    delay(100);

    // Wake up; use best available clock source.
    writeReg(MPU_REG_PWR_MGMT_1, 0x01);
    delay(10);

    // Enable all accel and gyro axes.
    writeReg(MPU_REG_PWR_MGMT_2, 0x00);
    delay(10);

    // High-performance settings:
    // Gyro DLPF ~184 Hz, internal sample 1 kHz when DLPF enabled.
    writeReg(MPU_REG_CONFIG, 0x01);
    writeReg(MPU_REG_SMPLRT_DIV, 0x00);     // 1000 Hz / (1 + 0) = 1000 Hz
    writeReg(MPU_REG_GYRO_CONFIG, 0x18);    // +-2000 dps
    writeReg(MPU_REG_ACCEL_CONFIG, 0x18);   // +-16 g
    writeReg(MPU_REG_ACCEL_CONFIG2, 0x01);  // accel DLPF fast setting

    // Enable MPU internal I2C master for AK8963 while using MPU over SPI.
    writeReg(MPU_REG_USER_CTRL, 0x20);
    delay(10);
    writeReg(MPU_REG_I2C_MST_CTRL, 0x0D);   // 400 kHz internal I2C master
    delay(10);

    // Optional data-ready interrupt config.
    writeReg(MPU_REG_INT_PIN_CFG, 0x30);
    writeReg(MPU_REG_INT_ENABLE, 0x01);

    uint8_t akWho = akReadByte(AK_REG_WIA);
    _magConnected = (akWho == AK8963_WHO_AM_I_EXPECT);

    if (_magConnected) {
        readMagASA();
        // AK8963 continuous measurement 2, 16-bit, 100 Hz.
        akWrite(AK_REG_CNTL1, 0x16);
        delay(10);
        setupContinuousMag();
        delay(50);
    }

    return true;
}

bool GY91_MPU9250::isConnected() {
    uint8_t who = whoAmI();
    return (who == MPU_WHO_AM_I_9250 || who == MPU_WHO_AM_I_9255);
}

uint8_t GY91_MPU9250::whoAmI() {
    return readReg(MPU_REG_WHO_AM_I);
}

bool GY91_MPU9250::isMagConnected() {
    return _magConnected;
}

bool GY91_MPU9250::readRaw(GY91_RawIMUData& out) {
    uint8_t buf[14];
    burstRead(MPU_REG_ACCEL_XOUT_H, buf, 14);

    out.ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    out.ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    out.az   = (int16_t)((buf[4]  << 8) | buf[5]);
    out.temp = (int16_t)((buf[6]  << 8) | buf[7]);
    out.gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    out.gy   = (int16_t)((buf[10] << 8) | buf[11]);
    out.gz   = (int16_t)((buf[12] << 8) | buf[13]);

    out.mx = 0;
    out.my = 0;
    out.mz = 0;
    out.magOk = false;

    if (_magConnected) {
        uint8_t mag[8];
        burstRead(MPU_REG_EXT_SENS_DATA, mag, 8);
        bool drdy = (mag[0] & 0x01) != 0;
        bool hofl = (mag[7] & 0x08) != 0;
        out.magOk = drdy && !hofl;

        if (out.magOk) {
            // AK8963 data is little-endian.
            out.mx = (int16_t)((mag[2] << 8) | mag[1]);
            out.my = (int16_t)((mag[4] << 8) | mag[3]);
            out.mz = (int16_t)((mag[6] << 8) | mag[5]);
        }
    }

    return true;
}

bool GY91_MPU9250::readAll(GY91_IMUData& out) {
    GY91_RawIMUData raw;
    if (!readRaw(raw)) {
        return false;
    }

    out.ax_g = (float)raw.ax * ACCEL_SCALE_16G - accelBiasX_g;
    out.ay_g = (float)raw.ay * ACCEL_SCALE_16G - accelBiasY_g;
    out.az_g = (float)raw.az * ACCEL_SCALE_16G - accelBiasZ_g;

    out.gx_dps = (float)raw.gx * GYRO_SCALE_2000DPS - gyroBiasX_dps;
    out.gy_dps = (float)raw.gy * GYRO_SCALE_2000DPS - gyroBiasY_dps;
    out.gz_dps = (float)raw.gz * GYRO_SCALE_2000DPS - gyroBiasZ_dps;

    out.magNew = raw.magOk;
    if (raw.magOk) {
        _lastMx_uT = (float)raw.mx * MAG_SCALE_16BIT_uT * magAsaX;
        _lastMy_uT = (float)raw.my * MAG_SCALE_16BIT_uT * magAsaY;
        _lastMz_uT = (float)raw.mz * MAG_SCALE_16BIT_uT * magAsaZ;
        _magValid = true;
    }

    out.mx_uT = _lastMx_uT;
    out.my_uT = _lastMy_uT;
    out.mz_uT = _lastMz_uT;
    out.magValid = _magValid;

    out.temp_c = (float)raw.temp / 333.87f + 21.0f;
    out.timestamp_us = micros();

    return true;
}
