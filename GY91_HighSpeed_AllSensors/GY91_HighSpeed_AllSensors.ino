/*
  ============================================================
  GY91_HighSpeed_AllSensors.ino
  Board: Adafruit HUZZAH32 Feather / ESP32
  Sensor module: GY-91 = MPU-9250 + BMP280

  Goal:
    - Read all GY-91 sensor values.
    - Keep accel/gyro fast.
    - Read magnetometer at its natural 100 Hz limit.
    - Read BMP280 slower so it does not disturb the IMU loop.

  Wiring for HUZZAH32 Feather using SPI:
    GY-91 VCC/VIN  -> 3V
    GY-91 GND      -> GND
    GY-91 SCL/SCK  -> GPIO 5   / SCK
    GY-91 SDA/SDI  -> GPIO 18  / MOSI
    GY-91 SDO/AD0  -> GPIO 19  / MISO
    GY-91 NCS      -> GPIO 33  / MPU-9250 CS
    GY-91 CSB      -> GPIO 32  / BMP280 CS
    GY-91 INT      -> GPIO 27  / optional, not used here

  Notes:
    - MPU-9250 uses SPI mode 3.
    - BMP280 uses SPI mode 0.
    - Both can share SCK/MOSI/MISO as long as they have separate CS pins.
  ============================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include "GY91_MPU9250.h"
#include "GY91_BMP280.h"

// ============================================================
// HUZZAH32 Feather SPI pin map
// ============================================================
static const int PIN_SPI_SCK  = 5;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 18;

static const int PIN_MPU_CS   = 33;
static const int PIN_BMP_CS   = 32;

// Optional interrupt pin. Not used in this basic polling sketch.
static const int PIN_MPU_INT  = 27;

// ============================================================
// Sensor objects
// ============================================================
GY91_MPU9250 imu(PIN_MPU_CS, SPI);
GY91_BMP280  bmp(PIN_BMP_CS, SPI);

GY91_IMUData imuData;
GY91_BMPData bmpData;

// Adjust this later using your local pressure/QNH for better altitude.
static const float SEA_LEVEL_HPA = 1013.25f;

// ============================================================
// Loop timing
// ============================================================
static const uint32_t BMP_PERIOD_US   = 20000;   // 50 Hz BMP280 read
static const uint32_t PRINT_PERIOD_US = 100000;  // 10 Hz Serial print

uint32_t lastBmpUs   = 0;
uint32_t lastPrintUs = 0;
uint32_t lastRateUs  = 0;

uint32_t imuCount = 0;
uint32_t bmpCount = 0;
uint32_t magNewCount = 0;

float imuHz = 0.0f;
float bmpHz = 0.0f;
float magHz = 0.0f;

void printHeader()
{
    Serial.println();
    Serial.println("==============================================");
    Serial.println("GY-91 High-Speed All Sensor Reader");
    Serial.println("MPU-9250: accel + gyro + mag");
    Serial.println("BMP280: pressure + temperature + altitude");
    Serial.println("==============================================");
}

void setup()
{
    Serial.begin(921600);
    delay(1000);

    printHeader();

    pinMode(PIN_MPU_CS, OUTPUT);
    pinMode(PIN_BMP_CS, OUTPUT);
    pinMode(PIN_MPU_INT, INPUT);

    digitalWrite(PIN_MPU_CS, HIGH);
    digitalWrite(PIN_BMP_CS, HIGH);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    Serial.println("[BOOT] Starting MPU-9250...");
    if (!imu.begin()) {
        Serial.printf("[ERROR] MPU-9250 not detected. WHO_AM_I=0x%02X\n", imu.whoAmI());
        Serial.println("        Check VCC=3V, GND, SCK, MOSI, MISO, NCS/CS.");
        while (true) delay(500);
    }

    Serial.printf("[OK] MPU-9250 detected. WHO_AM_I=0x%02X\n", imu.whoAmI());
    Serial.printf("[INFO] AK8963 magnetometer: %s\n", imu.isMagConnected() ? "detected" : "not detected");
    Serial.printf("[INFO] Mag ASA: X=%.4f Y=%.4f Z=%.4f\n", imu.magAsaX, imu.magAsaY, imu.magAsaZ);

    Serial.println("[BOOT] Starting BMP280...");
    if (!bmp.begin()) {
        Serial.printf("[ERROR] BMP280 not detected. CHIP_ID=0x%02X\n", bmp.chipID());
        Serial.println("        Check BMP CSB wiring. If your GY-91 board is I2C-only for BMP, this SPI code will not see it.");
        while (true) delay(500);
    }

    Serial.printf("[OK] BMP280 detected. CHIP_ID=0x%02X\n", bmp.chipID());
    Serial.println("[BOOT] Starting high-speed polling loop...");
    Serial.println();

    lastBmpUs = micros();
    lastPrintUs = micros();
    lastRateUs = micros();
}

void loop()
{
    const uint32_t nowUs = micros();

    // ========================================================
    // Fast path: read MPU-9250 as quickly as possible.
    // This gives accel, gyro, MPU temperature, and latest mag.
    // ========================================================
    if (imu.readAll(imuData)) {
        imuCount++;
        if (imuData.magNew) {
            magNewCount++;
        }
    }

    // ========================================================
    // Slow path: read BMP280 at 50 Hz.
    // Pressure/altitude do not belong in the fastest IMU loop.
    // ========================================================
    if ((uint32_t)(nowUs - lastBmpUs) >= BMP_PERIOD_US) {
        lastBmpUs += BMP_PERIOD_US;
        if (bmp.readAll(bmpData, SEA_LEVEL_HPA)) {
            bmpCount++;
        }
    }

    // ========================================================
    // Rate calculation once per second.
    // ========================================================
    if ((uint32_t)(nowUs - lastRateUs) >= 1000000UL) {
        float dt_s = (nowUs - lastRateUs) / 1000000.0f;

        imuHz = imuCount / dt_s;
        bmpHz = bmpCount / dt_s;
        magHz = magNewCount / dt_s;

        imuCount = 0;
        bmpCount = 0;
        magNewCount = 0;
        lastRateUs = nowUs;
    }

    // ========================================================
    // Print slowly. Printing every loop will destroy performance.
    // ========================================================
    if ((uint32_t)(nowUs - lastPrintUs) >= PRINT_PERIOD_US) {
        lastPrintUs += PRINT_PERIOD_US;

        Serial.printf(
            "Hz IMU=%7.1f MAG=%5.1f BMP=%5.1f | "
            "ACC[g] %+7.3f %+7.3f %+7.3f | "
            "GYRO[dps] %+8.2f %+8.2f %+8.2f | "
            "MAG[uT] %+7.1f %+7.1f %+7.1f valid=%d | "
            "MPU_T=%6.2fC | "
            "BMP_T=%6.2fC P=%8.2fhPa ALT=%7.2fm\n",

            imuHz,
            magHz,
            bmpHz,

            imuData.ax_g,
            imuData.ay_g,
            imuData.az_g,

            imuData.gx_dps,
            imuData.gy_dps,
            imuData.gz_dps,

            imuData.mx_uT,
            imuData.my_uT,
            imuData.mz_uT,
            imuData.magValid ? 1 : 0,

            imuData.temp_c,

            bmpData.temperature_c,
            bmpData.pressure_hpa,
            bmpData.altitude_m
        );
    }
}
