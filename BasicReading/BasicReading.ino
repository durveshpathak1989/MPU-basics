/*
  GY91_HighSpeed_AllSensors.ino

  Board: Adafruit HUZZAH32 Feather / ESP32
  Sensor: GY-91 = MPU-9250 + BMP280

  SPI wiring:
    GY-91 VCC/VIN  -> 3V
    GY-91 GND      -> GND
    GY-91 SCL/SCK  -> GPIO 5
    GY-91 SDA/SDI  -> GPIO 18
    GY-91 SDO/AD0  -> GPIO 19
    GY-91 NCS      -> GPIO 33   // MPU-9250 CS
    GY-91 CSB      -> GPIO 32   // BMP280 CS
    GY-91 INT      -> GPIO 27   // optional
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"

// ===============================
// Pin map: HUZZAH32 Feather
// ===============================
static const int PIN_SPI_SCK  = 5;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 18;

static const int PIN_MPU_CS   = 33;
static const int PIN_BMP_CS   = 32;

// ===============================
// Sensor objects
// ===============================
MPU9250 imu(PIN_MPU_CS, SPI);
Adafruit_BMP280 bmp(PIN_BMP_CS, &SPI);

// ===============================
// Latest sensor values
// ===============================
MPU_SensorData imuData;

float bmpTemp_C = 0.0f;
float bmpPressure_hPa = 0.0f;
float bmpAltitude_m = 0.0f;

// Sea-level pressure for altitude estimate.
// Change this to your local QNH later for better altitude.
static const float SEA_LEVEL_HPA = 1013.25f;

// ===============================
// Timing
// ===============================
uint32_t lastBmpUs = 0;
uint32_t lastPrintUs = 0;
uint32_t lastRateUs = 0;

uint32_t imuReadCount = 0;
uint32_t bmpReadCount = 0;

float imuHz = 0.0f;
float bmpHz = 0.0f;

// BMP read interval.
// 20000 us = 50 Hz.
// Do not try to read BMP280 at IMU speed.
static const uint32_t BMP_PERIOD_US = 20000;

// Print interval.
// Serial printing is slow, so keep this low.
static const uint32_t PRINT_PERIOD_US = 100000; // 10 Hz print

void setup()
{
    Serial.begin(921600);
    delay(1000);

    Serial.println();
    Serial.println("====================================");
    Serial.println("GY-91 High-Speed All Sensor Reader");
    Serial.println("HUZZAH32 / ESP32");
    Serial.println("====================================");

    // Keep both chip-selects high before SPI starts
    pinMode(PIN_MPU_CS, OUTPUT);
    pinMode(PIN_BMP_CS, OUTPUT);
    digitalWrite(PIN_MPU_CS, HIGH);
    digitalWrite(PIN_BMP_CS, HIGH);

    // Start VSPI with HUZZAH32 pin map
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // -------------------------------
    // Start MPU-9250
    // -------------------------------
    Serial.println("[BOOT] Starting MPU-9250...");
    if (!imu.begin()) {
        Serial.println("[ERROR] MPU-9250 not detected. Check wiring and CS pin.");
        while (true) {
            delay(500);
        }
    }

    Serial.println("[OK] MPU-9250 detected.");

    // Optional: load saved calibration if you already have one
    if (imu.loadCalibration()) {
        Serial.println("[OK] MPU calibration loaded from NVS.");
    } else {
        Serial.println("[INFO] No saved MPU calibration found. Using default offsets.");
    }

    // -------------------------------
    // Start BMP280 on SPI
    // -------------------------------
    Serial.println("[BOOT] Starting BMP280...");
    if (!bmp.begin()) {
        Serial.println("[ERROR] BMP280 not detected. Check BMP CSB wiring.");
        while (true) {
            delay(500);
        }
    }

    Serial.println("[OK] BMP280 detected.");

    // Fast BMP280 configuration
    // This favors speed over ultra-smooth pressure.
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X1,   // temperature oversampling
        Adafruit_BMP280::SAMPLING_X1,   // pressure oversampling
        Adafruit_BMP280::FILTER_OFF,
        Adafruit_BMP280::STANDBY_MS_1
    );

    lastRateUs = micros();

    Serial.println("[BOOT] Sensor read loop starting...");
    Serial.println();
}

void loop()
{
    uint32_t nowUs = micros();

    // =========================================================
    // 1. Read MPU-9250 as fast as possible
    //    This reads:
    //    ax, ay, az
    //    gx, gy, gz
    //    mx, my, mz
    //    temp
    // =========================================================
    if (imu.readScaled(imuData)) {
        imuReadCount++;
    }

    // =========================================================
    // 2. Read BMP280 at slower fixed rate
    // =========================================================
    if ((uint32_t)(nowUs - lastBmpUs) >= BMP_PERIOD_US) {
        lastBmpUs = nowUs;

        bmpTemp_C = bmp.readTemperature();
        bmpPressure_hPa = bmp.readPressure() / 100.0f;
        bmpAltitude_m = bmp.readAltitude(SEA_LEVEL_HPA);

        bmpReadCount++;
    }

    // =========================================================
    // 3. Calculate measured read rates once per second
    // =========================================================
    if ((uint32_t)(nowUs - lastRateUs) >= 1000000UL) {
        float dt_s = (nowUs - lastRateUs) / 1000000.0f;

        imuHz = imuReadCount / dt_s;
        bmpHz = bmpReadCount / dt_s;

        imuReadCount = 0;
        bmpReadCount = 0;
        lastRateUs = nowUs;
    }

    // =========================================================
    // 4. Print data slowly so Serial does not destroy performance
    // =========================================================
    if ((uint32_t)(nowUs - lastPrintUs) >= PRINT_PERIOD_US) {
        lastPrintUs = nowUs;

        Serial.printf(
            "IMU_Hz=%.1f BMP_Hz=%.1f | "
            "ACC[g] X=%+.3f Y=%+.3f Z=%+.3f | "
            "GYRO[dps] X=%+.2f Y=%+.2f Z=%+.2f | "
            "MAG[uT] X=%+.1f Y=%+.1f Z=%+.1f | "
            "MPU_TEMP=%.2fC | "
            "BMP_TEMP=%.2fC P=%.2fhPa ALT=%.2fm\n",

            imuHz,
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

            imuData.temp_c,

            bmpTemp_C,
            bmpPressure_hPa,
            bmpAltitude_m
        );
    }
}