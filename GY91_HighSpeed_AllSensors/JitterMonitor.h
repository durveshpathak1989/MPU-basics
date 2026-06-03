/*
  ============================================================
  JitterMonitor.h

  Small reusable timing/jitter helper for Arduino + ESP32 FreeRTOS.

  Intended usage inside a task:

      JitterMonitor imuJitter("IMU", 1000); // expected period = 1000 us

      void taskImu(void* parameter) {
          TickType_t lastWake = xTaskGetTickCount();
          while (true) {
              uint32_t t0 = imuJitter.start();

              // Do task work here

              imuJitter.end(t0);
              vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
          }
      }

  Intended usage inside a 1 Hz report task:

      imuJitter.updateReportAndReset();
      imuJitter.printReport(Serial);

  What it measures:
    - Start-to-start period timing
    - Absolute jitter from expected period
    - Execution time between start() and end()
    - Overrun count when execution time exceeds expected period

  This library does not create tasks. It only measures timing.
  ============================================================
*/

#pragma once
#ifndef JITTER_MONITOR_H
#define JITTER_MONITOR_H

#include <Arduino.h>

class JitterMonitor {
public:
    struct Snapshot {
        const char* name;
        uint32_t expectedPeriodUs;

        uint32_t samples;
        uint32_t minPeriodUs;
        uint32_t maxPeriodUs;
        float avgPeriodUs;

        uint32_t maxAbsJitterUs;
        float avgAbsJitterUs;

        uint32_t minExecUs;
        uint32_t maxExecUs;
        float avgExecUs;

        uint32_t overruns;
    };

    JitterMonitor();
    JitterMonitor(const char* name, uint32_t expectedPeriodUs);

    void begin(const char* name, uint32_t expectedPeriodUs);

    // Call at the start of the measured task section.
    // Returns a timestamp that should be passed to end().
    uint32_t start();

    // Call at the end of the measured task section.
    void end(uint32_t startUs);

    // Copies the current accumulation window into the report snapshot,
    // then clears the accumulation window for the next report interval.
    void updateReportAndReset();

    // Returns the last report snapshot.
    Snapshot getReport() const;

    // Convenience serial printer for the last report snapshot.
    void printReport(Stream& out) const;

    // Clear both live accumulation and report snapshot.
    void resetAll();

private:
    const char* _name;
    uint32_t _expectedPeriodUs;
    uint32_t _lastStartUs;

    // Live accumulation window
    uint32_t _samples;
    uint32_t _minPeriodUs;
    uint32_t _maxPeriodUs;
    uint32_t _maxAbsJitterUs;
    uint64_t _sumPeriodUs;
    uint64_t _sumAbsJitterUs;

    uint32_t _minExecUs;
    uint32_t _maxExecUs;
    uint64_t _sumExecUs;
    uint32_t _execSamples;
    uint32_t _overruns;

    Snapshot _report;

    // ESP32 critical-section lock. This keeps report reads safe while
    // a task is updating the monitor.
    mutable portMUX_TYPE _mux;

    void resetAccumNoLock();
    void resetReportNoLock();
};

#endif // JITTER_MONITOR_H
