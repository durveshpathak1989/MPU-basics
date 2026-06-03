# MPU Basics - GY-91 High-Speed Sensor Reader

**Author:** Durvesh Pathak  
**Repository:** https://github.com/durveshpathak1989/MPU-basics  
**Target controller:** Adafruit HUZZAH32 Feather / ESP32-WROOM-32E  
**Target sensor module:** GY-91 = MPU-9250 + AK8963 magnetometer + BMP280 barometer  
**License:** MIT

This project is a simple high-speed sensor extraction project for the **GY-91** module using an **Adafruit HUZZAH32 Feather ESP32**. The goal is to verify the sensor interface, extract all useful sensor values, and measure practical communication/read rates before using the data in a flight-controller loop.

The project intentionally keeps the architecture simple:

- **MPU-9250 accelerometer + gyroscope:** high-speed SPI
- **AK8963 magnetometer:** accessed through the MPU-9250 internal I2C master
- **BMP280 barometer:** SPI, if the GY-91 breakout exposes BMP280 SPI pins correctly
- **Serial output:** throttled so printing does not destroy sensor-read performance

---

## 1. Hardware Used

### Controller

- Adafruit HUZZAH32 Feather
- ESP32-WROOM-32E
- 3.3 V logic
- Arduino IDE / ESP32 Arduino core

### Sensor Module

GY-91 module containing:

| Chip | Function | Data Extracted |
|---|---|---|
| MPU-9250 | 3-axis accelerometer + 3-axis gyroscope + temperature | `ax`, `ay`, `az`, `gx`, `gy`, `gz`, `imuTemp` |
| AK8963 | 3-axis magnetometer inside the MPU-9250 package | `mx`, `my`, `mz` |
| BMP280 | Barometric pressure + temperature | `pressure`, `altitude`, `bmpTemp` |

The GY-91 is often marketed as a **10-DOF sensor**:

```text
3 accel + 3 gyro + 3 magnetometer + 1 pressure/altitude = 10 DOF
```

---

## 2. Wiring - HUZZAH32 to GY-91

This project uses shared SPI lines and separate chip-select pins.

| GY-91 Pin | HUZZAH32 Pin | ESP32 GPIO | Purpose |
|---|---|---:|---|
| VCC / VIN | 3V | 3.3 V | Power |
| GND | GND | GND | Ground |
| SCL / SCK | SCK | GPIO 5 | SPI clock |
| SDA / SDI / MOSI | MO | GPIO 18 | SPI MOSI |
| SDO / AD0 / MISO | MI | GPIO 19 | SPI MISO |
| NCS | User-selected CS | GPIO 33 | MPU-9250 chip select |
| CSB | User-selected CS | GPIO 32 | BMP280 chip select |
| INT | User-selected input | GPIO 27 | Optional MPU data-ready interrupt |

Important notes:

- Use **3.3 V** for the sensor module when working with HUZZAH32.
- Do not power the sensor from 5 V unless the exact breakout board is confirmed to have proper regulator and level shifting.
- If the MPU works but BMP280 does not, your specific GY-91 board may expose BMP280 only through I2C or may require a different CSB wiring configuration.

---

## 3. Communication Configuration

### SPI Bus

The project uses ESP32 VSPI-style pins:

```cpp
SCK  = GPIO 5
MISO = GPIO 19
MOSI = GPIO 18
```

### MPU-9250 SPI Rate

The driver uses two SPI speeds:

| Operation | SPI Speed | Reason |
|---|---:|---|
| Register configuration writes | 1 MHz | Safer during setup |
| Sensor burst reads | up to 20 MHz | High-speed data extraction |

The MPU read path uses burst reads so accel, gyro, and temperature are read efficiently from one register block.

### Serial Rate

Recommended debug serial baud:

```cpp
Serial.begin(115200);
```

For faster logging, use:

```cpp
Serial.begin(921600);
```

If the Serial Monitor shows garbage characters, the most likely cause is a **baud-rate mismatch**. Match the Arduino Serial Monitor baud rate to the value used in `Serial.begin()`.

---

## 4. Sensor Performance Configuration

The high-performance configuration is intended for fast motion sensing and flight-control learning.

| Sensor | Configuration |
|---|---|
| Accelerometer range | ±16 g |
| Gyroscope range | ±2000 °/s |
| Gyro DLPF | Fast / low-latency setting |
| Accel DLPF | Fast / low-latency setting |
| Magnetometer mode | 16-bit continuous mode, approximately 100 Hz |
| BMP280 sampling | Normal mode, low oversampling for speed |

Reason for these choices:

- ±16 g gives enough acceleration headroom for vibration and flight dynamics.
- ±2000 °/s prevents gyro saturation during quick rotations.
- Fast DLPF settings reduce sensor delay.
- BMP280 is intentionally kept slower because pressure sensing is not useful at IMU loop speed.

---

## 5. Expected Communication / Read Rates

The system does **not** read all sensors at the same rate. That is intentional.

| Data Source | Practical Rate | Why |
|---|---:|---|
| MPU-9250 accel + gyro | 500-1000 Hz target | Fast sensor path for control loop |
| MPU temperature | Same as accel/gyro burst | Included in 14-byte MPU burst read |
| AK8963 magnetometer | Up to ~100 Hz | Magnetometer hardware update-rate limit |
| BMP280 pressure/temp | 25-50 Hz recommended | Barometer is slower and noisier |
| Serial print | 4-10 Hz recommended | Serial printing is slow and should not block sensor reads |

Recommended architecture for future flight-control work:

```text
Fast loop:       accel + gyro       500-1000 Hz
Medium loop:     attitude filter    250-500 Hz
Slow loop:       magnetometer       50-100 Hz
Slow loop:       BMP280             25-50 Hz
Debug telemetry: Serial/Wi-Fi       4-10 Hz
```

The code tracks measured read rates such as:

```text
IMU_Hz=...
BMP_Hz=...
```

These values are more useful than theoretical rates because they include actual ESP32 execution time, SPI transfer time, BMP280 read cost, and Serial print overhead.

---

## 6. WHO_AM_I Values

During startup, the code reads the MPU identity register.

Common values:

| WHO_AM_I | Meaning |
|---:|---|
| `0x71` | Typical MPU-9250 response |
| `0x70` | MPU-6500 / compatible variant response; accel and gyro should still work |

If the board returns:

```text
WHO_AM_I = 0x70
```

that means SPI communication is working, but the module may not identify as a standard MPU-9250. In that case:

- Accelerometer should work.
- Gyroscope should work.
- Magnetometer may or may not be available depending on the actual chip/module variant.

For basic flight stabilization, accel + gyro are the critical sensors. The magnetometer is mostly useful for yaw/heading correction and is more sensitive to magnetic interference from motors, ESCs, and battery wiring.

---

## 7. Output Data

The sketch extracts and prints the following values:

| Field | Unit | Meaning |
|---|---|---|
| `ax_g` | g | X-axis acceleration |
| `ay_g` | g | Y-axis acceleration |
| `az_g` | g | Z-axis acceleration |
| `gx_dps` | degrees/sec | X-axis angular rate |
| `gy_dps` | degrees/sec | Y-axis angular rate |
| `gz_dps` | degrees/sec | Z-axis angular rate |
| `mx_uT` | microtesla | X-axis magnetic field |
| `my_uT` | microtesla | Y-axis magnetic field |
| `mz_uT` | microtesla | Z-axis magnetic field |
| `imuTemp_C` | °C | MPU internal die temperature |
| `bmpTemp_C` | °C | BMP280 temperature |
| `pressure_hPa` | hPa | Barometric pressure |
| `altitude_m` | meters | Estimated altitude from pressure |

Typical stationary flat-board behavior:

```text
ax ≈ 0 g
ay ≈ 0 g
az ≈ +1 g or -1 g depending on mounting orientation
gx ≈ 0 °/s
gy ≈ 0 °/s
gz ≈ 0 °/s
```

---

## 8. Troubleshooting

### Serial Monitor Shows Garbage Characters

Most likely cause: baud-rate mismatch.

Check the code:

```cpp
Serial.begin(115200);
```

or:

```cpp
Serial.begin(921600);
```

Then set the Arduino Serial Monitor to the exact same baud rate.

ESP32 may also print a few bootloader characters during reset. That is normal. The sketch output should become readable after startup.

### WHO_AM_I Returns `0x70`

This is not a wiring failure. It means the chip is responding. Continue testing accel and gyro first.

### WHO_AM_I Returns `0x00` or `0xFF`

Likely causes:

- Wrong chip-select pin
- MISO/MOSI swapped
- Bad ground connection
- Sensor not powered
- SPI mode or wiring issue

### BMP280 Not Detected

Possible causes:

- BMP280 CSB pin is not connected correctly
- The GY-91 breakout exposes BMP280 through I2C only
- BMP280 address or interface mode differs on the specific module

In that case, keep the MPU-9250 on SPI and switch only the BMP280 to I2C.

### Magnetometer Shows Zero

Possible causes:

- Actual module behaves like MPU-6500, not full MPU-9250
- AK8963 is not accessible through the MPU internal I2C master
- Magnetometer mode was not configured correctly
- Magnetic overflow or poor module variant

For first flight stabilization, continue with accel + gyro first.

---

## 9. Development Philosophy

This repository is not only for reading numbers from a sensor. It is the first interface-verification step for a future quadcopter flight controller.

Before trusting control logic, verify:

1. Sensor identity using `WHO_AM_I`
2. Correct wiring and voltage levels
3. Correct units and scaling
4. Correct axis signs
5. Correct sample rate
6. Correct loop timing
7. Correct handling of slow sensors versus fast sensors

A good embedded/control system does not just ask, "Does the code compile?" It asks, "Can I trust this data contract before I close the control loop?"
