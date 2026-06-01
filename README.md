# MPU Basics - GY91 9-Axis IMU Library

**Author:** Durvesh Pathak  
**GitHub:** https://github.com/durveshpathak1989/MPU-basics  
**License:** MIT  
**Version:** 1.0.1

Complete Arduino library for MPU (Motion Processing Unit) sensors with full GY91 9-axis IMU module support.

## Supported Modules

- **MPU6050** - 6-axis IMU (accelerometer + gyroscope)
- **MPU9250** - 9-axis IMU (accelerometer + gyroscope + magnetometer)
- **GY91** - 9-axis IMU module with MPU9250 + AK8963 magnetometer

## GY91 Module Features

The GY91 is a popular 9-axis IMU module that combines:
- **MPU9250** - 3-axis accelerometer, 3-axis gyroscope, temperature sensor
- **AK8963** - 3-axis magnetometer (compass)
- Single I2C interface for easy connection

## Features

- Read accelerometer data (X, Y, Z axes) in g
- Read gyroscope data (X, Y, Z axes) in °/s
- Read temperature sensor data
- Read magnetometer/compass data (GY91)
- Simple and intuitive API
- Support for I2C communication
- Multiple sensor address support

## Hardware Requirements

- Arduino board (Uno, Nano, Mega, Pro Mini, etc.)
- MPU6050, MPU9250, or GY91 sensor module
- I2C pull-up resistors (optional, usually built-in)

## GY91 Wiring

```
GY91 Module → Arduino
VCC         → 5V
GND         → GND
SDA         → A4 (SDA pin) or SDA pin
SCL         → A5 (SCL pin) or SCL pin

I2C Addresses:
- MPU9250:  0x68 (default) or 0x69
- AK8963:   0x0C (fixed, controlled by MPU9250)
```

## Installation

1. Download this library
2. Place it in your Arduino `libraries` folder
3. Restart Arduino IDE
4. Select the library from Sketch → Include Library

## Quick Start - Basic MPU Reading

```cpp
#include <MPU.h>

MPU mpu;  // Default I2C address 0x68

void setup() {
  Serial.begin(9600);
  mpu.initialize();
}

void loop() {
  mpu.readAll();
  
  Serial.print("Accel X: ");
  Serial.println(mpu.getAccelX());
  
  delay(100);
}
```

## GY91 Usage

For GY91 modules with magnetometer support, refer to the examples folder for advanced usage including:
- Reading all 9 axes (accel, gyro, mag)
- Compass heading calculation
- Magnetometer calibration

## API Reference

### Constructor
- `MPU(uint8_t address = 0x68)` - Create MPU object with optional I2C address

### Methods
- `bool initialize()` - Initialize sensor communication
- `void readAccel()` - Read accelerometer values
- `void readGyro()` - Read gyroscope values
- `void readTemp()` - Read temperature
- `void readAll()` - Read all sensors at once

### Getters
- `float getAccelX()` - Get X-axis acceleration (g)
- `float getAccelY()` - Get Y-axis acceleration (g)
- `float getAccelZ()` - Get Z-axis acceleration (g)
- `float getGyroX()` - Get X-axis angular velocity (°/s)
- `float getGyroY()` - Get Y-axis angular velocity (°/s)
- `float getGyroZ()` - Get Z-axis angular velocity (°/s)
- `float getTemp()` - Get temperature (°C)

## Sensor Specifications

### MPU9250
- Accelerometer: ±2, ±4, ±8, ±16 g
- Gyroscope: ±250, ±500, ±1000, ±2000 °/s
- Temperature: -40 to +85 °C

### AK8963 (GY91 Magnetometer)
- Magnetic Field Range: ±4800 µT
- Resolution: 0.15 µT/LSB (16-bit mode)

## Examples

See the `examples` folder for complete sketches:
- BasicReading - Read accelerometer, gyroscope, temperature
- GY91_BasicReading - Read all 9 axes including magnetometer
- GY91_Calibration - Calibrate magnetometer for accurate readings
- GY91_Heading - Calculate compass heading from magnetometer

## Troubleshooting

**I2C Communication Issues:**
- Check SDA/SCL connections
- Verify pull-up resistors (4.7kΩ recommended)
- Ensure power supply is stable

**Magnetometer Not Working (GY91):**
- AK8963 requires calibration for accurate readings
- See GY91_Calibration example
- Keep away from magnetic interference

## License

MIT

## Support

For issues, documentation, and updates, visit:
https://github.com/durveshpathak1989/MPU-basics
