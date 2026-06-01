# MPU Basics

**Author:** Durvesh Pathak  
**GitHub:** https://github.com/durveshpathak1989/MPU-basics  
**License:** MIT

Arduino library for MPU (Motion Processing Unit) sensors like MPU6050 and MPU9250.

## Features

- Read accelerometer data (X, Y, Z axes)
- Read gyroscope data (X, Y, Z axes)
- Read temperature sensor
- Simple and intuitive API
- Support for I2C communication

## Hardware Requirements

- Arduino board (Uno, Nano, Mega, Pro Mini, etc.)
- MPU6050 or MPU9250 sensor module
- I2C pull-up resistors (optional, usually built-in)

## Wiring

```
MPU Sensor → Arduino
VCC        → 5V
GND        → GND
SDA        → A4 (SDA pin)
SCL        → A5 (SCL pin)
```

## Installation

1. Download this library
2. Place it in your Arduino `libraries` folder
3. Restart Arduino IDE
4. Select the library from Sketch → Include Library

## Quick Start

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

## Examples

See the `examples` folder for complete sketches.

## License

MIT
