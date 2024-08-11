[![PlatformIO Registry](https://badges.registry.platformio.org/packages/madskjeldgaard/library/ArduinoDRV8833.svg)](https://registry.platformio.org/libraries/madskjeldgaard/ArduinoDRV8833)

# DRV8833 Arduino Library

This library provides an interface for controlling the DRV8833 dual H-bridge motor driver chip using the Arduino framework. Each DRV8833 chip can drive two DC motors or one stepper motor. Albeit this currently is mostly written with DC motors in mind.

This library additionally supports using the motor driver in a popular setup where it is connected to a [PCA9685](https://github.com/RobTillaart/PCA9685_RT) i2c chip to allow controlling it via i2c.

Works well with something like [this Adafruit board](https://www.adafruit.com/product/3297) but should also just work with any DRV8833 setup. 

## Features

- [Platformio library](https://registry.platformio.org/libraries/madskjeldgaard/ArduinoDRV8833)
- Set the speed and direction of each motor either using integers or floats.
- Set the decay mode of each H-bridge (fast or slow) to improve performance of brushed DC motors.
- Put the DRV8833 to sleep to save power.
- Supports chips connected via i2c (using the PCA9685 chip, like in [this board for example](https://kitronik.co.uk/products/5329-kitronik-compact-robotics-board-for-raspberry-pi-pico))
- Wake the DRV8833 from sleep.

## Usage

Include the library in your Arduino sketch:

```cpp
#include "DRV8833.h"
```

Create an instance of the `DRV8833` class:

```cpp
motor::DRV8833 motorDriver(in1, in2, in3, in4, sleep);

void setup() {
    motorDriver.begin();
}
```

Control a motor:

```cpp
// 25% speed, forward
motorDriver.getBridgeA().setSpeed(0.25f, motor::Direction::Forward);

motorDriver.getBridgeA().start();

// 13% speed, backwards
motorDriver.getBridgeA().setSpeed(0.13f, motor::Direction::Backward);

```

Put the DRV8833 to sleep:

```cpp
motorDriver.sleep();
```

Wake the DRV8833 from sleep:

```cpp
motorDriver.wake();
```

## Decay Modes

The decay mode affects the performance of the motor. Fast decay mode causes a rapid reduction in inductive current and allows the motor to coast toward zero velocity. Slow decay mode leads to a slower reduction in inductive current but produces rapid deceleration.

[This Adafruit article has some nice info on performance and DC motors which explains the topic in depth](https://learn.adafruit.com/improve-brushed-dc-motor-performance/overview).

It is also recommended to mess with lowering the PWM frequency of the board to improve performance:

```cpp
constexpr auto PWM_FREQ = 100; // hz
analogWriteFreq(PWM_FREQ);
```

## Installation

### Platformio

Add the library to your lib_deps in `platformio.ini`

```
lib_deps = 
    # ... other libraries...

    # DRV8833 motor controller
    https://github.com/madskjeldgaard/arduino-drv8833
```

### Arduino

Download the library and add it to your Arduino libraries folder.

You also need to manually download and install the [PCA9685_RT](https://github.com/RobTillaart/PCA9685_RT) library.

## Contributing

All contributions are welcome. Please open an issue or a pull request if you have ideas to change this library for the better :)

## License

This library is released under the MIT license.
