/*
 *
 * This file represents a DRV8833 motor driver chip.  The DRV8833 is a dual
 * H-bridge motor driver that can drive two DC motors or one stepper motor.
 *
 * See the datasheet here:
 * https://www.ti.com/lit/ds/symlink/drv8833.pdf
 *
 *
 */

#pragma once
#include <Arduino.h>

namespace motor {

// TODO: Make this consistent across boards
// The maximum PWM value of the current boards' PWM pins
// #if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32) || \
//      defined(ARDUINO_ARCH_SAM) ||              \
//     defined(ARDUINO_ARCH_SAMD)
// constexpr uint16_t MAX_PWM_VAL = 1023;
// // These are 255
// #elif defined(ARDUINO_ARCH_ESP8266)
// constexpr uint16_t MAX_PWM_VAL = 255;
// #else
// constexpr uint16_t MAX_PWM_VAL = 255;
// #endif
constexpr uint16_t MAX_PWM_VAL = 255;

/**
 * @brief The decay mode of the motor driver bridge.
 * @description The decay mode affects the performance of the motor. Fast decay
 * mode causes a rapid reduction in inductive current and allows the motor to
 * coast toward zero velocity. Slow decay mode leads to a slower reduction in
 * inductive current but produces rapid deceleration.
 *
 * Slow decay works well with DC motors.
 *
 * See
 * https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/251780/drv8833-pwm-control/881196#881196
 *
 */
enum class DecayMode {
  Slow = 0,
  Fast = 1,
};

/**
 * @brief The direction of the motor.
 */
enum class Direction {
  Forward,
  Backward,
};

/**
 * @class DRV8833_HBridge
 * @brief One bridge of a DRV8833 motor driver chip. This repsentation of a
 * bridge supports fast and slow decay modes.
 *
 */
class DRV8833_HBridge {
public:
  // Constructor
  // in1 and in2 are the pins connected to the motor
  // The default decay mode is slow
  /**
   * @brief Default constructor for the DRV8833_HBridge class. The default
   * decay mode is slow.
   *
   * @param in1 pin1 of the motor driver bridge
   * @param in2 pin2 of the motor driver bridge
   */
  DRV8833_HBridge(uint8_t in1, uint8_t in2) : mIn1(in1), mIn2(in2) {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    mDecayMode = DecayMode::Slow;
  }

  /**
   * @brief Constructor for the DRV8833_HBridge class.  The decay mode can be
   * specified.
   *
   * @param in1 pin1 of the motor driver bridge
   * @param in2 pin2 of the motor driver bridge
   * @param mode Decaymode of the motor driver bridge
   */
  DRV8833_HBridge(uint8_t in1, uint8_t in2, DecayMode mode)
      : mIn1(in1), mIn2(in2), mDecayMode(mode) {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
  }

  /**
   * @brief Set the decay mode of the motor driver bridge.
   *
   * @param mode The decay mode of the motor driver bridge.
   */
  void setDecayMode(DecayMode mode) { mDecayMode = mode; }

  void setSpeed(float speed, Direction dir) {
    const auto pwm_val = static_cast<int>(speed * MAX_PWM_VAL);

    setSpeed(pwm_val, dir);
  }

  /*
   * @brief Set the speed of the motor. This defaults to forward direction
   *
   * @param speed The speed of the motor.  The speed is a number between -1.0
   * and 1.0.
   *
   */
  void setSpeed(float speed) {
    auto direction = Direction::Forward;

    setSpeed(speed, direction);
  }

  /*
   * @brief Set the speed and direction of the motor bipolar mode
   * @param speed The speed of the motor.  The speed is a number between -1.0
   * and 1.0. Positive values are forward, negative values are backward, and 0
   * is stopped.
   *
   */
  void setSpeedBipolar(float speed) {
    Serial.println("SPEED IS: ");
    Serial.println(speed);

    if (speed > 0.f) {
      setSpeed(speed, Direction::Forward);
    } else if (speed < 0.f) {
      setSpeed(speed, Direction::Backward);
    } else {
      stop();
    }
  }

  /**
   * @brief Set speed. This version of the method assumes the motor is moving in
   * the forward direction.
   *
   * @param speed The speed of the motor. The speed is a number between 0 and
   * MAX_PWM_VAL (probably 255)
   */
  void setSpeed(int speed) { setSpeed(speed, Direction::Forward); }

  /**
   * @brief Set speed and direction of the motor
   *
   * @param speed The speed of the motor.
   * @param dir The direction of the motor.  Forward or Backwards.
   */
  void setSpeed(int motorSpeed, Direction dir) {

    auto speed = motorSpeed;

    switch (mDecayMode) {
    case DecayMode::Fast:

      switch (dir) {
      case Direction::Forward:
        analogWrite(mIn1, speed);
        digitalWrite(mIn2, 0);
        break;
      case Direction::Backward:
        digitalWrite(mIn1, 0);
        analogWrite(mIn2, speed);
        break;
      }
      break;
    case DecayMode::Slow:

      switch (dir) {
      case Direction::Forward:

        digitalWrite(mIn1, HIGH);
        analogWrite(mIn2, speed);
        break;
      case Direction::Backward:
        analogWrite(mIn1, speed);
        digitalWrite(mIn2, HIGH);
        break;
      }

      break;
    }
  }

  /**
   * @brief Set the speed and direction of the motor
   *
   * @param speed The speed of the motor, between -MAX_PWM_VAL(probably 255)
   * and MAX_PWM_VAL. Positive
   * values are forward, negative values are backward, and 0 is stopped.
   *
   */
  void setSpeedBipolar(int speed) {
    if (speed > 0) {
      setSpeed(static_cast<int>(speed), Direction::Forward);
    } else if (speed < 0) {
      setSpeed(static_cast<int>(speed), Direction::Backward);
    } else {
      stop();
    }
  }

  void stop() {
    digitalWrite(mIn1, LOW);
    digitalWrite(mIn2, LOW);
  }

private:
  uint8_t mIn1, mIn2;

  DecayMode mDecayMode;
};

/**
 * @class DRV8833
 * @brief Represents a DRV8833 motor driver with two H-bridges and a sleep pin.
 * Each H Bridge may be configured in a fast or slow decay mode.
 *
 * For more information about decay modes and their impact on performance, see
 * this article:
 * https://learn.adafruit.com/improve-brushed-dc-motor-performance/overview
 *
 */
/**
 * @class DRV8833
 * @brief A class to control a DRV8833 chip. It has two H-bridges and a sleep
 * pin to put the chip to sleep.
 *
 */
class DRV8833 {
public:
  /**
   * @brief Default constructor for the DRV8833 class.  The default decay mode
   * is slow.
   *
   * @param in1 Pin connected to AIN1 on the DRV8833
   * @param in2 Pin connected to AIN2 on the DRV8833
   * @param in3 Pin connected to BIN1 on the DRV8833
   * @param in4 Pin connected to BIN2 on the DRV8833
   * @param sleep Pin connected to SLEEP on the DRV8833
   */
  DRV8833(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, uint8_t sleep)
      : mBridgeA(in1, in2), mBridgeB(in3, in4), mSleep(sleep) {
    pinMode(sleep, OUTPUT);
    digitalWrite(sleep, HIGH);
  }

  /**
   * @brief Constructor for the DRV8833 class. The decay mode can be specified.
   *
   * @param in1 Pin connected to AIN1 on the DRV8833
   * @param in2 Pin connected to AIN2 on the DRV8833
   * @param in3 Pin connected to BIN1 on the DRV8833
   * @param in4 Pin connected to BIN2 on the DRV8833
   * @param sleep Pin connected to SLEEP on the DRV8833
   * @param mode Decay mode of the motor driver chip
   */
  DRV8833(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, uint8_t sleep,
          DecayMode mode)
      : mBridgeA(in1, in2, mode), mBridgeB(in3, in4, mode), mSleep(sleep) {
    pinMode(sleep, OUTPUT);
    digitalWrite(sleep, HIGH);
  }

  /**
   * @brief Stop all connected motors
   */
  void stopAll() {
    mBridgeA.stop();
    mBridgeB.stop();
  }

  /**
   * @brief Get the H-bridge object for the A side of the DRV8833
   *
   * @return DRV8833_HBridge&
   */
  auto &getBridgeA() { return mBridgeA; }

  /**
   * @brief Get the H-bridge object for the B side of the DRV8833
   *
   * @return DRV8833_HBridge&
   */
  auto &getBridgeB() { return mBridgeB; }

  /**
   * @brief Put the DRV8833 to sleep.
   */
  void sleep() { digitalWrite(mSleep, LOW); }

  /**
   * @brief Wake the DRV8833 from sleep.
   */
  void wake() { digitalWrite(mSleep, HIGH); }

private:
  DRV8833_HBridge mBridgeA, mBridgeB;
  uint8_t mSleep;
};

} // namespace motor
