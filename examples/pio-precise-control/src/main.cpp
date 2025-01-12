/*

This simply turns the motors at random speeds using the drv8833 class.

Use the BOOTSEL button on the Pico to toggle the motors on and off.

*/

#include "DRV8833.h"
#include <Arduino.h>
#include <array>

constexpr auto pwmFreq = 50000; // Low frequency works well with smaller motors, leads to audible pwm noise. Above nyqist (22khz ish) is silent.
constexpr auto decayMode = motor::DecayMode::Slow;
auto motorsRunning = false;

std::array<motor::DRV8833, 2> motors{
    motor::DRV8833(2, 3, 19, 18, /* sleepPin */ 6, decayMode),
    motor::DRV8833(14, 15, 20, 21, /* sleepPin */ 7, decayMode)};

auto getRandomSpeed() {
  return static_cast<float>(random(100, 1000)) / 1000.0f;
}

auto randomPolarity(int probability) {
  if (random(0, 100) > probability) {
    return -1.f;
  } else {
    return 1.0f;
  }
};

void randomizeAllMotorSpeeds() {
  for (auto &motor : motors) {

    // Random speed
    float speed = getRandomSpeed();
    speed *= randomPolarity(50);

    // Set Bridge A
    motor.getBridgeA().setSpeedBipolar(speed);

    // Set Bridge B
    speed = getRandomSpeed();
    speed *= randomPolarity(50);

    motor.getBridgeB().setSpeedBipolar(speed);
  }
}

void setAllMotorsToSpeed(float speed) {
  for (auto &motor : motors) {
    motor.getBridgeA().setSpeedBipolar(speed);
    motor.getBridgeB().setSpeedBipolar(speed);
  }
}

void start() {
  for (auto &motor : motors) {
    motor.startAll();
  }

  motorsRunning = true;
}

void stop() {
  for (auto &motor : motors) {
    motor.stopAll();
  }

  motorsRunning = false;
}

auto bootselButtonState = false;
void updateBootSelButton() {
  if (bootselButtonState != BOOTSEL) {

    // If high, toggle motors on/off
    if (BOOTSEL == true) {
      Serial.println("Bootsel button pressed");
      if (motorsRunning) {
        stop();
      } else {
        start();
      }
    }

    bootselButtonState = BOOTSEL;
  }
}

void setup() {
  Serial.begin(115200);

  // Set PWM frequency
  analogWriteFreq(pwmFreq);

  setAllMotorsToSpeed(0.8f);

  for (auto &motor : motors) {
    motor.begin();
    motor.wake();
  }

  bootselButtonState = BOOTSEL;
}

void loop() {
  updateBootSelButton();
  if (motorsRunning) {
    // Stop motors
    for (auto &motor : motors) {
      if (motor.getBridgeA().isRunning()) {
        motor.getBridgeA().stop();
      }
      if (motor.getBridgeB().isRunning()) {
        motor.getBridgeB().stop();
      }
    }

    // Randomize speeds
    randomizeAllMotorSpeeds();

    // Start motors again
    for (auto &motor : motors) {
      motor.getBridgeA().start();
      motor.getBridgeB().start();
    }

    const auto waitTime = random(100, 1000);
    delay(waitTime);
  }
}
