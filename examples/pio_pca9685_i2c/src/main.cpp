/*
 * This example shows how to set up a an array of DRV8833 motor drivers that are
 * connected to a PCA9685 PWM driver via I2C.
 *
 * It randommizes the speed of each motor every second.
 *
 *
 */
#include "DRV8833.h"
#include "PCA9685.h"
#include <Arduino.h>
#include <Wire.h>
#include <array>

constexpr auto pwmFreq = 25; // Low frequency works well with smaller motors
constexpr auto decayMode = motor::DecayMode::Fast;
constexpr auto i2c_sda = 8;
constexpr auto i2c_scl = 9;

constexpr auto pca9685Address = 0x6C;
// constexpr auto pca9685Address = 0x40;

std::shared_ptr<PCA9685> motorArray = std::make_shared<PCA9685>(pca9685Address);

// NOTE: Sleep pins not used in this example
std::array<motor::DRV8833, 4> motors{
    motor::DRV8833(motorArray, 0, 1, 2, 3, /* sleepPin */ 10, decayMode),
    motor::DRV8833(motorArray, 4, 5, 6, 7, /* sleepPin */ 11, decayMode),
    motor::DRV8833(motorArray, 8, 9, 10, 11, /* sleepPin */ 12, decayMode),
    motor::DRV8833(motorArray, 12, 13, 14, 15, /* sleepPin */ 13, decayMode)};

void randomizeAllMotorSpeeds() {
  for (auto &motor : motors) {

    // Random speed
    float randomSpeed = static_cast<float>(random(0, 400)) / 1000.0f;

    // Randomly reverse the direction
    if (random(0, 100) > 50) {
      randomSpeed *= -1.f;
    }

    motor.getBridgeA().setSpeedBipolar(randomSpeed);
    motor.getBridgeB().setSpeedBipolar(randomSpeed);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("PCA9685_LIB_VERSION: ");
  Serial.println(PCA9685_LIB_VERSION);
  Serial.println();

  Wire.setSDA(i2c_sda);
  Wire.setSCL(i2c_scl);
  Wire.begin();

  motorArray->begin();
  motorArray->setOutputEnable(true);
  motorArray->setFrequency(pwmFreq);

  for (auto &motor : motors) {
    motor.begin();
  }

  randomizeAllMotorSpeeds();
}

void loop() {
  randomizeAllMotorSpeeds();
  delay(1000);
}
