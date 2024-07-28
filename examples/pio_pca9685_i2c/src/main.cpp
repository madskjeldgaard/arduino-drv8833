/*
 * This example shows how to set up a an array of DRV8833 motor drivers that are
 * connected to a PCA9685 PWM driver via I2C.
 *
 *
 */
#include "DRV8833.h"
#include "PCA9685.h"
#include <Arduino.h>
#include <Wire.h>
#include <array>

constexpr auto i2c_sda = 8;
constexpr auto i2c_scl = 9;

constexpr auto pca9685Address = 0x6C;
// constexpr auto pca9685Address = 0x40;

std::shared_ptr<PCA9685> motorArray = std::make_shared<PCA9685>(pca9685Address);

// NOTE: Sleep pins not used in this example
std::array<motor::DRV8833, 4> motors{
    motor::DRV8833(motorArray, 0, 1, 2, 3, /* sleepPin */ 10),
    motor::DRV8833(motorArray, 4, 5, 6, 7, /* sleepPin */ 11),
    motor::DRV8833(motorArray, 8, 9, 10, 11, /* sleepPin */ 12),
    motor::DRV8833(motorArray, 12, 13, 14, 15, /* sleepPin */ 13)};

void randomizeAllMotorSpeeds() {
  Serial.println("Randomizing motor speeds");
  for (auto &motor : motors) {
    float randomSpeed = static_cast<float>(random(350, 1000)) / 1000.0f;

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
  motorArray->setFrequency(25);

  // if (motorArray->isConnected() == false) {
  //   while (1) {
  //     Serial.println("PCA9685 not connected");
  //     delay(1000);
  //   };
  // }

  for (auto &motor : motors) {
    motor.begin();
    motor.wake();
    motor.getBridgeA().stop();
    motor.getBridgeB().stop();

    const auto initSpeed = 0.75f;
    const auto dir = motor::Direction::Forward;
    motor.getBridgeA().setSpeed(initSpeed, dir);
    motor.getBridgeB().setSpeed(initSpeed, dir);
  }

  randomizeAllMotorSpeeds();
}

void loop() {
  randomizeAllMotorSpeeds();
  delay(1000);
}
