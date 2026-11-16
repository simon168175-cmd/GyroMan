// GyroMan hardware adapter for the wiring already built on the real robot.
#pragma once

#include <Arduino.h>

namespace hw {

constexpr uint8_t IMU_SDA = 21;
constexpr uint8_t IMU_SCL = 22;
constexpr uint8_t IMU_ADDRESS = 0x68;

constexpr uint8_t LEFT_MOTOR_IN1 = 25;
constexpr uint8_t LEFT_MOTOR_IN2 = 26;
constexpr uint8_t RIGHT_MOTOR_IN1 = 27;
constexpr uint8_t RIGHT_MOTOR_IN2 = 14;

constexpr uint8_t LEFT_ENCODER_A = 32;
constexpr uint8_t LEFT_ENCODER_B = 33;
constexpr uint8_t RIGHT_ENCODER_A = 18;
constexpr uint8_t RIGHT_ENCODER_B = 19;

// Verified on the assembled robot: positive command means wheel-forward.
constexpr bool LEFT_MOTOR_REVERSED = true;
constexpr bool RIGHT_MOTOR_REVERSED = false;
constexpr bool LEFT_ENCODER_REVERSED = false;
constexpr bool RIGHT_ENCODER_REVERSED = true;

constexpr uint32_t SERIAL_BAUD = 230400;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;

// JGA25-370 12 V 280 RPM Hall encoder estimate:
// 11 motor pulses * ~21.3 gearbox * 4 quadrature edges.
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 936.0f;

}  // namespace hw
