// firmware/src/drivers/qwiic_motor_driver.cpp

#include "balancebot/drivers/hardware/qwiic_motor_driver.hpp"

#include <Arduino.h>
#include <Wire.h>

#include "config/robot/motor_config.hpp"

#include <SCMD.h>

namespace balancebot {

QwiicMotorDriver::QwiicMotorDriver()
    : ready_(false),
      i2c_address_(config::MotorConfig::i2c_address),
      left_motor_id_(config::MotorConfig::left_motor_id),
      right_motor_id_(config::MotorConfig::right_motor_id),
      max_duty_(config::MotorConfig::max_duty),
      max_hardware_level_(config::MotorConfig::max_hardware_level),
      invert_left_motor_(config::MotorConfig::invert_left_motor),
      invert_right_motor_(config::MotorConfig::invert_right_motor),
      command_deadband_(config::MotorConfig::command_deadband) {}


// Bring up the SCMD motor driver over I2C
bool QwiicMotorDriver::begin() {
    ready_ = false;

    // Initialise the underlying SCMD driver
    if (!scmd_.begin(Wire, i2c_address_)) {
        return false;
    }

    // Wait for firmware enumeration (ready state)
    const std::uint32_t start_ms = millis();
    while (!scmd_.ready()) {
        delay(10);

        // Avoid waiting forever during bring-up
        if (millis() - start_ms > 2000U) {
            return false;
        }
    }

    // Enable motor outputs
    scmd_.enable();

    // Stop motors on startup
    stop();

    ready_ = true;
    return true;
}

// Ready
bool QwiicMotorDriver::is_ready() const {
    return ready_;
}

// Write a motor command (to both motors)
bool QwiicMotorDriver::write(const MotorCommand& command) {
    if (!ready_) {
        return false;
    }

    if (!command.valid) {
        stop();
        return false;
    }

    return write(command.left_duty, command.right_duty);
}

// Write direct left/right duty commands
bool QwiicMotorDriver::write(float left_duty, float right_duty) {
    if (!ready_) {
        return false;
    }

    write_motor_(left_motor_id_, invert_left_motor_ ? -left_duty : left_duty);
    write_motor_(right_motor_id_, invert_right_motor_ ? -right_duty : right_duty);

    return true;
}

// Stop both motors immediately
void QwiicMotorDriver::stop() {
    if (!ready_) {
        return;
    }

    write_motor_(left_motor_id_, 0.0f);
    write_motor_(right_motor_id_, 0.0f);
}

// --- Write helpers ---

// Encode one (normalised) duty command into: direction bit, hardware level
void QwiicMotorDriver::encode_level_(
    float duty,
    std::uint8_t& direction,
    std::uint8_t& level
) const {
    // Apply configured clamp
    float clamped = clamp_duty_(duty);

    // Apply deadband
    if (clamped > -command_deadband_ && clamped < command_deadband_) {
        clamped = 0.0f;
    }

    // Zero-output case
    if (clamped == 0.0f) {
        direction = 1U;
        level = 0U;
        return;
    }

    // Encode output direction
    direction = (clamped >= 0.0f) ? 1U : 0U;

    // Scale magnitude to hardware range
    const float magnitude = (clamped >= 0.0f) ? clamped : -clamped;
    const float scaled = magnitude * static_cast<float>(max_hardware_level_);

    // Truncate toward zero
    level = static_cast<std::uint8_t>(scaled);
}

// Write one motor output
void QwiicMotorDriver::write_motor_(std::uint8_t motor_id, float duty) {
    std::uint8_t direction = 1U;
    std::uint8_t level = 0U;

    encode_level_(duty, direction, level);

    scmd_.setDrive(motor_id, direction, level);
}

// Clamp duty to limits
float QwiicMotorDriver::clamp_duty_(float duty) const {
    if (duty > max_duty_) {
        return max_duty_;
    }

    if (duty < -max_duty_) {
        return -max_duty_;
    }

    return duty;
}

}  // namespace balancebot