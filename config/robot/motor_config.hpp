// config/robot/motor_config.hpp
#pragma once

#include <cstdint>

namespace balancebot::config {

struct MotorConfig {
    // I2C address (SparkFun Qwiic Motor Driver)
    static constexpr std::uint8_t i2c_address = 0x5D;

    // Motor IDs on the driver
    static constexpr std::uint8_t left_motor_id = 0;
    static constexpr std::uint8_t right_motor_id = 1;

    // Maximum normalised command
    static constexpr float max_duty = 1.0f;

    // Driver hardware command range
    static constexpr std::uint8_t max_hardware_level = 255;

    // Motor direction inversion flags
    static constexpr bool invert_left_motor = false;
    static constexpr bool invert_right_motor = false;

    // Deadband compensation
    static constexpr float command_deadband = 0.0f;
};

}  // namespace balancebot::config