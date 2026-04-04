// config/robot/imu_config.hpp
#pragma once

#include <cstdint>

namespace balancebot::config {

struct ImuConfig {
    // I2C address
    static constexpr std::uint8_t i2c_address = 0x69;

    // Sample rate
    static constexpr int sample_rate_hz = 200;

    // Invert pitch?
    static constexpr bool invert_pitch = false;

};

}  // namespace balancebot::config