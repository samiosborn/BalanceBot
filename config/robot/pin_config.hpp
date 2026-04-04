// config/robot/pin_config.hpp
#pragma once

namespace balancebot::config {

struct PinConfig {
    // I2C bus pins
    static constexpr int i2c_sda = 21;
    static constexpr int i2c_scl = 22;

    // Left encoder pins
    static constexpr int left_encoder_a = 32;
    static constexpr int left_encoder_b = 33;

    // Right encoder pins
    static constexpr int right_encoder_a = 25;
    static constexpr int right_encoder_b = 26;

    // Status LED
    static constexpr int status_led = 2;
};

}  // namespace balancebot::config