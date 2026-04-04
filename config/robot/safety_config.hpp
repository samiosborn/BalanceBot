// config/robot/safety_config.hpp
#pragma once

#include <cstdint>

namespace balancebot::config {

struct SafetyConfig {
    // Maximum allowed pitch angle (rad)
    static constexpr float max_safe_pitch_rad = 1;

    // Command timeout (ms)
    static constexpr std::uint32_t command_timeout_ms = 250;

    // Sensor timeout (ms)
    static constexpr std::uint32_t sensor_timeout_ms = 100;

    // Should robot begin armed on boot? 
    static constexpr bool boot_armed = false;

    // Should motors be enabled on boot? 
    static constexpr bool boot_motors_enabled = false;
};

}  // namespace balancebot::config