// firmware/include/balancebot/estimation/estimation_types.hpp
#pragma once

#include <cstdint>

namespace balancebot {

// Attitude state
struct AttitudeState {
    // Pitch angle in radians
    float pitch_rad = 0.0f;

    // Pitch rate in rad/s
    float pitch_rate_rad_s = 0.0f;

    // Roll angle in radians
    float roll_rad = 0.0f;

    // Yaw angle in radians
    float yaw_rad = 0.0f;

    // Timestamp of this estimate
    std::uint32_t timestamp_us = 0;

    // Whether this estimate is currently valid
    bool valid = false;
};

// Motion state
struct MotionState {
    // Forward velocity in m/s
    float forward_velocity_m_s = 0.0f;

    // Turning rate in rad/s
    float turn_rate_rad_s = 0.0f;

    // Left wheel velocity in m/s
    float left_wheel_m_s = 0.0f;

    // Right wheel velocity in m/s
    float right_wheel_m_s = 0.0f;

    // Timestamp of this estimate
    std::uint32_t timestamp_us = 0;

    // Whether this estimate is currently valid
    bool valid = false;
};

}  // namespace balancebot