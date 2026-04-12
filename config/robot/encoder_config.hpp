// config/robot/encoder_config.hpp
#pragma once

#include <cstdint>

#include "balancebot/core/physics_constants.hpp"

namespace balancebot::config {

struct EncoderConfig {
    // --- Encoder resolution ---

    // Pulses per revolution per channel on the motor encoder
    static constexpr std::int32_t encoder_ppr_channel = 11;

    // Quadrature decoding factor
    // 4x = both channels, both edges
    static constexpr std::int32_t quadrature_factor = 4;

    // Raw counts per motor-shaft revolution
    static constexpr std::int32_t encoder_cpr_raw =
        encoder_ppr_channel * quadrature_factor;


    // --- Gearbox ---

    // Motor-shaft turns per wheel-shaft turn
    static constexpr float gear_ratio = 18.75f;


    // --- Derived wheel quantities ---

    // Counts per full wheel revolution
    static constexpr float wheel_cpr =
        static_cast<float>(encoder_cpr_raw) * gear_ratio;

    // Radians per encoder count at the wheel
    static constexpr float wheel_rad_per_count =
        (2.0f * PhysicsConstants::pi) / wheel_cpr;


    // --- Velocity estimation / safety ---

    // If no new encoder edge arrives for longer than this,
    // wheel velocity can be treated as zero
    static constexpr std::uint32_t zero_velocity_timeout_us = 100000U;  // 100 ms
};

}  // namespace balancebot::config