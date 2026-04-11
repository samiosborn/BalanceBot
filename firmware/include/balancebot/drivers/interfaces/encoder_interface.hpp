// firmware/include/balancebot/drivers/interfaces/encoder_interface.hpp
#pragma once

#include <cstdint>

namespace balancebot {

// Encoder sample
struct EncoderSample {
    // Accumulated quadrature counts (since reset)
    std::int32_t left_counts = 0;
    std::int32_t right_counts = 0;

    // Wheel angular position in radians
    float left_wheel_rad = 0.0f;
    float right_wheel_rad = 0.0f;

    // Wheel angular velocity in rad/s
    float left_wheel_rad_s = 0.0f;
    float right_wheel_rad_s = 0.0f;

    // Wheel linear velocity in m/s
    float left_wheel_m_s = 0.0f;
    float right_wheel_m_s = 0.0f;

    // Timestamp when this state was assembled
    std::uint32_t timestamp_us = 0;

    // Whether this sample is valid
    bool valid = false;
};

// Interface for Encoder driver
class IEncoderDriver {
public:
    virtual ~IEncoderDriver() = default;

    // --- Bring-up ---
    // Initialise encoder GPIO / interrupts / counters
    virtual bool begin() = 0;

    // Whether the encoder subsystem is ready to use
    virtual bool is_ready() const = 0;


    // --- Read current encoder state ---
    // Returns true if a valid current sample was produced, and fills in 'sample'
    virtual bool read(EncoderSample& sample) = 0;


    // --- Reset state ---
    // Reset accumulated counts and if needed any derived velocity history
    virtual void reset() = 0;

};

}  // namespace balancebot
