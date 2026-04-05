// firmware/include/balancebot/drivers/imu_interface.hpp
#pragma once

#include <cstdint>

namespace balancebot {

// IMU Sample
struct ImuSample {
    // Linear acceleration in m/s^2
    float accel_forward_m_s2 = 0.0f;
    float accel_left_m_s2 = 0.0f;
    float accel_up_m_s2 = 0.0f;

    // Angular velocity in rad/s
    float gyro_roll_rad_s = 0.0f;
    float gyro_pitch_rad_s = 0.0f;
    float gyro_yaw_rad_s = 0.0f;

    // Timestamp
    uint32_t timestamp_us = 0;

    // Whether this sample is valid
    bool valid = false;
};

// IMU calibration result
struct ImuCalibrationResult {
    bool success = false;
    std::uint32_t samples_used = 0;
};

// IMU driver interface
class IImuDriver {
public:
    virtual ~IImuDriver() = default;

    // --- Bring-up ---
    // Start communication
    virtual bool begin() = 0;

    // Whether the driver believes the IMU is usable
    virtual bool is_ready() const = 0;


    // --- Reading ---
    // On success, 'sample' should be filled in
    virtual bool read(ImuSample& sample) = 0;


    // --- Calibration ---
    // Run a stationary calibration routine
    virtual ImuCalibrationResult calibrate(std::uint32_t num_samples) = 0;

    // Reset any stored calibration
    virtual void reset() = 0;

};

}  // namespace balancebot