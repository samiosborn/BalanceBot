// config/robot/imu_config.hpp
#pragma once

#include <cstdint>

namespace balancebot::config {

struct ImuConfig {
    // --- Bus / device ---

    // I2C address
    static constexpr std::uint8_t i2c_address = 0x69;

    // Sample rate
    static constexpr int sample_rate_hz = 200;


    // --- Orientation ---

    // Inverse pitch sign
    static constexpr bool invert_pitch = false;


    // --- Gyroscope configuration (GYRO_CONFIG_1) ---

    // Digital low-pass filter setting
    static constexpr std::uint8_t gyro_dlpf_cfg = 3;

    // Full-scale range selection
    static constexpr std::uint8_t gyro_fs_sel = 1;

    // DLPF enable flag
    static constexpr std::uint8_t gyro_fchoice = 1;


    // --- Accelerometer configuration (ACCEL_CONFIG) ---

    // Digital low-pass filter setting
    static constexpr std::uint8_t accel_dlpf_cfg = 3;

    // Full-scale range selection
    static constexpr std::uint8_t accel_fs_sel = 1;

    // DLPF enable flag
    static constexpr std::uint8_t accel_fchoice = 1;


    // --- Sensitivity ---

    // Accelerometer sensitivity (LSB per g)
    static constexpr float accel_sensitivity_lsb_per_g = 8192.0f;

    // Gyroscope sensitivity (LSB per dps)
    static constexpr float gyro_sensitivity_lsb_per_dps = 65.5f;


    // --- Power / bring-up behaviour ---

    // Value written to PWR_MGMT_1
    static constexpr std::uint8_t pwr_mgmt_1_value = 0x01;
};

}  // namespace balancebot::config