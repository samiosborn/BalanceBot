// firmware/include/balancebot/drivers/hardware/icm20948.hpp
#pragma once

#include <cstdint>

#include "balancebot/drivers/interfaces/imu_interface.hpp"

namespace balancebot {

// IMU driver for the ICM-20948
class ICM20948 : public IImuDriver {
public:
    // Constructor
    ICM20948();

    // Virtual destructor
    ~ICM20948() override = default;

    // --- IImuDriver API ---

    // Bring up the IMU
    bool begin() override;

    // Whether the IMU driver is ready
    bool is_ready() const override;

    // Read one IMU sample
    bool read(ImuSample& sample) override;

    // Run stationary calibration
    ImuCalibrationResult calibrate(std::uint32_t num_samples) override;

    // Reset stored calibration / state
    void reset() override;

private:
    // --- Low-level register helpers ---

    // Write one register in the current bank
    void write_register_(std::uint8_t reg, std::uint8_t value);

    // Read one register in the current bank
    std::uint8_t read_register_(std::uint8_t reg) const;

    // Change active user bank
    void set_bank_(std::uint8_t bank);

    // Read one register from a specific bank
    std::uint8_t read_bank_register_(std::uint8_t bank, std::uint8_t reg);

    // Write one register in a specific bank
    void write_bank_register_(std::uint8_t bank, std::uint8_t reg, std::uint8_t value);


    // --- Device bring-up helpers ---

    // Wake the IMU from sleep
    void wake_();

    // Check WHO_AM_I
    bool check_identity_();

    // Configure gyroscope
    void configure_gyro_();

    // Configure accelerometer
    void configure_accel_();


    // --- Raw sensor read helpers ---

    // Read raw accelerometer counts
    void read_raw_accel_(std::int16_t& ax, std::int16_t& ay, std::int16_t& az);

    // Read raw gyroscope counts
    void read_raw_gyro_(std::int16_t& gx, std::int16_t& gy, std::int16_t& gz);

    // Combine two bytes into a signed 16-bit integer
    std::int16_t combine_bytes_(std::uint8_t high, std::uint8_t low) const;


    // --- Conversion / bias helpers ---

    // Convert raw accel counts to m/s^2
    float accel_counts_to_m_s2_(std::int16_t counts) const;

    // Convert raw gyro counts to rad/s
    float gyro_counts_to_rad_s_(std::int16_t counts) const;

    // Apply stored accel bias
    void apply_accel_bias_(float& ax, float& ay, float& az) const;

    // Apply stored gyro bias
    void apply_gyro_bias_(float& gx, float& gy, float& gz) const;

    // Fill a robot-frame IMU sample
    void fill_robot_frame_sample_(
        float ax,
        float ay,
        float az,
        float gx,
        float gy,
        float gz,
        std::uint32_t timestamp_us,
        ImuSample& sample
    ) const;


    // --- Driver state ---

    // Whether bring-up completed successfully
    bool ready_ = false;

    // Current selected user bank
    std::uint8_t current_bank_ = 0;

    // Stored accel bias (m/s^2)
    float accel_bias_x_m_s2_ = 0.0f;
    float accel_bias_y_m_s2_ = 0.0f;
    float accel_bias_z_m_s2_ = 0.0f;

    // Stored gyro bias (rad/s)
    float gyro_bias_x_rad_s_ = 0.0f;
    float gyro_bias_y_rad_s_ = 0.0f;
    float gyro_bias_z_rad_s_ = 0.0f;

    // Sensitivity constants derived from config
    const float accel_sensitivity_lsb_per_g_;
    const float gyro_sensitivity_lsb_per_dps_;
};

}  // namespace balancebot