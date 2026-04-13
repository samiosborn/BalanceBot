// firmware/src/estimation/complementary_filter.cpp

#include "balancebot/estimation/complementary_filter.hpp"
#include "config/robot/control_config.hpp"

#include <cmath>

namespace balancebot {

// Constructor
ComplementaryFilter::ComplementaryFilter()
    // Initialise fusion coefficient from compile-time config
    : alpha_(config::ControlConfig::complementary_alpha),
      state_{},
      initialised_(false) {}


// Reset estimator state
void ComplementaryFilter::reset(float pitch0_rad) {
    // Reset estimated attitude
    state_.pitch_rad = pitch0_rad;
    state_.pitch_rate_rad_s = 0.0f;
    state_.roll_rad = 0.0f;
    state_.yaw_rad = 0.0f;
    state_.timestamp_us = 0;
    state_.valid = true;

    // Initialise filter
    initialised_ = true;
}


// Update the attitude state from one IMU sample
AttitudeState ComplementaryFilter::update(const ImuSample& imu, float dt_s) {
    // Return the last state if invalid IMU sample
    if (!imu.valid) {
        return state_;
    }

    // Compute pitch angle from IMU sample
    const float pitch_acc_rad = compute_accel_pitch_rad_(imu);

    // On first valid update, initialise pitch from the current accel pitch
    if (!initialised_) {
        state_.pitch_rad = pitch_acc_rad;
        state_.pitch_rate_rad_s = imu.gyro_pitch_rad_s;
        state_.roll_rad = 0.0f;
        state_.yaw_rad = 0.0f;
        state_.timestamp_us = imu.timestamp_us;
        state_.valid = true;

        initialised_ = true;
        return state_;
    }

    // Take pitch rate from measured gyro pitch rate
    const float pitch_rate_rad_s = imu.gyro_pitch_rad_s;

    // Predict pitch from gyro integration
    float pitch_gyro_rad = state_.pitch_rad;
    if (dt_s > 0.0f) {
        pitch_gyro_rad += pitch_rate_rad_s * dt_s;
    }

    // State pitch rate as fusion of accelerometer tilt and gyro integration
    state_.pitch_rad =
        alpha_ * pitch_gyro_rad +
        (1.0f - alpha_) * pitch_acc_rad;

    // State pitch rate directly from gyro pitch rate
    state_.pitch_rate_rad_s = pitch_rate_rad_s;

    // State roll (not estimated yet)
    state_.roll_rad = 0.0f;

    // State yaw (not estimated yet)
    state_.yaw_rad = 0.0f;

    // Update time
    state_.timestamp_us = imu.timestamp_us;
    
    // Confirm valid
    state_.valid = true;

    return state_;
}


// Expose the current attitude estimate
const AttitudeState& ComplementaryFilter::state() const {
    return state_;
}


// Compute pitch angle from IMU sample
float ComplementaryFilter::compute_accel_pitch_rad_(const ImuSample& imu) const {
    // Angle between forward acceleration and upwards acceleration
    return std::atan2(-imu.accel_forward_m_s2, imu.accel_up_m_s2);
}

}  // namespace balancebot