// firmware/include/balancebot/estimation/complementary_filter.hpp
#pragma once

#include "balancebot/drivers/imu_interface.hpp"
#include "balancebot/estimation/estimation_types.hpp"

namespace balancebot {

class ComplementaryFilter {
public:
    // Constructor
    ComplementaryFilter();

    // Reset estimator state
    void reset(float pitch0_rad = 0.0f);

    // Update the attitude state from one IMU sample
    AttitudeState update(const ImuSample& imu, float dt_s);

    // Expose the current attitude estimate
    const AttitudeState& state() const;

private:
    // Compute pitch angle from IMU sample
    float compute_accel_pitch_rad_(const ImuSample& imu) const;

    // Complementary filter fusion coefficient
    const float alpha_;

    // Current estimated attitude state
    AttitudeState state_;

    // Whether the filter has been initialised yet
    bool initialised_ = false;
};

}  // namespace balancebot