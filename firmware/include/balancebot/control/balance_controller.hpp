// firmware/include/balancebot/control/balance_controller.hpp
#pragma once

#include "balancebot/core/types.hpp"
#include "balancebot/estimation/estimation_types.hpp"

namespace balancebot {

// Forward declaration
class PidController;

class BalanceController {
public:
    // Constructor
    explicit BalanceController(PidController& pid_controller);

    // Reset controller state
    void reset();

    // Compute one balancing motor command
    MotorCommand update(const AttitudeState& attitude, float dt_s);

private:
    // Apply the config pitch angle offset
    float apply_angle_offset_(float pitch_rad) const;

    // Apply deadband to the output duty
    float apply_deadband_(float duty) const;

    // Reference to the underlying PID controller
    PidController& pid_controller_;

    // Configured balance deadband
    const float deadband_;

    // Configured balance angle offset
    const float angle_offset_rad_;
};

}  // namespace balancebot