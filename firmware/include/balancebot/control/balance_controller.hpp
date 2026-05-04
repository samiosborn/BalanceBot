// firmware/include/balancebot/control/balance_controller.hpp
#pragma once

#include "balancebot/control/control_debug.hpp"
#include "balancebot/core/types.hpp"
#include "balancebot/drivers/interfaces/encoder_interface.hpp"
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
    MotorCommand update(const AttitudeState& attitude, const EncoderSample& encoder_sample, float dt_s);

    // Reveal latest temporary debug telemetry
    const BalanceDebugState& debug_state() const;

private:
    // Apply the config pitch angle offset
    float apply_angle_offset_(float pitch_rad) const;

    // Apply deadband to the output duty
    float apply_deadband_(float duty) const;

    // Clamp the final balance output duty
    float clamp_output_(float duty) const;

    // Reference to the underlying PID controller
    PidController& pid_controller_;

    // Configured balance deadband
    const float deadband_;

    // Configured balance angle offset
    const float angle_offset_rad_;

    // Configured wheel-speed damping gain
    const float wheel_speed_damping_k_;

    // Configured output limits
    const float output_min_;
    const float output_max_;

    // Temporary debug telemetry
    BalanceDebugState debug_state_;
};

}  // namespace balancebot
