// firmware/src/control/balance_controller.cpp

#include "balancebot/control/balance_controller.hpp"
#include "balancebot/control/pid_controller.hpp"
#include "config/robot/control_config.hpp"

namespace balancebot {

// Constructor
BalanceController::BalanceController(PidController& pid_controller)
    
    // Reference to PID controller
    : pid_controller_(pid_controller),

      // --- Initialise from compile-time config ---

      // Initialise deadband
      deadband_(config::ControlConfig::balance_deadband),
      
      // Initialise angle offset
      angle_offset_rad_(config::ControlConfig::balance_angle_offset_rad) {}


// Reset controller state
void BalanceController::reset() {
    // Reset underlying PID state
    pid_controller_.reset();
}


// Compute one motor command to balance
MotorCommand BalanceController::update(const AttitudeState& attitude, float dt_s) {
    // Start with a safe default command
    MotorCommand command{};

    // Return safe command if attitude estimate isn't valid
    if (!attitude.valid) {
        return command;
    }

    // Apply pitch angle offset
    const float corrected_pitch_rad = apply_angle_offset_(attitude.pitch_rad);

    // Run PID on (corrected) pitch and pitch rate 
    float duty = pid_controller_.update(
        corrected_pitch_rad,
        attitude.pitch_rate_rad_s,
        dt_s
    );

    // Apply output deadband
    duty = apply_deadband_(duty);

    // Left and right duty commands (same for pure balance)
    command.left_duty = duty;
    command.right_duty = duty;

    // Valid command
    command.valid = true;

    return command;
}


// Apply configured pitch angle offset (helper)
float BalanceController::apply_angle_offset_(float pitch_rad) const {
    return pitch_rad - angle_offset_rad_;
}


// Apply deadband to the output duty (helper)
float BalanceController::apply_deadband_(float duty) const {
    // Deadband
    if (duty > -deadband_ && duty < deadband_) {
        return 0.0f;
    }

    return duty;
}

}  // namespace balancebot