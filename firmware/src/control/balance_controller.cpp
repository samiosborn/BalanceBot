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
      angle_offset_rad_(config::ControlConfig::balance_angle_offset_rad),

      // Initialise temporary debug telemetry
      debug_state_{} {}


// Reset controller state
void BalanceController::reset() {
    // Reset underlying PID state
    pid_controller_.reset();

    // Clear temporary debug telemetry
    debug_state_ = BalanceDebugState{};
}


// Compute one motor command to balance
MotorCommand BalanceController::update(const AttitudeState& attitude, float dt_s) {
    // Start with a safe default command
    MotorCommand command{};

    // Start a fresh temporary debug snapshot for this control attempt
    debug_state_ = BalanceDebugState{};
    debug_state_.dt_s = dt_s;
    debug_state_.pitch_rad = attitude.pitch_rad;
    debug_state_.pitch_rate_rad_s = attitude.pitch_rate_rad_s;

    // Return safe command if attitude estimate isn't valid
    if (!attitude.valid) {
        return command;
    }

    // Apply pitch angle offset
    const float corrected_pitch_rad = apply_angle_offset_(attitude.pitch_rad);
    debug_state_.corrected_pitch_rad = corrected_pitch_rad;

    // Run PID on (corrected) pitch and pitch rate 
    const float duty_before_deadband = pid_controller_.update(
        corrected_pitch_rad,
        attitude.pitch_rate_rad_s,
        dt_s
    );

    // Apply output deadband
    const float duty_after_deadband = apply_deadband_(duty_before_deadband);

    // Left and right duty commands (same for pure balance)
    command.left_duty = duty_after_deadband;
    command.right_duty = duty_after_deadband;

    // Valid command
    command.valid = true;

    // Store temporary debug telemetry for the exact values computed above
    debug_state_.duty_before_deadband = duty_before_deadband;
    debug_state_.duty_after_deadband = duty_after_deadband;
    debug_state_.left_duty = command.left_duty;
    debug_state_.right_duty = command.right_duty;
    debug_state_.command_valid = command.valid;
    debug_state_.pid = pid_controller_.debug_state();

    return command;
}


// Reveal latest temporary debug telemetry
const BalanceDebugState& BalanceController::debug_state() const {
    return debug_state_;
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
