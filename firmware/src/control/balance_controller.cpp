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

      // Initialise wheel-speed damping
      wheel_speed_damping_k_(config::ControlConfig::balance_wheel_speed_k),

      // Initialise final output limits
      output_min_(config::ControlConfig::output_min),
      output_max_(config::ControlConfig::output_max),

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
MotorCommand BalanceController::update(
    const AttitudeState& attitude,
    const EncoderSample& encoder_sample,
    float dt_s
) {
    // Start with a safe default command
    MotorCommand command{};

    // Encoders use the same positive-forward convention for both wheels
    const float average_wheel_rad_s = encoder_sample.valid
        ? 0.5f * (encoder_sample.left_wheel_rad_s + encoder_sample.right_wheel_rad_s)
        : 0.0f;
    const float average_forward_velocity_m_s = encoder_sample.valid
        ? 0.5f * (encoder_sample.left_wheel_m_s + encoder_sample.right_wheel_m_s)
        : 0.0f;

    // Start a fresh temporary debug snapshot for this control attempt
    debug_state_ = BalanceDebugState{};
    debug_state_.dt_s = dt_s;
    debug_state_.pitch_rad = attitude.pitch_rad;
    debug_state_.pitch_rate_rad_s = attitude.pitch_rate_rad_s;
    debug_state_.average_wheel_rad_s = average_wheel_rad_s;
    debug_state_.average_forward_velocity_m_s = average_forward_velocity_m_s;

    // Return safe command if attitude estimate isn't valid
    if (!attitude.valid) {
        return command;
    }

    // Shift the measured pitch so that zero corresponds to the balance reference
    const float corrected_pitch_rad = apply_angle_offset_(attitude.pitch_rad);
    debug_state_.corrected_pitch_rad = corrected_pitch_rad;

    // Error is reference minus measured pitch
    const float pitch_error_rad = -corrected_pitch_rad;

    // Since the reference is constant, the error derivative is minus pitch rate
    const float pitch_error_rate_rad_s = -attitude.pitch_rate_rad_s;

    // Update the pitch/rate PID state, then combine using its unclamped pure PID output.
    pid_controller_.update(
        pitch_error_rad,
        pitch_error_rate_rad_s,
        dt_s
    );
    const PidDebugState& pid_debug = pid_controller_.debug_state();
    const float pitch_pid_output = pid_debug.raw_output;

    // Positive wheel speed means forward travel; this term commands reverse duty.
    const float wheel_speed_damping_term = (wheel_speed_damping_k_ == 0.0f)
        ? 0.0f
        : -wheel_speed_damping_k_ * average_wheel_rad_s;
    const float combined_raw_output = pitch_pid_output + wheel_speed_damping_term;
    const float combined_clamped_output = clamp_output_(combined_raw_output);

    // Apply output deadband
    const float duty_after_deadband = apply_deadband_(combined_clamped_output);

    // Left and right duty commands (same for pure balance)
    command.left_duty = duty_after_deadband;
    command.right_duty = duty_after_deadband;

    // Valid command
    command.valid = true;

    // Store temporary debug telemetry for the exact values computed above
    debug_state_.wheel_speed_damping_term = wheel_speed_damping_term;
    debug_state_.combined_raw_output = combined_raw_output;
    debug_state_.combined_clamped_output = combined_clamped_output;
    debug_state_.duty_before_deadband = combined_clamped_output;
    debug_state_.duty_after_deadband = duty_after_deadband;
    debug_state_.left_duty = command.left_duty;
    debug_state_.right_duty = command.right_duty;
    debug_state_.command_valid = command.valid;
    debug_state_.pid = pid_debug;

    return command;
}


// Reveal latest temporary debug telemetry
const BalanceDebugState& BalanceController::debug_state() const {
    return debug_state_;
}


// Apply configured pitch angle offset (helper). Zero means the pitch reference is met.
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


// Clamp final combined output to the configured duty limits
float BalanceController::clamp_output_(float duty) const {
    if (duty < output_min_) {
        return output_min_;
    }

    if (duty > output_max_) {
        return output_max_;
    }

    return duty;
}

}  // namespace balancebot
