// firmware/src/control/pid_controller.cpp

#include "balancebot/control/pid_controller.hpp"
#include "config/robot/control_config.hpp"

namespace balancebot {

// Constructor
PidController::PidController()
    // --- Initialise from compile-time config ---
    : 
      // Initialise gains
      kp_(config::ControlConfig::balance_kp),
      ki_(config::ControlConfig::balance_ki),
      kd_(config::ControlConfig::balance_kd),

      // Initialise integral state
      integral_(0.0f),

      // Initialise integral limits
      integral_min_(config::ControlConfig::integral_min),
      integral_max_(config::ControlConfig::integral_max),

      // Initialise output limits
      output_min_(config::ControlConfig::output_min),
      output_max_(config::ControlConfig::output_max) {}


// Reset controller state
void PidController::reset() {
    // Clear accumulated integral term
    integral_ = 0.0f;
}


// Compute one PID output
float PidController::update(float error, float error_rate, float dt_s) {
    // Proportional term
    const float p = kp_ * error;

    // Integral (clamp)
    if (dt_s > 0.0f) {
        integral_ += error * dt_s;
        integral_ = clamp_(integral_, integral_min_, integral_max_);
    }
    // Integral term
    const float i = ki_ * integral_;

    // Derivative term
    const float d = -kd_ * error_rate;

    // Combine terms
    float output = p + i + d;

    // Clamp final output
    output = clamp_(output, output_min_, output_max_);

    return output;
}


// Clamp (helper)
float PidController::clamp_(float value, float min_value, float max_value) const {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

}  // namespace balancebot