// firmware/include/balancebot/control/control_debug.hpp
#pragma once

namespace balancebot {

// Debug telemetry for the PID
struct PidDebugState {
    float dt_s = 0.0f;
    float error = 0.0f;
    float error_rate = 0.0f;
    float integral_state = 0.0f;
    float proportional_term = 0.0f;
    float integral_term = 0.0f;
    float derivative_term = 0.0f;
    float raw_output = 0.0f;
    float clamped_output = 0.0f;
};


// Debug telemetry for the balance controller
struct BalanceDebugState {
    float dt_s = 0.0f;
    float pitch_rad = 0.0f;
    float pitch_rate_rad_s = 0.0f;
    float corrected_pitch_rad = 0.0f;
    float average_wheel_rad_s = 0.0f;
    float average_forward_velocity_m_s = 0.0f;
    float wheel_speed_damping_term = 0.0f;
    float combined_raw_output = 0.0f;
    float combined_clamped_output = 0.0f;
    float duty_before_deadband = 0.0f;
    float duty_after_deadband = 0.0f;
    float left_duty = 0.0f;
    float right_duty = 0.0f;
    bool command_valid = false;
    PidDebugState pid{};
};

}  // namespace balancebot
