// config/robot/control_config.hpp
#pragma once

namespace balancebot::config {

struct ControlConfig {
    // Main control loop rate
    static constexpr int control_loop_hz = 200;

    // Loop period (s)
    static constexpr float control_dt_s = 1.0f / static_cast<float>(control_loop_hz);

    // Telemetry update rate
    static constexpr int telemetry_hz = 25;

    // Complementary filter alpha
    static constexpr float complementary_alpha = 0.975f;

    // Balance controller deadband 
    static constexpr float balance_deadband = 0.05f;

    // Balance controller angle offset
    static constexpr float balance_angle_offset_rad = 0.0f;

    // PID gains
    static constexpr float balance_kp = 1.0f;
    static constexpr float balance_ki = 0.05f;
    static constexpr float balance_kd = 0.01f;

    // Integral clamp
    static constexpr float integral_min = -0.5f;
    static constexpr float integral_max = 0.5f;

    // Output clamp
    static constexpr float output_min = -1.0f;
    static constexpr float output_max = 1.0f;
};

}  // namespace balancebot::config