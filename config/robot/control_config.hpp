// config/robot/control_config.hpp
#pragma once

namespace balancebot::config {

struct ControlConfig {
    // --- Loop timing ---

    // Fixed control timestep
    static constexpr float control_dt_s = 0.005f;


    // --- Complementary filter ---

    // Gyro / accelerometer fusion coefficient
    static constexpr float complementary_alpha = 0.975f;


    // --- Balance PID ---

    // Proportional gain
    static constexpr float balance_kp = 8.0f;

    // Integral gain
    static constexpr float balance_ki = 0.0f;

    // Derivative gain
    static constexpr float balance_kd = -0.15f;


    // --- Balance damping ---

    // Balance wheel-speed gain
    static constexpr float balance_wheel_speed_k = 0.0f;


    // --- PID clamps ---

    // Integral lower limit
    static constexpr float integral_min = -0.5f;

    // Integral upper limit
    static constexpr float integral_max = 0.5f;

    // Output lower limit
    static constexpr float output_min = -1.0f;

    // Output upper limit
    static constexpr float output_max = 1.0f;


    // --- Balance controller ---

    // Small output deadband
    static constexpr float balance_deadband = 0.005f;

    // Fixed pitch reference offset
    static constexpr float balance_angle_offset_rad = 0.0f;
};

}  // namespace balancebot::config
