// config/robot/control_config.hpp
#pragma once

namespace balancebot::config {

struct ControlConfig {
    // --- Loop timing ---

    // Fixed control timestep used by the current robot update path
    static constexpr float control_dt_s = 0.005f;


    // --- Complementary filter ---

    // Gyro / accelerometer fusion coefficient
    static constexpr float complementary_alpha = 0.975f;


    // --- Balance PID ---

    // Proportional gain; positive, with error = reference - pitch
    static constexpr float balance_kp = 9.0f;

    // Integral gain; positive, with error = reference - pitch
    static constexpr float balance_ki = 0.0f;

    // Derivative gain; positive, applied to de/dt = -pitch_rate for a fixed reference
    static constexpr float balance_kd = 1000.0f;


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

    // Small output deadband around zero
    static constexpr float balance_deadband = 0.01f;

    // Fixed pitch reference offset. A pitch equal to this value has zero error.
    static constexpr float balance_angle_offset_rad = 0.0f;
};

}  // namespace balancebot::config
