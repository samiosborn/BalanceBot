// firmware/include/balancebot/control/pid_controller.hpp
#pragma once

namespace balancebot {

class PidController {
public:
    // Constructor
    PidController();

    // Reset controller state
    void reset();

    // Update the PID output
    float update(float error, float error_rate, float dt_s);

private:
    // Clamp a value to [min_value, max_value]
    float clamp_(float value, float min_value, float max_value) const;

    // --- Gains ---
    float kp_;
    float ki_;
    float kd_;

    // --- Integral state ---
    float integral_;

    // --- Integral limits ---
    float integral_min_;
    float integral_max_;

    // --- Output limits ---
    float output_min_;
    float output_max_;
};

}  // namespace balancebot