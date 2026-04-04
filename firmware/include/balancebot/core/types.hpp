// firmware/include/balancebot/core/types.hpp
#pragma once

#include <cstdint>

namespace balancebot {

// Robot modes
enum class RobotMode : uint8_t {
    Boot = 0,
    Idle,
    Calibration,
    ImuTest,
    EncoderTest,
    MotorTest,
    Armed,
    Balancing,
    RemoteControl,
    Fault
};

// Fault codes
enum class FaultCode : uint8_t {
    None = 0,
    ImuNotReady,
    MotorDriverNotReady,
    EncoderNotReady,
    UnsafeTilt,
    CommandTimeout,
    SensorTimeout,
    InvalidState,
    InternalError
};

// Drive command
struct DriveCommand {
    // Desired forward chassis velocity (m/s)
    float desired_forward_velocity_m_s = 0.0f;
    // Desired turning rate (rad/s)
    float desired_turn_rate_rad_s = 0.0f;
    // Whether balancing is requested
    bool enable_balance = false;
    // Whether motor output is permitted at all
    bool enable_motors = false;
    // Timestamp of most recent command update
    uint32_t timestamp_us = 0;
};

// Motor command
struct MotorCommand {
    // Normalised duty commands in range [-1.0, +1.0]
    float left_duty = 0.0f;
    float right_duty = 0.0f;
    // Whether this command should be treated as valid
    bool valid = false;
};

// Loop timing statistics
struct LoopStats {
    // Loop count
    uint32_t loop_count = 0;
    // Time (us) since last loop
    uint32_t last_loop_dt_us = 0;
    // Max time (us) between loops
    uint32_t max_loop_dt_us = 0;
};

// Robot state
struct RobotState {
    RobotMode mode = RobotMode::Boot;
    FaultCode fault = FaultCode::None;

    bool armed = false;
    bool motors_enabled = false;
    bool balance_enabled = false;

    float pitch_rad = 0.0f;
    float pitch_rate_rad_s = 0.0f;
    float forward_velocity_m_s = 0.0f;
    float turn_rate_rad_s = 0.0f;
};

}  // namespace balancebot