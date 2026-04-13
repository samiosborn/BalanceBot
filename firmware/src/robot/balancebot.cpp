// firmware/src/robot/balancebot.cpp

#include "balancebot/robot/balancebot.hpp"

#include "balancebot/drivers/interfaces/imu_interface.hpp"
#include "balancebot/drivers/interfaces/encoder_interface.hpp"
#include "balancebot/drivers/interfaces/motor_driver_interface.hpp"

#include "balancebot/estimation/complementary_filter.hpp"
#include "balancebot/estimation/estimation_types.hpp"
#include "balancebot/control/balance_controller.hpp"

#include "config/robot/control_config.hpp"
#include "config/robot/safety_config.hpp"

#include <cmath>

namespace balancebot {

// Constructor
BalanceBot::BalanceBot(
    IImuDriver& imu_driver,
    IEncoderDriver& encoder_driver,
    IMotorDriver& motor_driver,
    ComplementaryFilter& attitude_estimator,
    BalanceController& balance_controller
)
    : imu_driver_(imu_driver),
      encoder_driver_(encoder_driver),
      motor_driver_(motor_driver),
      attitude_estimator_(attitude_estimator),
      balance_controller_(balance_controller),
      state_{},
      drive_command_{},
      motor_command_{},
      loop_stats_{},
      last_update_us_(0),
      first_update_(true) {}


// Begin robot bring-up
bool BalanceBot::begin() {
    // Start in a safe default state
    state_.mode = RobotMode::Boot;
    state_.fault = FaultCode::None;
    state_.armed = false;
    state_.motors_enabled = false;
    state_.balance_enabled = false;

    // Reset outputs and timing state
    drive_command_ = DriveCommand{};
    motor_command_ = MotorCommand{};
    loop_stats_ = LoopStats{};
    last_update_us_ = 0;
    first_update_ = true;

    // Bring up the IMU
    if (!imu_driver_.begin() || !imu_driver_.is_ready()) {
        enter_fault_state_(FaultCode::ImuNotReady);
        return false;
    }

    // Bring up the encoder subsystem
    if (!encoder_driver_.begin() || !encoder_driver_.is_ready()) {
        enter_fault_state_(FaultCode::EncoderNotReady);
        return false;
    }

    // Bring up the motor driver
    if (!motor_driver_.begin() || !motor_driver_.is_ready()) {
        enter_fault_state_(FaultCode::MotorDriverNotReady);
        return false;
    }

    // Reset estimator state
    attitude_estimator_.reset();

    // Reset controller state
    balance_controller_.reset();

    // Force motors off on startup
    force_motor_stop_();

    // Move into safe idle mode
    state_.mode = RobotMode::Idle;

    // No fault
    state_.fault = FaultCode::None;

    return true;
}


// Update step
void BalanceBot::update(std::uint32_t now_us) {
    // --- Loop timing ---
    // Dt from config
    float dt_s = config::ControlConfig::control_dt_s;
    // If first update, use the current time
    if (first_update_) {
        last_update_us_ = now_us;
        first_update_ = false;
    } else {
        // Calc dt (us) and update last update time
        const std::uint32_t dt_us = now_us - last_update_us_;
        last_update_us_ = now_us;
        // Update loop stats
        loop_stats_.last_loop_dt_us = dt_us;
        if (dt_us > loop_stats_.max_loop_dt_us) {
            loop_stats_.max_loop_dt_us = dt_us;
        }
        // Convert dt to seconds
        if (dt_us > 0U) {
            dt_s = static_cast<float>(dt_us) * 1.0e-6f;
        }
    }
    // Increment loop count
    ++loop_stats_.loop_count;

    // --- Read raw sensors ---
    read_sensors_();

    // --- Estimate state ---
    estimate_state_(dt_s);

    // --- Check safety ---
    check_safety_();

    // If fault, don't continue (force motors to stop)
    if (state_.mode == RobotMode::Fault) {
        force_motor_stop_();
        return;
    }

    // If not armed, keep motors off
    if (!state_.armed || !state_.motors_enabled || !state_.balance_enabled) {
        force_motor_stop_();
        return;
    }

    // --- Run control algorithm ---
    run_control_(dt_s);
}


// Arm the robot
void BalanceBot::arm() {
    // Clear controller state before a new balance attempt
    balance_controller_.reset();
    // Arm and enable motors / robot
    state_.armed = true;
    state_.motors_enabled = true;
    state_.balance_enabled = true;
    state_.mode = RobotMode::Armed;
}


// Disarm the robot
void BalanceBot::disarm() {
    // Disable
    state_.armed = false;
    state_.motors_enabled = false;
    state_.balance_enabled = false;
    // Reset balance controller
    balance_controller_.reset();
    force_motor_stop_();
    // Fall back to idle if not already faulted
    if (state_.mode != RobotMode::Fault) {
        state_.mode = RobotMode::Idle;
    }
}


// Apply a drive command
void BalanceBot::set_drive_command(const DriveCommand& command) {
    drive_command_ = command;
}


// Force an immediate stop
void BalanceBot::stop() {
    motor_command_ = MotorCommand{};
    force_motor_stop_();
}


// Expose the current top-level robot state
const RobotState& BalanceBot::state() const {
    return state_;
}


// Expose loop timing statistics
const LoopStats& BalanceBot::loop_stats() const {
    return loop_stats_;
}


// Read raw sensor values
void BalanceBot::read_sensors_() {
    // Read IMU sample
    imu_driver_.read(imu_sample_);

    // Read encoder sample
    encoder_driver_.read(encoder_sample_);
}


// Estimate state from raw sensor samples
void BalanceBot::estimate_state_(float dt_s) {
    // Update attitude estimate from IMU sample
    attitude_state_ = attitude_estimator_.update(
        imu_sample_,
        dt_s
    );

    // Update summary robot state
    state_.pitch_rad = attitude_state_.pitch_rad;
    state_.pitch_rate_rad_s = attitude_state_.pitch_rate_rad_s;
    state_.forward_velocity_m_s = 0.0f;
    state_.turn_rate_rad_s = 0.0f;
}


// Compute control commands
void BalanceBot::run_control_(float dt_s) {
    // Return motor commands to balance
    motor_command_ = balance_controller_.update(attitude_state_, dt_s);

    // Stop if controller refuses to generate a valid command
    if (!motor_command_.valid) {
        force_motor_stop_();
        return;
    }

    // Write outputs
    write_outputs_();
}


// Check safety rules
void BalanceBot::check_safety_() {
    // Attitude estimate must be valid (SensorTimeout)
    if (!attitude_state_.valid) {
        enter_fault_state_(FaultCode::SensorTimeout);
        return;
    }

    // If pitch exceeds the configured safe threshold, fault immediately (UnsafeTilt)
    if (std::fabs(attitude_state_.pitch_rad) > config::SafetyConfig::max_safe_pitch_rad) {
        enter_fault_state_(FaultCode::UnsafeTilt);
        return;
    }

    // Clear fault if we are still OK and have not already faulted
    if (state_.mode != RobotMode::Fault) {
        state_.fault = FaultCode::None;
    }
}


// Write outputs to motor driver
void BalanceBot::write_outputs_() {
    // If the motor command is invalid, do not pass it on
    if (!motor_command_.valid) {
        // Force motors to stop
        force_motor_stop_();
        return;
    }

    // Write motor command
    motor_driver_.write(motor_command_);

    // If we are actively controlling, the robot mode can move from Armed to Balancing
    if (state_.mode == RobotMode::Armed) {
        state_.mode = RobotMode::Balancing;
    }
}


// Enter fault state
void BalanceBot::enter_fault_state_(FaultCode fault) {
    state_.fault = fault;
    state_.mode = RobotMode::Fault;
    state_.armed = false;
    state_.motors_enabled = false;
    state_.balance_enabled = false;

    balance_controller_.reset();
    force_motor_stop_();
}


// Force motors to stop
void BalanceBot::force_motor_stop_() {
    motor_command_ = MotorCommand{};
    motor_driver_.stop();
}

}  // namespace balancebot
