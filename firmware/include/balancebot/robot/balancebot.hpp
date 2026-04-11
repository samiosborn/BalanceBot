// firmware/include/balancebot/robot/balancebot.hpp
#pragma once

#include <cstdint>

#include "balancebot/core/types.hpp"
#include "balancebot/estimation/estimation_types.hpp"

namespace balancebot {

// Interface for drivers
class IImuDriver; 
class IEncoderDriver;
class IMotorDriver; 

// Estimators
class AttitudeEstimator; 
class MotionEstimator; 

// Balance controller
class BalanceController;


// Top-level robot object
class BalanceBot {

// Public API
public: 
    // Constructor
    BalanceBot(
        IImuDriver& imu_driver,
        IEncoderDriver& encoder_driver,
        IMotorDriver& motor_driver,
        AttitudeEstimator& attitude_estimator,
        MotionEstimator& motion_estimator,
        BalanceController& balance_controller
    );

    // Begin robot bring-up
    bool begin();

    // Update step
    void update(std::uint32_t now_us);

    // Arm the robot
    void arm();

    // Disarm the robot
    void disarm();

    // Apply a high-level drive command
    void set_drive_command(const DriveCommand& command); 

    // Force an immediate stop
    void stop();

    // Reveal the current robot state (high-level)
    const RobotState& state() const;

    // Reveal loop timing statistics
    const LoopStats& loop_stats() const;


private: 
    // --- Private methods ---

    // Read raw sensor values
    void read_sensors_();

    // Estimate state from raw sensor samples
    void estimate_state_();

    // Check safety rules
    void check_safety_();

    // Compute control commands
    void run_control_();

    // Write outputs to motor driver
    void write_outputs_();

    // Enter fault state
    void enter_fault_state_(FaultCode fault);

    // Force motors to stop
    void force_motor_stop_();


    // --- References ---

    // Reference to the drivers
    IImuDriver& imu_driver_;
    IEncoderDriver& encoder_driver_;
    IMotorDriver& motor_driver_;

    // Reference to the estimators
    AttitudeEstimator& attitude_estimator_;
    MotionEstimator& motion_estimator_;

    // Reference to the balance controller
    BalanceController& balance_controller_;


    // --- Current State ---

    // Current robot state
    RobotState state_;

    // Last drive command
    DriveCommand drive_command_;

    // Last low-level motor command
    MotorCommand motor_command_;

    // Current loop timing stats
    LoopStats loop_stats_;

    // Last update time
    std::uint32_t last_update_us_ = 0; 

    // First update? 
    bool first_update_ = true; 

};


} // namespace balancebot