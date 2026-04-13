// firmware/src/main.cpp

#include <Arduino.h>
#include <cmath>

#include "balancebot/robot/balancebot.hpp"

#include "balancebot/drivers/hardware/icm20948.hpp"
#include "balancebot/drivers/hardware/quadrature_encoder.hpp"
#include "balancebot/drivers/hardware/qwiic_motor_driver.hpp"

#include "balancebot/estimation/complementary_filter.hpp"

#include "balancebot/control/pid_controller.hpp"
#include "balancebot/control/balance_controller.hpp"

#include "balancebot/debug/telemetry.hpp"

namespace balancebot {

// Subsystem objects
ICM20948 imu_driver;
QuadratureEncoder encoder_driver;
QwiicMotorDriver motor_driver;

ComplementaryFilter attitude_estimator;

PidController pid_controller;
BalanceController balance_controller(pid_controller);

// Robot object
BalanceBot robot(
    imu_driver,
    encoder_driver,
    motor_driver,
    attitude_estimator,
    balance_controller
);

}  // namespace balancebot


// Send a zero-velocity balance command
static void send_balance_hold_command(std::uint32_t now_us) {
    balancebot::DriveCommand command{};

    command.desired_forward_velocity_m_s = 0.0f;
    command.desired_turn_rate_rad_s = 0.0f;
    command.enable_balance = true;
    command.enable_motors = true;
    command.timestamp_us = now_us;

    balancebot::robot.set_drive_command(command);
}


void setup() {
    // Serial
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("BOOT");

    // Bring up the robot
    const bool ok = balancebot::robot.begin();

    if (!ok) {
        Serial.println("BEGIN FAILED");
        return;
    }

    Serial.println("READY");
}


void loop() {
    // Time
    const std::uint32_t now_us = static_cast<std::uint32_t>(micros());
    const std::uint32_t now_ms = millis();

    // Timing and limits
    static const std::uint32_t arm_delay_ms = 3000U;
    static const std::uint32_t armed_duration_ms = 10000U;
    static const float max_pitch_to_arm_rad = 0.20f;
    static const float emergency_pitch_limit_rad = 0.50f;

    // State
    static bool started = false;
    static bool finished = false;
    static std::uint32_t boot_time_ms = now_ms;
    static std::uint32_t arm_time_ms = 0U;

    // Keep the current drive command fresh while armed
    if (started && !finished) {
        send_balance_hold_command(now_us);
    }

    // Update robot
    balancebot::robot.update(now_us);

    const balancebot::RobotState& state = balancebot::robot.state();

    // Auto-arm after a short delay if the robot is near upright
    if (!started && !finished && (now_ms - boot_time_ms >= arm_delay_ms)) {
        if ((state.fault == balancebot::FaultCode::None) &&
            (std::fabs(state.pitch_rad) <= max_pitch_to_arm_rad)) {
            Serial.println("arm()");
            balancebot::robot.arm();
            started = true;
            arm_time_ms = now_ms;
        }
    }

    // Stop on any fault
    if (started && !finished && state.fault != balancebot::FaultCode::None) {
        Serial.println("abort: fault");
        balancebot::robot.disarm();
        balancebot::robot.stop();
        finished = true;
    }

    // Stop on excessive tilt
    if (started && !finished && std::fabs(state.pitch_rad) > emergency_pitch_limit_rad) {
        Serial.println("abort: tilt");
        balancebot::robot.disarm();
        balancebot::robot.stop();
        finished = true;
    }

    // Stop after the armed window
    if (started && !finished && (now_ms - arm_time_ms >= armed_duration_ms)) {
        Serial.println("disarm()");
        balancebot::robot.disarm();
        balancebot::robot.stop();
        finished = true;
    }

    // Periodic telemetry
    static std::uint32_t last_print_ms = 0;
    if (now_ms - last_print_ms >= 200U) {
        last_print_ms = now_ms;

        balancebot::print_status_line(state);

        balancebot::EncoderSample encoder_sample{};
        if (balancebot::encoder_driver.read(encoder_sample)) {
            balancebot::print_encoder_sample(encoder_sample);
        } else {
            balancebot::print_encoder_read_failed();
        }
    }
}