// firmware/src/main.cpp

#include <Arduino.h>

#include "balancebot/robot/balancebot.hpp"

#include "balancebot/drivers/hardware/icm20948.hpp"
#include "balancebot/drivers/hardware/quadrature_encoder.hpp"
#include "balancebot/drivers/hardware/qwiic_motor_driver.hpp"

#include "balancebot/estimation/complementary_filter.hpp"

#include "balancebot/control/pid_controller.hpp"
#include "balancebot/control/balance_controller.hpp"

#include "config/robot/board_config.hpp"

namespace balancebot {

// --- Subsystem objects ---
ICM20948 imu_driver;
QuadratureEncoder encoder_driver;
QwiicMotorDriver motor_driver;

ComplementaryFilter attitude_estimator;

PidController pid_controller;
BalanceController balance_controller(pid_controller);


// Top-level robot object
BalanceBot robot(
    imu_driver,
    encoder_driver,
    motor_driver,
    attitude_estimator,
    balance_controller
);

}  // namespace balancebot


void setup() {
    // Start serial
    Serial.begin(balancebot::config::BoardConfig::serial_baud);
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
    // Run one update step using current timestamp (in microseconds)
    balancebot::robot.update(static_cast<std::uint32_t>(micros()));

    // Don't arm the robot here
}