// firmware/src/main.cpp

#include <Arduino.h>

#include "balancebot/robot/balancebot.hpp"

#include "balancebot/drivers/hardware/icm20948.hpp"
#include "balancebot/drivers/hardware/quadrature_encoder.hpp"
#include "balancebot/drivers/hardware/qwiic_motor_driver.hpp"

#include "balancebot/estimation/complementary_filter.hpp"

#include "balancebot/control/pid_controller.hpp"
#include "balancebot/control/balance_controller.hpp"

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
    // Start serial for bring-up logs
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
    // Run one update step using the current timestamp (microseconds)
    balancebot::robot.update(static_cast<std::uint32_t>(micros()));

    // Leave the robot disarmed here for safety during bring-up
}