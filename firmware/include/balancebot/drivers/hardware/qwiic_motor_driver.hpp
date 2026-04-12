// firmware/include/balancebot/drivers/hardware/qwiic_motor_driver.hpp
#pragma once

#include <cstdint>

#include <SCMD.h>

#include "balancebot/core/types.hpp"
#include "balancebot/drivers/interfaces/motor_driver_interface.hpp"

namespace balancebot {

// Motor driver for  SparkFun Qwiic SCMD board
class QwiicMotorDriver : public IMotorDriver {
public:
    // Constructor
    QwiicMotorDriver();

    // Virtual destructor
    ~QwiicMotorDriver() override = default;


    // --- IMotorDriver API ---

    // Bring-up the motor driver
    bool begin() override;

    // Whether the motor driver is ready
    bool is_ready() const override;

    // Write a motor command (to both motors)
    bool write(const MotorCommand& command) override;

    // Write direct left/right duty commands (convenience overload)
    bool write(float left_duty, float right_duty) override;

    // Stop both motors immediately
    void stop() override;

private:
    // --- Write helpers ---

    // Encode one (normalised) duty command into: direction bit, hardware level
    void encode_level_(
        float duty,
        std::uint8_t& direction,
        std::uint8_t& level
    ) const;

    // Write one motor output
    void write_motor_(std::uint8_t motor_id, float duty);

    // Clamp duty
    float clamp_duty_(float duty) const;


    // --- Driver state ---

    // Whether bring-up completed successfully
    bool ready_ = false;

    // Underlying SparkFun SCMD driver object
    SCMD scmd_;

    // Driver I2C address
    const std::uint8_t i2c_address_;

    // Logical motor IDs on the SCMD board
    const std::uint8_t left_motor_id_;
    const std::uint8_t right_motor_id_;

    // Maximum allowed duty (magnitude)
    const float max_duty_;

    // Maximum hardware output level
    const std::uint8_t max_hardware_level_;

    // Motor inversion flags
    const bool invert_left_motor_;
    const bool invert_right_motor_;

    // Command deadband
    const float command_deadband_;
};

}  // namespace balancebot