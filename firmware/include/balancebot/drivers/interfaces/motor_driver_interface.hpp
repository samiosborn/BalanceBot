// firmware/include/balancebot/drivers/interfaces/motor_driver_interface.hpp
#pragma once

#include "balancebot/core/types.hpp"

namespace balancebot {

// Interface for Motor driver
class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    // --- Bring-up ---
    // Initialise the motor driver
    virtual bool begin() = 0;

    // Whether the motor driver is ready to accept commands
    virtual bool is_ready() const = 0;


    // --- Command output ---
    // Write a full motor command
    virtual bool write(const MotorCommand& command) = 0;

    // Convenience overload for direct left/right duty writes
    virtual bool write(float left_duty, float right_duty) = 0;


    // --- Stop ---
    // Immediately command zero output to both motors
    virtual void stop() = 0;
    
};

}  // namespace balancebot
