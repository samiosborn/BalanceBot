// firmware/src/core/debug_codes.cpp

#include "balancebot/core/debug_codes.hpp"

namespace balancebot {

// Convert robot mode to a string
const char* mode_to_string(RobotMode mode) {
    switch (mode) {
        case RobotMode::Boot:          return "Boot";
        case RobotMode::Idle:          return "Idle";
        case RobotMode::Calibration:   return "Calibration";
        case RobotMode::ImuTest:       return "ImuTest";
        case RobotMode::EncoderTest:   return "EncoderTest";
        case RobotMode::MotorTest:     return "MotorTest";
        case RobotMode::Armed:         return "Armed";
        case RobotMode::Balancing:     return "Balancing";
        case RobotMode::RemoteControl: return "RemoteControl";
        case RobotMode::Fault:         return "Fault";
        default:                       return "Unknown";
    }
}


// Convert fault code to a string
const char* fault_to_string(FaultCode fault) {
    switch (fault) {
        case FaultCode::None:                return "None";
        case FaultCode::ImuNotReady:         return "ImuNotReady";
        case FaultCode::MotorDriverNotReady: return "MotorDriverNotReady";
        case FaultCode::EncoderNotReady:     return "EncoderNotReady";
        case FaultCode::UnsafeTilt:          return "UnsafeTilt";
        case FaultCode::CommandTimeout:      return "CommandTimeout";
        case FaultCode::SensorTimeout:       return "SensorTimeout";
        case FaultCode::InvalidState:        return "InvalidState";
        case FaultCode::InternalError:       return "InternalError";
        default:                             return "Unknown";
    }
}

}  // namespace balancebot