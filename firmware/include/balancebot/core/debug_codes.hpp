// firmware/include/balancebot/core/debug_codes.hpp
#pragma once

#include "balancebot/core/types.hpp"

namespace balancebot {

// Convert robot mode to a string
const char* mode_to_string(RobotMode mode);

// Convert fault code to a string
const char* fault_to_string(FaultCode fault);

}  // namespace balancebot