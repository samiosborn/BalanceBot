// firmware/include/balancebot/debug/telemetry.hpp
#pragma once

#include "balancebot/core/types.hpp"
#include "balancebot/drivers/interfaces/encoder_interface.hpp"

namespace balancebot {

// Print one robot status line
void print_status_line(const RobotState& state);

// Print one encoder sample
void print_encoder_sample(const EncoderSample& sample);

// Print encoder read failure
void print_encoder_read_failed();

}  // namespace balancebot