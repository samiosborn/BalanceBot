// firmware/src/debug/telemetry.cpp

#include "balancebot/debug/telemetry.hpp"

#include <Arduino.h>

#include "balancebot/core/debug_codes.hpp"

namespace balancebot {

// Print one robot status line
void print_status_line(const RobotState& state) {
    Serial.print("mode=");
    Serial.print(mode_to_string(state.mode));

    Serial.print(" fault=");
    Serial.print(fault_to_string(state.fault));

    Serial.print(" pitch=");
    Serial.print(state.pitch_rad, 4);

    Serial.print(" pitch_rate=");
    Serial.print(state.pitch_rate_rad_s, 4);

    Serial.print(" armed=");
    Serial.println(state.armed ? 1 : 0);
}


// Print one encoder sample
void print_encoder_sample(const EncoderSample& sample) {
    Serial.print("enc left_counts=");
    Serial.print(sample.left_counts);
    Serial.print(" right_counts=");
    Serial.print(sample.right_counts);
    Serial.print(" left_rad_s=");
    Serial.print(sample.left_wheel_rad_s, 4);
    Serial.print(" right_rad_s=");
    Serial.println(sample.right_wheel_rad_s, 4);
}


// Print encoder read failure
void print_encoder_read_failed() {
    Serial.println("enc read failed");
}

}  // namespace balancebot