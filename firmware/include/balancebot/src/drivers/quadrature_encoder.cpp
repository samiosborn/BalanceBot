// firmware/src/drivers/quadrature_encoder.cpp

#include "balancebot/drivers/hardware/quadrature_encoder.hpp"

#include <Arduino.h>

#include "config/robot/encoder_config.hpp"
#include "config/robot/geometry_config.hpp"
#include "config/robot/pin_config.hpp"

namespace balancebot {

// Static ISR instance pointers
QuadratureEncoder* QuadratureEncoder::left_instance_ = nullptr;
QuadratureEncoder* QuadratureEncoder::right_instance_ = nullptr;


// Constructor
QuadratureEncoder::QuadratureEncoder()
    : ready_(false),
      left_count_(0),
      right_count_(0),
      left_prev_state_(0),
      right_prev_state_(0),
      left_last_edge_us_(0),
      right_last_edge_us_(0),
      left_velocity_rad_s_(0.0f),
      right_velocity_rad_s_(0.0f),
      left_last_sample_us_(0),
      right_last_sample_us_(0),
      left_last_sample_count_(0),
      right_last_sample_count_(0),
      wheel_radius_m_(config::GeometryConfig::wheel_radius_m),
      wheel_rad_per_count_(config::EncoderConfig::wheel_rad_per_count) {}


// Bring up encoder GPIO / interrupt handling
bool QuadratureEncoder::begin() {
    // Configure encoder pins with pull-ups
    pinMode(config::PinConfig::left_encoder_a, INPUT_PULLUP);
    pinMode(config::PinConfig::left_encoder_b, INPUT_PULLUP);
    pinMode(config::PinConfig::right_encoder_a, INPUT_PULLUP);
    pinMode(config::PinConfig::right_encoder_b, INPUT_PULLUP);

    // Initialise the decoder with the current encoder pin states
    left_prev_state_ = read_state_(true);
    right_prev_state_ = read_state_(false);

    // Reset counts / timing
    const std::uint32_t now_us = static_cast<std::uint32_t>(micros());

    left_count_ = 0;
    right_count_ = 0;

    left_last_edge_us_ = now_us;
    right_last_edge_us_ = now_us;

    left_last_sample_us_ = now_us;
    right_last_sample_us_ = now_us;

    left_last_sample_count_ = 0;
    right_last_sample_count_ = 0;

    left_velocity_rad_s_ = 0.0f;
    right_velocity_rad_s_ = 0.0f;

    // Store instance pointers for ISR bridging
    left_instance_ = this;
    right_instance_ = this;

    // Attach one ISR per wheel to both channels of that wheel
    attachInterrupt(
        digitalPinToInterrupt(config::PinConfig::left_encoder_a),
        left_isr_,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(config::PinConfig::left_encoder_b),
        left_isr_,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(config::PinConfig::right_encoder_a),
        right_isr_,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(config::PinConfig::right_encoder_b),
        right_isr_,
        CHANGE
    );

    ready_ = true;
    return true;
}


// Whether the encoder subsystem is ready
bool QuadratureEncoder::is_ready() const {
    return ready_;
}


// Read one encoder sample
bool QuadratureEncoder::read(EncoderSample& sample) {
    // Start from a safe default sample
    sample = EncoderSample{};

    if (!ready_) {
        return false;
    }

    // Local copies of the latest encoder counts and edge timestamps
    std::int32_t left_count = 0;
    std::int32_t right_count = 0;
    std::uint32_t left_last_edge_us = 0;
    std::uint32_t right_last_edge_us = 0;

    const std::uint32_t now_us = static_cast<std::uint32_t>(micros());

    noInterrupts();
    left_count = left_count_;
    right_count = right_count_;
    left_last_edge_us = left_last_edge_us_;
    right_last_edge_us = right_last_edge_us_;
    interrupts();

    // Count changes since the last sampled read
    const std::int32_t left_count_delta = left_count - left_last_sample_count_;
    const std::int32_t right_count_delta = right_count - right_last_sample_count_;

    // Time since the last sampled read
    const std::uint32_t left_dt_us = now_us - left_last_sample_us_;
    const std::uint32_t right_dt_us = now_us - right_last_sample_us_;

    // Compute wheel angular velocities
    left_velocity_rad_s_ = compute_velocity_rad_s_(left_count_delta, left_dt_us);
    right_velocity_rad_s_ = compute_velocity_rad_s_(right_count_delta, right_dt_us);

    // Zero velocity if there has been no recent encoder activity
    if ((now_us - left_last_edge_us) > config::EncoderConfig::zero_velocity_timeout_us) {
        left_velocity_rad_s_ = 0.0f;
    }

    if ((now_us - right_last_edge_us) > config::EncoderConfig::zero_velocity_timeout_us) {
        right_velocity_rad_s_ = 0.0f;
    }

    // Fill output sample
    sample.left_counts = left_count;
    sample.right_counts = right_count;

    sample.left_wheel_rad = static_cast<float>(left_count) * wheel_rad_per_count_;
    sample.right_wheel_rad = static_cast<float>(right_count) * wheel_rad_per_count_;

    sample.left_wheel_rad_s = left_velocity_rad_s_;
    sample.right_wheel_rad_s = right_velocity_rad_s_;

    sample.left_wheel_m_s = left_velocity_rad_s_ * wheel_radius_m_;
    sample.right_wheel_m_s = right_velocity_rad_s_ * wheel_radius_m_;

    sample.timestamp_us = now_us;
    sample.valid = true;

    // Store this snapshot for the next read
    left_last_sample_us_ = now_us;
    right_last_sample_us_ = now_us;

    left_last_sample_count_ = left_count;
    right_last_sample_count_ = right_count;

    return true;
}


// Reset counts and velocity state
void QuadratureEncoder::reset() {
    const std::uint32_t now_us = static_cast<std::uint32_t>(micros());

    noInterrupts();
    left_count_ = 0;
    right_count_ = 0;

    left_prev_state_ = read_state_(true);
    right_prev_state_ = read_state_(false);

    left_last_edge_us_ = now_us;
    right_last_edge_us_ = now_us;
    interrupts();

    left_velocity_rad_s_ = 0.0f;
    right_velocity_rad_s_ = 0.0f;

    left_last_sample_us_ = now_us;
    right_last_sample_us_ = now_us;

    left_last_sample_count_ = 0;
    right_last_sample_count_ = 0;
}


// Read the current 2-bit quadrature state for one wheel
std::uint8_t QuadratureEncoder::read_state_(bool is_left) const {
    int a = 0;
    int b = 0;

    if (is_left) {
        a = digitalRead(config::PinConfig::left_encoder_a);
        b = digitalRead(config::PinConfig::left_encoder_b);
    } else {
        a = digitalRead(config::PinConfig::right_encoder_a);
        b = digitalRead(config::PinConfig::right_encoder_b);
    }

    return static_cast<std::uint8_t>((a << 1) | b);
}


// Decode one quadrature transition into count delta
std::int8_t QuadratureEncoder::decode_transition_(
    std::uint8_t previous_state,
    std::uint8_t current_state
) const {
    // Forward sequence: 00 -> 01 -> 11 -> 10 -> 00
    switch ((previous_state << 2) | current_state) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            return +1;

        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            return -1;

        default:
            return 0;
    }
}


// Update one wheel from a newly observed quadrature state
void QuadratureEncoder::update_wheel_state_(
    bool is_left,
    std::uint8_t current_state,
    std::uint32_t now_us
) {
    if (is_left) {
        const std::uint8_t previous_state = left_prev_state_;
        const std::int8_t delta = decode_transition_(previous_state, current_state);

        left_prev_state_ = current_state;

        if (delta != 0) {
            left_count_ += delta;
            left_last_edge_us_ = now_us;
        }
    } else {
        const std::uint8_t previous_state = right_prev_state_;
        const std::int8_t delta = decode_transition_(previous_state, current_state);

        right_prev_state_ = current_state;

        if (delta != 0) {
            right_count_ += delta;
            right_last_edge_us_ = now_us;
        }
    }
}


// Estimate wheel angular velocity from count / time change
float QuadratureEncoder::compute_velocity_rad_s_(
    std::int32_t count_delta,
    std::uint32_t dt_us
) const {
    if (dt_us == 0U) {
        return 0.0f;
    }

    const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;
    const float delta_rad = static_cast<float>(count_delta) * wheel_rad_per_count_;

    return delta_rad / dt_s;
}


// Static encoder interrupt handlers
void QuadratureEncoder::left_isr_() {
    if (left_instance_ != nullptr) {
        left_instance_->handle_left_edge_();
    }
}

void QuadratureEncoder::right_isr_() {
    if (right_instance_ != nullptr) {
        right_instance_->handle_right_edge_();
    }
}


// Handle one interrupt event for the left wheel
void QuadratureEncoder::handle_left_edge_() {
    const std::uint32_t now_us = static_cast<std::uint32_t>(micros());
    const std::uint8_t current_state = read_state_(true);
    update_wheel_state_(true, current_state, now_us);
}


// Handle one interrupt event for the right wheel
void QuadratureEncoder::handle_right_edge_() {
    const std::uint32_t now_us = static_cast<std::uint32_t>(micros());
    const std::uint8_t current_state = read_state_(false);
    update_wheel_state_(false, current_state, now_us);
}

}  // namespace balancebot