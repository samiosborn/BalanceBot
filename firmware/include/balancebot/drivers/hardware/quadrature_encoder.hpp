// firmware/include/balancebot/drivers/hardware/quadrature_encoder.hpp
#pragma once

#include <cstdint>

#include "balancebot/drivers/interfaces/encoder_interface.hpp"

namespace balancebot {

// Quadrature encoder driver for the left and right wheel encoders
class QuadratureEncoder : public IEncoderDriver {
public:
    // Constructor
    QuadratureEncoder();

    // Virtual destructor
    ~QuadratureEncoder() override = default;


    // --- IEncoderDriver API ---

    // Bring up encoder GPIO / interrupt handling
    bool begin() override;

    // Whether the encoder subsystem is ready
    bool is_ready() const override;

    // Read one encoder sample
    bool read(EncoderSample& sample) override;

    // Reset counts and velocity state
    void reset() override;

private:
    // --- GPIO / quadrature helpers ---

    // Read the current 2-bit quadrature state for one wheel
    std::uint8_t read_state_(bool is_left) const;

    // Decode one quadrature transition into count delta
    std::int8_t decode_transition_(
        std::uint8_t previous_state,
        std::uint8_t current_state
    ) const;

    // Update one wheel from a newly observed quadrature state
    void update_wheel_state_(bool is_left, std::uint8_t current_state, std::uint32_t now_us);

    // Estimate wheel angular velocity from count / time change
    float compute_velocity_rad_s_(
        std::int32_t count_delta,
        std::uint32_t dt_us
    ) const;


    // --- Interrupt Service Routine (ISR) ---
    
    // Static interrupt handlers for the left and right encoder channels
    static void left_isr_();
    static void right_isr_();

    // Handle one interrupt event for the left wheel
    void handle_left_edge_();

    // Handle one interrupt event for the right wheel
    void handle_right_edge_();

    
    // --- Driver state ---

    // Whether bring-up completed successfully
    bool ready_ = false;

    // Left encoder count
    volatile std::int32_t left_count_ = 0;

    // Right encoder count
    volatile std::int32_t right_count_ = 0;

    // Previous quadrature state
    volatile std::uint8_t left_prev_state_ = 0;
    volatile std::uint8_t right_prev_state_ = 0;

    // Last edge timestamps
    volatile std::uint32_t left_last_edge_us_ = 0;
    volatile std::uint32_t right_last_edge_us_ = 0;

    // Last velocity estimates
    float left_velocity_rad_s_ = 0.0f;
    float right_velocity_rad_s_ = 0.0f;

    // Last sample timestamps used for readout
    std::uint32_t left_last_sample_us_ = 0;
    std::uint32_t right_last_sample_us_ = 0;

    // Last sampled counts used for velocity estimation
    std::int32_t left_last_sample_count_ = 0;
    std::int32_t right_last_sample_count_ = 0;

    // Conversion constants
    const float wheel_radius_m_;
    const float wheel_rad_per_count_;

    // Static instance pointers for ISR bridging
    static QuadratureEncoder* left_instance_;
    static QuadratureEncoder* right_instance_;
};

}  // namespace balancebot