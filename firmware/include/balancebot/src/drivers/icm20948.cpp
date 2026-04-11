// firmware/src/drivers/icm20948.cpp

#include "balancebot/drivers/hardware/icm20948.hpp"

#include <Arduino.h>
#include <Wire.h>

#include "balancebot/core/physics_constants.hpp"
#include "config/robot/imu_config.hpp"
#include "config/robot/pin_config.hpp"

namespace balancebot {
namespace {

// --- Register / bank constants ---

// USER BANK select register
constexpr std::uint8_t kRegBankSel = 0x7F;

// USER BANK 0
constexpr std::uint8_t kWhoAmIReg = 0x00;
constexpr std::uint8_t kExpectedWhoAmI = 0xEA;
constexpr std::uint8_t kPwrMgmt1Reg = 0x06;

// USER BANK 2
constexpr std::uint8_t kGyroConfig1Reg = 0x01;
constexpr std::uint8_t kAccelConfigReg = 0x14;

// USER BANK 0 accel output registers
constexpr std::uint8_t kAccelXoutH = 0x2D;
constexpr std::uint8_t kAccelXoutL = 0x2E;
constexpr std::uint8_t kAccelYoutH = 0x2F;
constexpr std::uint8_t kAccelYoutL = 0x30;
constexpr std::uint8_t kAccelZoutH = 0x31;
constexpr std::uint8_t kAccelZoutL = 0x32;

// USER BANK 0 gyro output registers
constexpr std::uint8_t kGyroXoutH = 0x33;
constexpr std::uint8_t kGyroXoutL = 0x34;
constexpr std::uint8_t kGyroYoutH = 0x35;
constexpr std::uint8_t kGyroYoutL = 0x36;
constexpr std::uint8_t kGyroZoutH = 0x37;
constexpr std::uint8_t kGyroZoutL = 0x38;

}  // namespace

// Constructor
ICM20948::ICM20948()
    // Driver starts not ready
    : ready_(false),

      // Default user bank
      current_bank_(0),

      // Default accel bias
      accel_bias_x_m_s2_(0.0f),
      accel_bias_y_m_s2_(0.0f),
      accel_bias_z_m_s2_(0.0f),

      // Default gyro bias
      gyro_bias_x_rad_s_(0.0f),
      gyro_bias_y_rad_s_(0.0f),
      gyro_bias_z_rad_s_(0.0f),

      // Sensitivity constants
      accel_sensitivity_lsb_per_g_(config::ImuConfig::accel_sensitivity_lsb_per_g),
      gyro_sensitivity_lsb_per_dps_(config::ImuConfig::gyro_sensitivity_lsb_per_dps) {}


// Bring up the IMU
bool ICM20948::begin() {
    // Start I2C
    Wire.begin(config::PinConfig::i2c_sda, config::PinConfig::i2c_scl);

    // Assume bank 0 after reset
    current_bank_ = 0U;

    // Wake the IMU
    wake_();

    // Check identity
    if (!check_identity_()) {
        ready_ = false;
        return false;
    }

    // Configure gyro and accelerometer
    configure_gyro_();
    configure_accel_();

    ready_ = true;
    return true;
}


// Whether the IMU driver is ready
bool ICM20948::is_ready() const {
    return ready_;
}


// Read one IMU sample
bool ICM20948::read(ImuSample& sample) {
    // Start from a safe default sample
    sample = ImuSample{};

    // Refuse reads if the driver is not ready
    if (!ready_) {
        return false;
    }

    // Raw accel counts
    std::int16_t ax_counts = 0;
    std::int16_t ay_counts = 0;
    std::int16_t az_counts = 0;

    // Raw gyro counts
    std::int16_t gx_counts = 0;
    std::int16_t gy_counts = 0;
    std::int16_t gz_counts = 0;

    // Read raw registers
    read_raw_accel_(ax_counts, ay_counts, az_counts);
    read_raw_gyro_(gx_counts, gy_counts, gz_counts);

    // Convert accel counts -> SI
    float ax_m_s2 = accel_counts_to_m_s2_(ax_counts);
    float ay_m_s2 = accel_counts_to_m_s2_(ay_counts);
    float az_m_s2 = accel_counts_to_m_s2_(az_counts);

    // Convert gyro counts -> SI
    float gx_rad_s = gyro_counts_to_rad_s_(gx_counts);
    float gy_rad_s = gyro_counts_to_rad_s_(gy_counts);
    float gz_rad_s = gyro_counts_to_rad_s_(gz_counts);

    // Apply stored bias
    apply_accel_bias_(ax_m_s2, ay_m_s2, az_m_s2);
    apply_gyro_bias_(gx_rad_s, gy_rad_s, gz_rad_s);

    // Fill robot-frame sample
    fill_robot_frame_sample_(
        ax_m_s2,
        ay_m_s2,
        az_m_s2,
        gx_rad_s,
        gy_rad_s,
        gz_rad_s,
        static_cast<std::uint32_t>(micros()),
        sample
    );

    return sample.valid;
}


// Run stationary calibration
ImuCalibrationResult ICM20948::calibrate(std::uint32_t num_samples) {
    // Default failed result
    ImuCalibrationResult result{};

    // Refuse calibration if driver is not ready or no samples requested
    if (!ready_ || num_samples == 0U) {
        return result;
    }

    // Accumulated accel sums
    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;

    // Accumulated gyro sums
    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;

    // Collect stationary samples
    for (std::uint32_t i = 0; i < num_samples; ++i) {
        std::int16_t ax_counts = 0;
        std::int16_t ay_counts = 0;
        std::int16_t az_counts = 0;

        std::int16_t gx_counts = 0;
        std::int16_t gy_counts = 0;
        std::int16_t gz_counts = 0;

        // Read raw data
        read_raw_accel_(ax_counts, ay_counts, az_counts);
        read_raw_gyro_(gx_counts, gy_counts, gz_counts);

        // Convert and accumulate accel
        ax_sum += accel_counts_to_m_s2_(ax_counts);
        ay_sum += accel_counts_to_m_s2_(ay_counts);
        az_sum += accel_counts_to_m_s2_(az_counts);

        // Convert and accumulate gyro
        gx_sum += gyro_counts_to_rad_s_(gx_counts);
        gy_sum += gyro_counts_to_rad_s_(gy_counts);
        gz_sum += gyro_counts_to_rad_s_(gz_counts);

        // Small spacing between stationary samples
        delay(2);
    }

    const float inv_n = 1.0f / static_cast<float>(num_samples);

    // Mean accel
    const float ax_mean = ax_sum * inv_n;
    const float ay_mean = ay_sum * inv_n;
    const float az_mean = az_sum * inv_n;

    // Mean gyro
    const float gx_mean = gx_sum * inv_n;
    const float gy_mean = gy_sum * inv_n;
    const float gz_mean = gz_sum * inv_n;

    // Store gyro bias directly
    gyro_bias_x_rad_s_ = gx_mean;
    gyro_bias_y_rad_s_ = gy_mean;
    gyro_bias_z_rad_s_ = gz_mean;

    // Store accel bias assuming stationary upright calibration in sensor frame
    accel_bias_x_m_s2_ = ax_mean - 0.0f;
    accel_bias_y_m_s2_ = ay_mean - 0.0f;
    accel_bias_z_m_s2_ = az_mean - PhysicsConstants::gravity_m_s2;

    result.success = true;
    result.samples_used = num_samples;
    return result;
}


// Reset stored calibration / state
void ICM20948::reset() {
    // Clear accel bias
    accel_bias_x_m_s2_ = 0.0f;
    accel_bias_y_m_s2_ = 0.0f;
    accel_bias_z_m_s2_ = 0.0f;

    // Clear gyro bias
    gyro_bias_x_rad_s_ = 0.0f;
    gyro_bias_y_rad_s_ = 0.0f;
    gyro_bias_z_rad_s_ = 0.0f;
}


// Write one register in the current bank
void ICM20948::write_register_(std::uint8_t reg, std::uint8_t value) {
    Wire.beginTransmission(config::ImuConfig::i2c_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}


// Read one register in the current bank
std::uint8_t ICM20948::read_register_(std::uint8_t reg) const {
    // Write register address without releasing the bus
    Wire.beginTransmission(config::ImuConfig::i2c_address);
    Wire.write(reg);
    Wire.endTransmission(false);

    // Request one byte
    Wire.requestFrom(static_cast<int>(config::ImuConfig::i2c_address), 1);

    if (Wire.available() < 1) {
        return 0U;
    }

    return static_cast<std::uint8_t>(Wire.read());
}


// Change active user bank
void ICM20948::set_bank_(std::uint8_t bank) {
    // Bank select uses bits [5:4]
    const std::uint8_t value = static_cast<std::uint8_t>((bank & 0x03U) << 4);

    write_register_(kRegBankSel, value);
    current_bank_ = bank;
}


// Read one register from a specific bank
std::uint8_t ICM20948::read_bank_register_(std::uint8_t bank, std::uint8_t reg) {
    if (current_bank_ != bank) {
        set_bank_(bank);
    }

    return read_register_(reg);
}


// Write one register in a specific bank
void ICM20948::write_bank_register_(std::uint8_t bank, std::uint8_t reg, std::uint8_t value) {
    if (current_bank_ != bank) {
        set_bank_(bank);
    }

    write_register_(reg, value);
}


// Wake the IMU from sleep
void ICM20948::wake_() {
    // USER BANK 0
    // Write the configured PWR_MGMT_1 value
    write_bank_register_(0U, kPwrMgmt1Reg, config::ImuConfig::pwr_mgmt_1_value);

    // Let the device stabilise
    delay(50);
}


// Check WHO_AM_I
bool ICM20948::check_identity_() {
    const std::uint8_t who_am_i = read_bank_register_(0U, kWhoAmIReg);
    return who_am_i == kExpectedWhoAmI;
}


// Configure gyroscope
void ICM20948::configure_gyro_() {
    // USER BANK 2, GYRO_CONFIG_1
    const std::uint8_t gyro_config_1 = static_cast<std::uint8_t>(
        ((config::ImuConfig::gyro_dlpf_cfg & 0x07U) << 3) |
        ((config::ImuConfig::gyro_fs_sel & 0x03U) << 1) |
        (config::ImuConfig::gyro_fchoice & 0x01U)
    );

    write_bank_register_(2U, kGyroConfig1Reg, gyro_config_1);
}


// Configure accelerometer
void ICM20948::configure_accel_() {
    // USER BANK 2, ACCEL_CONFIG
    const std::uint8_t accel_config = static_cast<std::uint8_t>(
        ((config::ImuConfig::accel_dlpf_cfg & 0x07U) << 3) |
        ((config::ImuConfig::accel_fs_sel & 0x03U) << 1) |
        (config::ImuConfig::accel_fchoice & 0x01U)
    );

    write_bank_register_(2U, kAccelConfigReg, accel_config);
}


// Read raw accelerometer counts
void ICM20948::read_raw_accel_(std::int16_t& ax, std::int16_t& ay, std::int16_t& az) {
    const std::uint8_t ax_h = read_bank_register_(0U, kAccelXoutH);
    const std::uint8_t ax_l = read_bank_register_(0U, kAccelXoutL);

    const std::uint8_t ay_h = read_bank_register_(0U, kAccelYoutH);
    const std::uint8_t ay_l = read_bank_register_(0U, kAccelYoutL);

    const std::uint8_t az_h = read_bank_register_(0U, kAccelZoutH);
    const std::uint8_t az_l = read_bank_register_(0U, kAccelZoutL);

    ax = combine_bytes_(ax_h, ax_l);
    ay = combine_bytes_(ay_h, ay_l);
    az = combine_bytes_(az_h, az_l);
}


// Read raw gyroscope counts
void ICM20948::read_raw_gyro_(std::int16_t& gx, std::int16_t& gy, std::int16_t& gz) {
    const std::uint8_t gx_h = read_bank_register_(0U, kGyroXoutH);
    const std::uint8_t gx_l = read_bank_register_(0U, kGyroXoutL);

    const std::uint8_t gy_h = read_bank_register_(0U, kGyroYoutH);
    const std::uint8_t gy_l = read_bank_register_(0U, kGyroYoutL);

    const std::uint8_t gz_h = read_bank_register_(0U, kGyroZoutH);
    const std::uint8_t gz_l = read_bank_register_(0U, kGyroZoutL);

    gx = combine_bytes_(gx_h, gx_l);
    gy = combine_bytes_(gy_h, gy_l);
    gz = combine_bytes_(gz_h, gz_l);
}


// Combine two bytes into a signed 16-bit integer
std::int16_t ICM20948::combine_bytes_(std::uint8_t high, std::uint8_t low) const {
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(high) << 8) |
        static_cast<std::uint16_t>(low)
    );
}


// Convert raw accel counts to m/s^2
float ICM20948::accel_counts_to_m_s2_(std::int16_t counts) const {
    // counts -> g
    const float accel_g = static_cast<float>(counts) / accel_sensitivity_lsb_per_g_;

    // g -> m/s^2
    return accel_g * PhysicsConstants::gravity_m_s2;
}


// Convert raw gyro counts to rad/s
float ICM20948::gyro_counts_to_rad_s_(std::int16_t counts) const {
    // counts -> deg/s
    const float gyro_dps = static_cast<float>(counts) / gyro_sensitivity_lsb_per_dps_;

    // deg/s -> rad/s
    return gyro_dps * PhysicsConstants::deg_to_rad;
}


// Apply stored accel bias
void ICM20948::apply_accel_bias_(float& ax, float& ay, float& az) const {
    ax -= accel_bias_x_m_s2_;
    ay -= accel_bias_y_m_s2_;
    az -= accel_bias_z_m_s2_;
}


// Apply stored gyro bias
void ICM20948::apply_gyro_bias_(float& gx, float& gy, float& gz) const {
    gx -= gyro_bias_x_rad_s_;
    gy -= gyro_bias_y_rad_s_;
    gz -= gyro_bias_z_rad_s_;
}


// Fill a robot-frame IMU sample
void ICM20948::fill_robot_frame_sample_(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz,
    std::uint32_t timestamp_us,
    ImuSample& sample
) const {
    // Mapping assumption:
    // sensor X -> robot forward
    // sensor Y -> robot left
    // sensor Z -> robot up
    const float accel_forward = config::ImuConfig::invert_pitch ? -ax : ax;
    const float gyro_pitch = config::ImuConfig::invert_pitch ? -gy : gy;

    sample.accel_forward_m_s2 = accel_forward;
    sample.accel_left_m_s2 = ay;
    sample.accel_up_m_s2 = az;

    sample.gyro_roll_rad_s = gx;
    sample.gyro_pitch_rad_s = gyro_pitch;
    sample.gyro_yaw_rad_s = gz;

    sample.timestamp_us = timestamp_us;
    sample.valid = true;
}

}  // namespace balancebot