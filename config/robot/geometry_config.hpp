// config/robot/geometry_config.hpp
#pragma once

namespace balancebot::config {

struct GeometryConfig {
    // Wheel diameter (m)
    static constexpr float wheel_diameter_m = 0.065f;

    // Wheel radius (m)
    static constexpr float wheel_radius_m = wheel_diameter_m * 0.5f;

    // Distance between left and right wheel contact lines (m)
    static constexpr float axle_track_m = 0.150f;
};

}  // namespace balancebot::config