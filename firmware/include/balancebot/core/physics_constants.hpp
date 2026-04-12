// firmware/include/balancebot/core/physics_constants.hpp
#pragma once

namespace balancebot {

// Physics constants
struct PhysicsConstants {
    // Pi
    static constexpr float pi = 3.14159265358979323846f;

    // Standard gravitational acceleration (m/s^2)
    static constexpr float gravity_m_s2 = 9.80665f;

    // Degrees to radians conversion factor
    static constexpr float deg_to_rad = pi / 180.0f;
};

}  // namespace balancebot