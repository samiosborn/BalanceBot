# BalanceBot

BalanceBot is a two-wheel self-balancing robot built around an ESP32.

Planned capabilities:
- balance upright
- drive forwards and backwards
- turn left and right
- accept phone control over Wi-Fi

## Hardware

Components:
- ELEGOO ESP32 development board
- 2x JGA25-37 12V 200 RPM brushed DC gear motors with rear quadrature encoders
- SparkFun Qwiic Motor Driver (SCMD)
- ICM-20948 IMU
- 2S LiPo power supply
- 5V 1A buck converter

## Repository structure

This repository is currently organised around:

- `config/` for robot, sensor, and control configuration
- `firmware/` for embedded C++ code, drivers, estimation, and control