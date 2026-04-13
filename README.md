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
- diymore 30-pin ESP32 1-in-2 breakout board with screw terminals
- 2x JGA25-37 12V 200 RPM brushed DC gear motors with rear quadrature encoders
- SparkFun Qwiic Motor Driver (SCMD)
- ICM-20948 IMU
- 2S LiPo power supply
- 5V 1A buck converter

## Pin layout

Current ESP32 pin usage:
- GPIO21 -> I2C SDA
- GPIO22 -> I2C SCL
- GPIO32 -> left encoder A
- GPIO33 -> left encoder B
- GPIO25 -> right encoder A
- GPIO26 -> right encoder B
- 3V3 -> Qwiic logic power and encoder VCC
- GND -> common ground for Qwiic and encoders
- 5V / VIN -> power input from buck converter

The ESP32 is mounted on a 30-pin breakout board, so each pin is available both on a header and on a screw terminal. The screw terminals are used for the main wiring. A small breakout board is for 3.3V and GND from ESP32 for the Qwiic and encoder power. 

## Repository structure

This repository is currently organised around:

- `config/` for robot, sensor, and control configuration
- `firmware/` for embedded C++ code, drivers, estimation, and control