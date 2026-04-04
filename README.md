# BalanceBot

BalanceBot is a two-wheel self-balancing robot. 

BalanceBot will be able to:
- balance upright
- drive forwards and backwards
- turn left and right
- be controlled from a phone over Wi-Fi

## Hardware

- ESP32
- dual brushed gear motors with quadrature encoders
- IMU for tilt and angular rotation
- motor driver
- 2S LiPo power system
- 5V1A buck converter

## Repository structure

This repository will likely be organised around:

- `docs/` for build logs, notes, and diagrams
- `hardware/` for pinouts, wiring, and parts information
- `firmware/` for embedded code
- `tools/` for scripts used during development and tuning
- `media/` for photos and videos of progress
