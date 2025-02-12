# open-servo-controller

An smart open servo controller board and firmware for common servos such as SG90 / MG90.

## Goals

Currently this project is under development. I'm working on this in my spare time and there is no guarantee that any of the goals will be achieved at all.

* More sophisticated control scheme via cascaded control loops instead of simple PID.
* Allow Position / Torque / Current feedback.
* I2C & UART communication.
* Clear and easy instructions to modify existing servos.
* Arduino plugin to control servo via I2C / UART.

## Status

* **Servo Dev Board** (`hardware/servo-dev-board`) - This is the board first developed to facility the development of the firmware. Currently, all major components except I2C & UART has been tested to be working correctly.
* **Firmware** (`firmware/open-servo-controller`) - The firmware is written in Rust. This is currently under active development.

All other folders are not ready yet.
