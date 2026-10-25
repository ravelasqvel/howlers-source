# Sensor Fusion Velocity Control

## Description

This project implements state estimation (position and velocity) for a BLDC motor using sensor fusion (encoder, lidar, and IMU) through a Kalman Filter. The estimated states are fed into PID controllers for precise control of the motor's velocity and position. The system runs on an ESP32-S3 using FreeRTOS and ESP-IDF.

The code is structured into modular components for easy portability and adaptation to other microcontrollers, through a hardware abstraction layer (HAL) defined in the `platform` directory.


## Task Structure

The system is organized into **seven FreeRTOS tasks**, each with a specific role. The **system initialization** (Init System) is not a task itself but runs before the scheduler starts, setting up drivers, peripherals, and shared resources.

<p align="center">
  <img src="docs/task_diagram.jpg" alt="FreeRTOS Task Diagram" width="700"/>
</p>

### ✔️ Tasks:

1. **Trigger Task**: Periodically activated by a timer interrupt. It synchronizes all sensor reads and control updates.
2. **Hall Sensor Task**: Acquires angular position data using the **AS5600** magnetic encoder.
3. **IMU Task**: Retrieves linear acceleration and angular velocity from the **BNO055** IMU.
4. **Lidar Task**: Measures distance using the **VL53L1X** ToF sensor.
5. **Control Task**: Performs sensor fusion using a Kalman filter and computes the control output (incremental PID). It also pushes relevant data into a queue.
6. **UART Task**: Receives setpoint commands from the serial interface and logs data for debugging or telemetry.
7. **Save Task**: Reads data from a queue (pushed by the control task) and stores it for offline analysis (e.g., on an SD card or internal memory).

> 🛠 **Init System**: Although not a task, this function runs at system startup to initialize hardware abstraction, timers, drivers, queues, and shared variables.
---

## Features

- **Kalman Filter** for sensor fusion (encoder, IMU, and LIDAR)
- **Incremental PID Controller** for velocity and position regulation
- **FreeRTOS-based** architecture with 7 independent tasks
- **Modular driver structure** using ESP-IDF components
- **Platform Abstraction Layer** to support multiple MCUs
- UART interface for runtime configuration and data output


## Hardware Overview

- **MCU:** ESP32-S3 (using [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/))
- **Motor:** Brushless DC Motor (BLDC)
- **ESC:** Electronic Speed Controller compatible with PWM
- **Sensors:**
  - AS5600 (Magnetic Encoder)
  - VL53L1X (Time-of-Flight Lidar)
  - BNO055 (9DOF IMU with sensor fusion)
- **UART Serial** interface for command input


## Project Structure

```text
├── components/
│   ├── control/
│   │   └── control_senfusion.c/.h
│   ├── drivers/
│   │   ├── bldc_pwm/
│   │   ├── bno055/
│   │   ├── as5600/
│   │   ├── vl53l1x/
│   │   ├── uart_console/
│   │   └── led/
│   ├── platform/
│   │   └── platform_esp32s3.c/.h
├── src/
│   ├── main.c
│   ├── functions.c/.h
│   └── tasks.c/.h
├── docs/
│   └── task_diagram.png
├── Makefile
├── README.md
```


## Dependencies

* ESP-IDF framework
* FreeRTOS (included by default in ESP-IDF)
* Sensor driver libraries (integrated into `components/drivers/`)
* HAL files (`platform/`) for porting to other microcontrollers via I2C, UART, ADC abstraction

## Key Algorithms

* **Kalman Filter** for sensor fusion:

  * Two stages: `predict()` and `update()` functions
  * State-space model: uses identified discrete-time matrices
  * Uses sensor data from encoder, lidar, and IMU
  * Inverse matrix computation is done using **LU Decomposition** (matrix factorization method for efficient inversion)

* **Incremental PID Control** for velocity and position:

  * The control system computes changes in the actuator signal rather than the absolute value, improving numerical stability on embedded systems.

---

## Getting Started

### Requirements

- ESP-IDF toolchain
- CMake
- Python 3.x for ESP-IDF tools
- Drivers for AS5600, VL53L1X, BNO055

### Build and Flash

```bash
git clone https://github.com/MaverickST/RoboCup_Striker2.git
cd RoboCup_Striker2
idf.py set-target esp32s3
idf.py build
idf.py flash
idf.py monitor
```

> ⚠️ Ensure the correct serial port is selected and the MCU is in bootloader mode.

---

## License

This project inherits its license from the root repository. See the LICENSE file in the root of the repo.

---

© 2025 Striker2 
