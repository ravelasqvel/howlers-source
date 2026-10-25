# Striker 1
## Advanced Embedded Systems

The best course in Electronic Engineering has its own repository. Here you'll find a well-organized collection of sensor drivers and comprehensive programs developed throughout the course, focusing on autonomous robot control systems using the ESP32-S3 microcontroller.

This repository documents the complete development journey from individual sensor drivers to a fully integrated three-wheeled robot control system, showcasing embedded systems design, real-time control algorithms, and wireless communication protocols.

## Repository Information

**Original Repository**: [Link to original repo](https://github.com/jusanchez6/Sistemas_embebidos_avanzados)  
*This is a migrated copy to ensure project preservation and continued development. The original repository may continue to evolve independently.*

## Hardware Platform

All projects in this repository are built on the **ESP32-S3** microcontroller, leveraging its dual-core Xtensa LX7 processor, extensive peripheral support, and wireless capabilities. The development environment is based on ESP-IDF (Espressif IoT Development Framework), providing access to FreeRTOS for real-time task management.

---

# LAB 1 - [C libraries](./C%20libraries/)

The first laboratory focused on developing robust, production-ready drivers for all sensors required by the robot platform. This foundational work established the sensing capabilities necessary for autonomous navigation and motion control.

## Developed Sensor Drivers

### AS5600 - Magnetic Rotary Position Sensor
A 12-bit contactless magnetic encoder used for wheel position and velocity measurement. This driver provides:
- Angle measurement with 0.087° resolution
- Angular velocity calculation through differential measurements
- I2C communication interface (configurable address)
- Calibration routines for zero position setting
- Continuous rotation tracking with overflow handling

### VL53L1X - Long Range Time-of-Flight (ToF) Sensor
A laser-ranging module capable of measuring distances up to 4 meters with millimeter precision. The driver implements:
- Multiple ranging modes (short, medium, long distance)
- Configurable timing budgets for speed vs. accuracy tradeoffs
- Region of Interest (ROI) configuration for targeted measurements
- Interrupt-driven distance acquisition
- Temperature compensation and crosstalk calibration

### VL6180X - Short Range Time-of-Flight and Ambient Light Sensor
A proximity sensor with integrated ambient light sensing, optimized for distances up to 200mm. Features include:
- High-precision proximity detection
- Ambient light measurement in lux
- Configurable ranging and ALS integration periods
- Factory calibration data handling
- Low-power operation modes

### APDS9960 - RGB and Gesture Sensor
A multi-functional sensor combining RGB color detection, proximity sensing, and gesture recognition. The driver supports:
- 16-bit RGBC (Red, Green, Blue, Clear) color data acquisition
- Proximity detection with adjustable LED drive strength
- Gesture recognition for up/down/left/right movements
- Configurable gain and integration time
- Interrupt generation on color or proximity thresholds

### TM151 - 9-Axis Inertial Measurement Unit (IMU)
An advanced motion sensing platform integrating accelerometer, gyroscope, and magnetometer. The comprehensive driver provides:
- 3-axis accelerometer data (±2g to ±16g range)
- 3-axis gyroscope data (±250°/s to ±2000°/s range)
- 3-axis magnetometer for heading determination
- Digital Motion Processor (DMP) integration
- Sensor fusion algorithms for orientation estimation
- FIFO buffer management for burst reading
- Temperature compensation

## Architecture & Design Principles

### Hardware Abstraction Layer (HAL)
All drivers are built upon a well-defined HAL that abstracts platform-specific implementations:
- **I2C Communication**: Generic read/write functions that can be mapped to any I2C peripheral
- **Timing Functions**: Delay and timestamp utilities independent of the underlying timer implementation
- **GPIO Control**: Pin manipulation for interrupt handling and sensor enable signals
- **DMA Support**: Optional direct memory access for high-throughput sensor data

This architecture allows the drivers to be ported to different microcontroller platforms (STM32, Arduino, Raspberry Pi, etc.) by simply reimplementing the HAL layer while keeping the sensor logic unchanged.

### Code Quality Features
- Comprehensive error handling and status reporting
- Doxygen-documented API for all public functions
- Unit-testable design with clear separation of concerns
- Minimal memory footprint suitable for resource-constrained systems
- Thread-safe implementations for RTOS environments


---

# LAB 2 - [One-Wheel Movement](./wheel-movement/)

This laboratory established the foundation for closed-loop motion control by developing a complete firmware system for precise wheel velocity and position control. The project serves as a proof-of-concept for the control algorithms that would later be scaled to the full three-wheeled robot.

## Project Overview

The system integrates multiple sensors (AS5600 encoder, VL53L1X ToF sensor, and TM151 IMU) with a brushless DC motor controlled via an Electronic Speed Controller (ESC). The primary focus is on implementing robust PID-based velocity control and precise distance tracking using encoder feedback.

### Motor Control System

**Brushless DC Motor with ESC Interface**
- PWM signal generation (1000-2000µs pulse width) for ESC control
- Smooth acceleration/deceleration profiles to prevent mechanical shock
- Safety features including timeout protection and emergency stop
- Calibration routines for ESC initialization and endpoint configuration

### Control Algorithms

**Incremental PID Controller**
The velocity control system implements a discrete incremental PID algorithm:
- **Proportional Term**: Responds to current error magnitude
- **Integral Term**: Eliminates steady-state error and compensates for system bias
- **Derivative Term**: Provides damping to reduce overshoot and oscillations
- **Anti-Windup**: Prevents integral term saturation during setpoint changes
- Real-time tuning capability for adaptive control in different operating conditions

**Encoder-Based Odometry**
- High-resolution angle measurement (4096 steps per revolution)
- Velocity estimation through first-order differentiation with noise filtering
- Distance calculation using arc length integration
- Overflow detection for multi-revolution tracking
- Calibration compensation for mounting offsets

### Real-Time Operating System Architecture

The firmware is structured using FreeRTOS tasks for concurrent operation:
- **Control Task** (100Hz): Executes PID algorithm and updates motor commands
- **Sensor Task** (50Hz): Reads encoder and IMU data with synchronized timestamps
- **Communication Task**: Handles UART command parsing and response generation
- **Monitor Task** (10Hz): Logs system status and performance metrics

### UART Command Interface

An interactive command-line interface enables real-time system configuration and control:

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| **PID Update** | `P <Kp> <Ki> <Kd> <SP>` | Adjusts PID gains and setpoint online | `P 1.5 0.3 0.05 50.0` |
| **Direct PWM** | `X <pwm>` | Sets raw ESC duty cycle (1000-2000) | `X 1500` |
| **Setpoint** | `S <velocity>` | Changes target velocity (RPM) | `S 75.0` |
| **Right Move** | `D<dist>_<vel>` | Executes forward motion at specified distance/velocity | `D100_50.0` |
| **Left Move** | `I<dist>_<vel>` | Executes backward motion at specified distance/velocity | `I50_30.0` |

### Performance Characteristics

- **Control Loop Frequency**: 100 Hz (10ms period)
- **Velocity Accuracy**: ±2% of setpoint at steady state
- **Distance Precision**: ±5mm over 1 meter travel
- **Response Time**: <500ms to reach 90% of target velocity
- **Maximum Speed**: Configurable, typically 200 RPM

## Development Insights

This laboratory phase revealed several important considerations:
- The critical importance of encoder mounting alignment for accurate measurements
- The need for velocity filtering to handle quantization noise at low speeds
- ESC calibration sensitivity and its impact on low-speed performance
- The balance between control aggressiveness and mechanical wear

The VL53L1X and TM151 sensors, while integrated into the system, are currently reserved for future enhancements such as obstacle detection and inertial navigation.


---

# LAB 3 - [Robot Movement](./robot-mov/) (**In Active Development**)

The culminating project integrates all previous work into a complete autonomous three-wheeled robot system. This laboratory represents a significant leap in complexity, requiring coordination of multiple actuators, sensor fusion, and advanced control strategies for holonomic motion.

## System Architecture

### Three-Wheeled Holonomic Platform

The robot employs a triangular wheel configuration (120° spacing) enabling omnidirectional movement:
- Each wheel is independently driven by a brushless DC motor with ESC
- Each wheel has its own AS5600 magnetic encoder for precise feedback
- The geometry allows for linear motion in any direction and in-place rotation
- Forward kinematics convert wheel velocities to robot velocity (Vx, Vy, ω)
- Inverse kinematics decompose desired robot motion into individual wheel commands

### Multi-Motor Control System

**Independent PID Controllers**
Each wheel operates with its own tuned PID controller:
- Compensates for individual motor characteristics and mechanical variations
- Synchronized control loop execution across all three controllers
- Cross-coupling compensation to handle wheel interaction forces
- Dynamic load balancing for optimal traction distribution

**Velocity Decomposition Matrix**
The system uses a transformation matrix to convert between robot-frame and wheel-frame velocities:
```
[ω₁]   [cos(0°)    sin(0°)    L]   [Vx]
[ω₂] = [cos(120°)  sin(120°)  L] × [Vy]
[ω₃]   [cos(240°)  sin(240°)  L]   [ω ]
```
Where L is the distance from wheel to robot center, and ω₁, ω₂, ω₃ are individual wheel angular velocities.

### Wireless Control Interface

**WiFi-based Command System**
Transitioning from UART to WiFi enables remote operation and monitoring:
- **UDP Protocol**: Low-latency command transmission for real-time control
- **TCP Telemetry**: Reliable delivery of sensor data and system status
- **Web Interface**: Browser-based control dashboard (served from ESP32-S3)
- **Mobile App Integration**: Support for smartphone control applications
- **Python Proxy Server**: Development tool for command scripting and data logging

### Movement Primitives

The robot supports three fundamental motion types:

**1. Linear Displacement**
- Move in any direction (0-360°) relative to robot frame
- Specify target distance, velocity, and heading angle
- Straight-line trajectory tracking with PID correction
- Encoder-based distance estimation with sensor fusion

**2. In-Place Rotation**
- Clockwise and counter-clockwise rotation about robot center
- Specify target angle (degrees) and rotational velocity
- All three wheels contribute to rotation with equal contribution
- Gyroscope feedback from TM151 IMU for accurate angle tracking

**3. Circular Displacement**
- Follow circular arc with specified radius
- Combined linear and rotational motion
- Requires coordinated differential wheel velocities
- Path following accuracy enhanced by IMU data

### Advanced Features

**Sensor Fusion & Localization**
- Combines encoder odometry with IMU acceleration and gyroscope data
- Complementary filter implementation to merge sensor streams
- Reduces drift in long-distance autonomous navigation
- Real-time position estimation (X, Y, θ) in robot coordinate frame

**Obstacle Detection & Avoidance** *(Planned)*
- Integration of VL53L1X and VL6180X ToF sensors for 360° awareness
- Emergency stop on obstacle detection
- Dynamic path replanning around detected obstacles

**Trajectory Planning** *(In Development)*
- Quintic polynomial profiles for smooth acceleration/deceleration
- Jerk-limited motion for mechanical stress reduction
- Waypoint navigation with automatic path generation

### Communication Protocol

The WiFi interface accepts structured JSON commands:

**Linear Movement Command:**
```json
{
  "cmd": "linear",
  "distance": 1000,
  "velocity": 50.0,
  "angle": 45.0
}
```

**Rotation Command:**
```json
{
  "cmd": "rotate",
  "degrees": 90,
  "velocity": 30.0,
  "direction": "cw"
}
```

**Circular Motion Command:**
```json
{
  "cmd": "circular",
  "radius": 500,
  "degrees": 180,
  "velocity": 40.0,
  "direction": "ccw"
}
```

### Development Tools

**Python UDP Proxy** (`proxy_udp.py`)
A development utility that facilitates testing and debugging:
- Forwards commands from development PC to robot over WiFi
- Logs all transmitted commands and received responses
- Provides command replay for regression testing
- Enables automated testing sequences

**Web Dashboard** (`index.html`)
Browser-based monitoring and control interface:
- Real-time telemetry visualization
- Interactive joystick control
- System parameter adjustment
- Performance metric graphs

## Technical Challenges & Solutions

### Challenge 1: Wheel Synchronization
**Problem**: Slight variations in motor response caused skewed trajectories  
**Solution**: Implemented feedforward compensation based on measured motor characteristics and added cross-axis PID coupling

### Challenge 2: WiFi Latency
**Problem**: Variable network latency affected control responsiveness  
**Solution**: Adopted UDP for commands (accepting packet loss), implemented client-side prediction, and added command timestamping

### Challenge 3: Encoder Noise at Low Speeds
**Problem**: Quantization noise dominated measurements below 10 RPM  
**Solution**: Multi-sample averaging, Kalman filtering, and IMU data fusion for velocity estimation

### Challenge 4: Power Distribution
**Problem**: Uneven battery discharge caused motor performance drift  
**Solution**: Battery voltage monitoring and adaptive PWM scaling to maintain consistent performance

## Current Status & Future Work

**Completed:**
- ✅ Individual wheel control with PID
- ✅ Forward and inverse kinematics implementation
- ✅ Basic WiFi command interface
- ✅ Linear motion in cardinal directions
- ✅ In-place rotation
- ✅ Web dashboard feature completion

**Future:**
- 📋 Complete sensor fusion algorithm tuning
- 📋 Circular trajectory implementation refinement
- 📋 SLAM (Simultaneous Localization and Mapping) integration
- 📋 Autonomous navigation with obstacle avoidance
- 📋 Multi-robot coordination protocols
- 📋 Computer vision integration for target tracking

---

## Building & Flashing

All projects use the ESP-IDF build system:

```bash
# Configure the project
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32-S3
idf.py -p COMx flash monitor
```

## Contributing

This repository represents coursework for the Advanced Embedded Systems course. While it's primarily educational, contributions, bug reports, and suggestions are welcome through issues and pull requests.

## License

See [LICENSE](./LICENSE) file for details.

---

*Developed as part of the Advanced Embedded Systems course in Electronic Engineering*