# 🤖 Robot-Control

This module implements the **low-level velocity control** for the Omniwheel robot.  
The system receives motion commands via a **UDP server**, processes them, and controls each wheel independently using **PI controllers** designed from identified motor models.

---

## 🧩 Overview

The robot can execute several types of movements commanded from the host computer:
- **Linear motion** – Forward or backward with specified velocity and distance.  
- **Circular motion** – Rotation around a defined radius and angular speed.  
- **In-place rotation** – Spin around its own axis by a given angle.

Each command is sent through the UDP communication interface located in [`udp_server`](../udp_server).

---

## 🧠 System Structure

The control logic is divided between the **UDP command server** (on PC) and the **embedded control system** (on ESP32-S3).  
Commands are parsed, transformed into trajectory parameters, and then translated into individual wheel speed setpoints.

<p align="center">
  <img src="./docs/SW block diagram.jpg" alt="Software Architecture Diagram" width="500"/>
</p>

---

## 🧰 Hardware Setup

The system is built around the **ESP32-S3** microcontroller and the following components:

| Component | Description |
|------------|-------------|
| **Microcontroller** | ESP32-S3 (FreeRTOS-based) |
| **Motors** | 3x BLDC motors with Hall sensors |
| **Encoders** | AS5600 magnetic encoders |
| **Power source** | 6S Li-Po battery |
| **Communication** | UDP server + Wi-Fi interface |

<p align="center">
  <img src="./docs/HW Block Diagram.jpg" alt="Hardware Architecture Diagram" width="500"/>
</p>

---

## 🧵 Task Management

The ESP32-S3 runs a **FreeRTOS** scheduler with three main tasks:  
- **WiFi Command Task:** receives UDP velocity commands.  
- **BLDC Control Task:** executes motor control every 1 ms (timer-triggered).  
- **UART Task:** handles telemetry output.  

A periodic timer releases the control loop via semaphore, ensuring deterministic updates.

<p align="center">
  <img src="./docs/Task Diagram.jpg" alt="Task Diagram" width="550">
</p>

---

## ⚙️ Control Architecture

- **Sensors used:** Wheel encoders only.  
- **Estimation:** Encoder-derived velocities are filtered using a **Kalman filter** to reduce noise.  
- **Model identification:** Each DC motor was modeled as a **first-order system** to obtain parameters for controller tuning.  
- **Controller:** A **PI (Proportional–Integral)** controller was implemented for precise wheel speed tracking.  
- **Kinematics:** Inverse kinematics are applied to convert desired robot body velocities into individual wheel targets.

<p align="center">
  <img src="./docs/Control Block Diagram.jpg" alt="Control Architecture Diagram" width="500"/>
</p>

---

## 🧠 Relation to Higher-Level Modules

This module serves as the **foundation for [Robot-Navigation](../Robot-Navigation)**,  
where higher-level behaviors (trajectory tracking, obstacle avoidance, and sensor fusion) are built on top of this control layer.

---

## 🚀 How to Run

```bash
# Open project directory
cd Robot-Control

# Build and deploy to the microcontroller
make all
# or compile from your preferred IDE

# Start UDP server on PC
cd ../udp_server
python3 udp_server.py
```

Commands can then be sent to control the robot’s motion.

---

## 🧪 Validation

- Step response tests were used to validate the PI controller performance.
- Encoder velocity filtering was verified against noisy raw signals.
- Command tracking was analyzed for linear, circular, and rotational motions.

---

## License

This project inherits its license from the root repository. See the LICENSE file in the root of the repo.

---

© 2025 Striker2 
