# "Howlers Source" Unified Robosyn Repo (Universidad de Antioquia)

Repositorio unificado del equipo **Howlers FC (RoboCup Small-Size League)** que integra:
1. **firmware/components/**: Drivers HAL en C (AS5600, BNO055, TM151, VL53L1X, VL6180X, APDS9960), cinemática holonómica, control PID y filtros probabilísticos (EKF, UKF, PF) para ESP32-S3 (ESP-IDF).
2. **firmware/apps/**: Aplicaciones para **Striker 1**, **Striker 2**, banco de pruebas de **PID Autotuning** y **State Estimation**.
3. **software/**: Identificación de sistemas, estimación de estado en Python, sintonización automática de PID con redes RNN-LSTM, seguimiento por visión y estación base UDP/Web (ground_station).
4. **datasets/**: Registros experimentales de motores, telemetría e identificación dinámica.
