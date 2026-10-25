# Howlers SSL Suite â€” Unified Robotics Monorepo (Universidad de Antioquia)

Repositorio unificado del equipo **Howlers FC (RoboCup Small-Size League)** que integra:
1. **irmware/components/**: Drivers HAL en C (AS5600, BNO055, TM151, VL53L1X, VL6180X, APDS9960), cinemÃ¡tica holonÃ³mica, control PID y filtros probabilÃ­sticos (EKF, UKF, PF) para ESP32-S3 (ESP-IDF).
2. **irmware/apps/**: Aplicaciones para **Striker 1**, **Striker 2**, banco de pruebas de **PID Autotuning** y **State Estimation**.
3. **software/**: IdentificaciÃ³n de sistemas, estimaciÃ³n de estado en Python, sintonizaciÃ³n automÃ¡tica de PID con redes RNN-LSTM, tracking por visiÃ³n y estaciÃ³n base UDP/Web (ground_station).
4. **datasets/**: Registros experimentales de motores, telemetrÃ­a e identificaciÃ³n dinÃ¡mica.
