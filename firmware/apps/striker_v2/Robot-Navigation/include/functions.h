/**
 * @file functions.h
 * @author Striker2
 * @brief Header file for the functions used in the project
 * @details This file contains the declarations of the functions used in the project.
 *          It includes the functions to handle the UART events, trigger the flow of the tasks,
 * @version 0.1
 * @date 2025-05-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "types.h"
#include "senfusion.h"

/**
 * @brief Initialize drivers like: LED, UART, BLDC, AS5600, BNO055, VL53L1X
 * 
 */
void init_drivers(void);

/**
 * @brief Initialize and setup the AS5600 sensor.
 * Also runs an individual check to verify the sensor.
 * 
 * @param num_checks Number of checks to verify the sensor.
 */
bool setup_as5600(uint32_t num_checks);

/**
 * @brief Initialize and setup the VL53L1X sensor
 * Also runs an individual check to verify the sensor.
 * 
 * @param num_checks Number of checks to verify the sensor.
 */
bool setup_bno055(uint32_t num_checks);

/**
 * @brief Verify and check if all the sensors are ready to be used.
 * 
 * @param num_checks 
 * @return true 
 * @return false 
 */
bool verify_sensors(uint32_t num_checks);

/**
 * @brief Initialize the system variables
 * 
 */
void init_system(void);

/**
 * @brief Callback for the system one-shot timer
 * 
 * @param arg 
 */
void sys_timer_cb(void *arg);

/**
 * @brief Perform motor identification for all motors.
 * 
 * This function will iterate through all motors and perform the identification process.
 * It will also handle the AS5600 sensors associated with each motor.
 */
void motor_identification_all();

/**
 * @brief Callback function for motor identification.
 * 
 * @param arg 
 */
void motor_bldc_cb(void *arg);

/**
 * @brief Check if the drivers and sensors are ready to be used.
 * 
 * @return true 
 * @return false 
 */
bool is_drivers_ready(void);

/**
 * @brief Stop the robot by setting the duty cycle of the BLDC motors to 0.
 * 
 */
void stop_robot(void);

/**
 * @brief Calculate the angular speed of a motor based on the encoder readings.
 * 
 * @param kf Pointer to the Kalman filter structure for the motor.
 * @param midx Index of the motor (0, 1, or 2).
 * @return float 
 */
float calculate_motor_speed(kalman1D_t *kf, int midx);

/**
 * @brief Parses a JSON-like string and updates the velocity setpoints for the motors.
 * It is a command send by xIMU3 GUI to update the setpoints.
 * 
 * @param uart_buffer 
 */
void parse_and_update_setpoints(const char *uart_buffer);

/**
 * @brief Wrapper function for printf to handle formatted output.
 * 
 * @param format 
 * @param ... 
 */
void wrap_printf(const char *format, ...);

#endif // FUNCTIONS_H