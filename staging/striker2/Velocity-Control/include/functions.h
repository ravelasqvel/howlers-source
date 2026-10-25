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
 * @brief Initialize and setup the BNO055 sensor
 * Also runs an individual check to verify the sensor.
 * 
 * @param num_checks Number of checks to verify the sensor.
 */
bool setup_vl53l1x(uint32_t num_checks);

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
 * @brief Proccess the command received from the UART console
 * 
 * @param cmd 
 */
void process_cmd(const char *cmd);

/**
 * @brief Parse the setpoint command received from the UART console.
 * 
 * @param command 
 * @param dir 
 * @param dist 
 * @param vel 
 */
void parse_command_setpoint(const uint8_t *command, bool *dir, int *dist, int *vel);

#endif // FUNCTIONS_H