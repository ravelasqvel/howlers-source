/**
 * @file tasks.h
 * @author Striker2
 * @brief  Header file for the tasks used in the project
 * @details This file contains the declarations of the tasks used in the project.
 * @version 0.1
 * @date 2025-05-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef TASKS_H
#define TASKS_H

#include "types.h"

/**
 * @brief Creates the tasks used in the project
 * 
 */
void create_tasks(void);

/**
 * @brief Task to trigger the flow of the tasks
 * 
 * @param pvParameters 
 */
void trigger_task(void *pvParameters);

/**
 * @brief Task to manage the BNO055 sensor
 * 
 * @param pvParameters 
 */
void bno055_task(void *pvParameters);

/**
 * @brief Task to manage the VL53L1X sensor
 * 
 * @param pvParameters 
 */
void vl53l1x_task(void *pvParameters);

/**
 * @brief Task to manage the AS5600 sensor
 * 
 * @param pvParameters 
 */
void as5600_task(void *pvParameters);

/**
 * @brief Task to control the BLDC motor
 * 
 * @param pvParameters 
 */
void control_task(void *pvParameters);

/**
 * @brief Task to save the data collected by the sensors
 * 
 * @param pvParameters 
 */
void save_data_task(void *pvParameters);

/**
 * @brief Task to handle the UART events
 * 
 * @param pvParameters 
 */
void uart_event_task(void *pvParameters);


#endif // TASKS_H