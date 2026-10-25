/**
 * @file main.c
 * @author Striker2
 * @brief Main file for the project
 * @version 0.1
 * @date 2025-05-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "types.h"
#include "functions.h"
#include "tasks.h"

// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

led_rgb_t gLed;
bldc_pwm_motor_t gMotor;
system_t gSys;
ctrl_senfusion_t gCtrl;
uart_console_t gUc;

AS5600_t gAS5600;
vl53l1x_t gVL53L1X;
BNO055_t gBNO055;

void app_main(void)
{
    ///< Initialize the drivers: LED, UART, BLDC
    init_drivers(); 

    ///< Initialize and setup each sensor
    if (!setup_as5600(100)) { ///< Setup the AS5600 sensor
        ESP_LOGI("app_main", "AS5600 sensor is not ready");
        return;
    }
    while (!setup_bno055(100)) { ///< Setup the BNO055 sensor
        ESP_LOGI("app_main", "BNO055 sensor is not ready. Resetting...");
        BNO055_Reset(&gBNO055); ///< Reset the BNO055 sensor
        vTaskDelay(pdMS_TO_TICKS(1000)); ///< Wait 1 second before retrying
    }
    if (!setup_vl53l1x(100)) { ///< Setup the VL53L1X sensor
        ESP_LOGI("app_main", "VL53L1X sensor is not ready");
        return;
    }

    ///< Verify the sensors together
    if (verify_sensors(100)) { 
        ESP_LOGI("app_main", "Sensors are ready");
    }
    else {
        ESP_LOGI("app_main", "Sensors are not ready");
        return;
    }
    
    ///< Create the tasks
    create_tasks(); 

    ///< Initialize the system
    ///< 'System' refers to more general variables and functions that are used to control the project.
    init_system();

}
