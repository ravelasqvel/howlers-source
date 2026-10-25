/**
 * \file        types.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_partition.h"
#include "esp_flash.h"
#include "esp_timer.h"

#include "uart_console.h"
#include "functions.h"
#include "tasks.h"
#include "led.h"
#include "platform_esp32s3.h"
#include "bldc_pwm.h"
#include "as5600_lib.h"
#include "VL53L1X.h"
#include "bno055.h"
#include "control_senfusion.h"

// -------------------------------------------------------------------------- 
// ----------------------------- DEFINITIONS --------------------------------
// --------------------------------------------------------------------------

#define AS5600_I2C_MASTER_SCL_GPIO 4    /*!< gpio number for I2C master clock */
#define AS5600_I2C_MASTER_SDA_GPIO 5    /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6               /*!< gpio number for OUT signal */
#define AS5600_I2C_MASTER_NUM 1         /*!< I2C port number for master dev */

#define BNO055_I2C_MASTER_SCL_GPIO 11    /*!< gpio number for I2C master clock */
#define BNO055_I2C_MASTER_SDA_GPIO 12    /*!< gpio number for I2C master data  */
#define BNO055_I2C_MASTER_NUM 0         /*!< I2C port number for master dev */
#define BNO055_RST_GPIO 13           /*!< gpio number for I2C reset */


#define VL53L1X_I2C_MASTER_SCL_GPIO 38    /*!< gpio number for I2C master clock */
#define VL53L1X_I2C_MASTER_SDA_GPIO 39    /*!< gpio number for I2C master data  */
#define VL53L1X_I2C_MASTER_NUM 1         /*!< I2C port number for master dev */
#define VL53L1X_RST_GPIO 41              /*!< gpio number for I2C reset */

#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 1000*1000 // 1MHz, 1 tick = 1us
#define MOTOR_MCPWM_FREQ_HZ             50    // 50Hz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_MCPWM_GPIO                3
#define MOTOR_REVERSE_GPIO              8
#define MOTOR_PWM_BOTTOM_DUTY           50
#define MOTOR_PWM_TOP_DUTY              120

#define LED_TIME_US     2*1000*1000
#define LED_LSB_GPIO    15

#define UART_NUM        0

#define TIME_SAMPLING_S		10		/* 10s sampling data */
#define SAMPLING_RATE_HZ	100 	/* 10ms between each data saved */
#define SAMPLING_PERIOD_S        1.0/SAMPLING_RATE_HZ // 10ms
#define NUM_SAMPLES			TIME_SAMPLING_S*SAMPLING_RATE_HZ
#define TIME_SAMPLING_US    1e6/SAMPLING_RATE_HZ // 1ms
#define NUM_SAMPLES_CONTROL        5*SAMPLING_RATE_HZ /* 5s sampling data */

#define RAD_TO_DEG      57.2957795 // Conversion factor from radians to degrees
#define DEG2RAD         (3.14159265358979323846f / 180.0f)
#define RADIUS_M        0.03
#define MAX_BRIGHTNESS  5 // Max brightness of the LED

// --------------------------------------------------------------------------

/**
 * @brief System structure.
 * 
 * It will be used to store the data from the sensors and the state of the system.
 * 
 */
typedef struct
{
    enum 
    {
        NONE, ///< Nothing to do
        // To initialize the motor, it is necessary to send two PWM signals to the BLDC motor (ESC)
        INIT_BLDC_STEP_1, ///< The first PWM value

        SYS_SAMPLING_EXP, ///< The system is running the experiment for the indentification of the BLDC motor
        SYS_SAMPLING_CONTROL, ///< The system is sampling the data for a control command.
        CHECK_SENSORS, ///< Check if the sensors are calibrated
    } STATE;

    enum
    {
        NONE_TO_STEPS_US   = 3*1000*1000, ///< The time between the initial state and the first step is 100ms
        STEP1_TO_STEP2_US  = 3*1000*1000, ///< The time between step 1 and step 2 is 1s
        STEPS_TO_SEQ_US    = 5*1000*1000, ///< The time between the first two steps and the sequence is 5s
    } TIME;

    int cnt_sample; ///< Count the number of samples readed from all the sensors.
    int cnt_smp_control; ///< Count the number of samples readed from all the sensors for the control command.

    /**
     * @brief Buffer to save the data from sensors during 5s, so there are 5*SAMPLING_RATE_HZ samples, and
     * each sample has 4 values: duty, angle, acceleration, distance.
     * 
     * Each value is saved in a 5 bytes integer.
     * 
     */
    // int8_t buffer[1*SAMPLING_RATE_HZ][5*4];

    gpio_t rst_pin; ///< GPIO pin for the reset of the ESP32S3
    uint16_t raw_angle; ///< Raw angle readed from the AS5600 sensor
    QueueHandle_t queue; ///< Queue to send the data to the save task
    SemaphoreHandle_t mutex; ///< Mutex to protect the access to the global variables

    ///< Values for the sensors
    float distance; ///< Distance readed from the VL53L1X sensor
    float angle;    ///< Angle readed from the AS5600 sensor
    float dist_enc; ///< Distance readed from the AS5600 sensor
    float acceleration; ///< Acceleration readed from the BNO055 sensor
    float duty; ///< Duty cycle of the BLDC motor
    float dist_origin_offset; ///< Distance offset for the VL53L1X sensor
    float angle_origin_offset; ///< Angle offset for the AS5600 sensor

    ///< Flags to check if the sensors are calibrated or not
    bool is_as5600_calibrated;  ///< Flag to check if the AS5600 sensor is calibrated or not
    bool is_bno055_calibrated;  ///< Flag to check if the BNO055 sensor is calibrated or not
    bool is_vl53l1x_calibrated; ///< Flag to check if the VL53L1X sensor is calibrated or not
    bool is_bldc_calibrated;    ///< Flag to check if the BLDC motor is calibrated or not

    ///< Variables for the control of the BLDC motor
    bool setpoint_dir; ///< Direction of the BLDC motor
    float setpoint_dist; ///< Setpoint distance for the BLDC motor
    float setpoint_vel; ///< Setpoint velocity for the BLDC motor

    ///< Task handles for the tasks
    TaskHandle_t task_handle_bno055;    ///< Task handle for the BNO055 sensor
    TaskHandle_t task_handle_vl53l1x;   ///< Task handle for the VL53L1X sensor
    TaskHandle_t task_handle_as5600;    ///< Task handle for the AS5600 sensor
    TaskHandle_t task_handle_ctrl;      ///< Task handle for the control task
    TaskHandle_t task_handle_trigger;    ///< Task handle for the trigger task
    TaskHandle_t task_handle_save;      ///< Task handle for the save task

    esp_timer_handle_t oneshot_timer;    ///< Timer to control the sequence

}system_t;


// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

static const char* TAG_UART_TASK = "uart_task";
static const char* TAG_CMD = "cmd";
static const char* TAG_BNO055_TASK = "bno055_task";
static const char* TAG_VL53L1X_TASK = "vl53l1x_task";
static const char* TAG_AS5600_TASK = "as5600_task";
static const char* TAG_CTRL_TASK = "ctrl_task";

extern led_rgb_t gLed;
extern bldc_pwm_motor_t gMotor;
extern system_t gSys;
extern ctrl_senfusion_t gCtrl;
extern uart_console_t gUc;

extern AS5600_t gAS5600;
extern vl53l1x_t gVL53L1X;
extern BNO055_t gBNO055;


/**
 * @brief Flash
 * Some useful commands to read and write data to the flash memory via python console:
 *      - Erase partition with name 'storage'
 *              parttool.py --port "/dev/ttyUSB1" erase_partition --partition-name=storage
 *      - Read partition with type 'data' and subtype 'spiffs' and save to file 'spiffs.bin' 
 *              parttool.py --port "/dev/ttyUSB1" read_partition --partition-type=data --partition-subtype=spiffs --output "spiffs.bin"
 *      - Write to partition 'factory' the contents of a file named 'factory.bin'
 *              parttool.py --port "/dev/ttyUSB1" write_partition --partition-name=factory --input "factory.bin"
 *      - Print the size of default boot partition
 *              parttool.py --port "/dev/ttyUSB1" get_partition_info --partition-boot-default --info size
 *      - Print the size of the partition with name 'storage'
 *              parttool.py --port "/dev/ttyUSB1" get_partition_info --partition-name=storage --info size 
 * 
 * You just have to replace the port with the one you are using: "/dev/ttyUSB1" --> com5, com6, etc.
 * 
 * You may need to specify the path to the python script:
 *              python C:\Espressif\esp-idf-v5.2.2\components\partition_table\parttool.py --port com5.... 
 * 
 * python C:\Espressif\esp-idf-v5.2.2\components\partition_table\parttool.py --port com5 read_partition --partition-name=angle_pos --partition-subtype=nvs --output "angle.txt"
 * 
 * 
 */


#endif // __TYPES_H__