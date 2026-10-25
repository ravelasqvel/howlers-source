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
#include "platform_esp32s3.h"
#include "platform_wifi_esp32s3.h"
#include "bldc_pwm.h"
#include "as5600_lib.h"
#include "VL53L1X.h"
#include "bno055.h"
#include "senfusion.h"
#include "control.h"

// -------------------------------------------------------------------------- 
// ----------------------------- DEFINITIONS --------------------------------
// --------------------------------------------------------------------------


///< WiFi definitions
#define WIFI_SSID "Howlers - UdeA" ///< WiFi SSID
#define WIFI_PASS "9876543210" ///< WiFi Password

// UDP server configuration
#define UDP_PORT 12345
#define UDP_RX_BUF_SIZE 128

///< AS5600 definitions
#define AS5600_ADC_UNIT_ID  ADC_UNIT_1 ///< ADC unit ID for AS5600 sensors
#define AS5600_I2C_MASTER_SCL_GPIO 4    /*!< gpio number for I2C master clock */
#define AS5600_I2C_MASTER_SDA_GPIO 5    /*!< gpio number for I2C master data  */
#define AS5600_I2C_MASTER_NUM 1         /*!< I2C port number for master dev */
#define AS5600_N_OUT_GPIO(x) ((x) == 0 ? 6 : \
                               (x) == 1 ? 7 : \
                               (x) == 2 ? 1 : 0)

///< BNO055 definitions
#define BNO055_I2C_MASTER_SCL_GPIO 11    /*!< gpio number for I2C master clock */
#define BNO055_I2C_MASTER_SDA_GPIO 12    /*!< gpio number for I2C master data  */
#define BNO055_I2C_MASTER_NUM 0         /*!< I2C port number for master dev */
#define BNO055_RST_GPIO 13           /*!< gpio number for I2C reset */

///< VL53L1X definitions
// #define VL53L1X_SENSOR_ADDR  0x29         /*!< slave address for sensor */
#define VL53L1X_I2C_MASTER_SCL_GPIO 38    /*!< gpio number for I2C master clock */
#define VL53L1X_I2C_MASTER_SDA_GPIO 39    /*!< gpio number for I2C master data  */
#define VL53L1X_RST_GPIO 41              /*!< gpio number for I2C reset */
#define VL53L1X_I2C_MASTER_NUM 1         /*!< I2C port number for master dev */

///< BLDC motor definitions
#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 1000*1000 // 1MHz, 1 tick = 1us
#define MOTOR_MCPWM_FREQ_HZ             50    // 50Hz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_PWM_BOTTOM_DUTY           50
#define MOTOR_PWM_TOP_DUTY              120

#define MOTOR_N_FORWARD_DIR(x)  ((x) == 0 ? 1 : \
                                 (x) == 1 ? 1 : \
                                 (x) == 2 ? 1 : 0)
                                 
#define MOTOR_N_MCPWM_GPIO(x)   ((x) == 0 ? 17 : \
                                 (x) == 1 ? 8 : \
                                 (x) == 2 ? 46 : 0)

#define MOTOR_N_MCPWM_REVERSE_GPIO(x) ((x) == 0 ? 18 : \
                                       (x) == 1 ? 3 : \
                                       (x) == 2 ? 9 : 0)

///< System timing definitions
#define TIME_SAMPLING_S		10		/* 10s sampling data */
#define SAMPLING_RATE_HZ	100 	/* 10ms between each data saved */
#define SAMPLING_PERIOD_S        1.0/SAMPLING_RATE_HZ // 10ms
#define NUM_SAMPLES			TIME_SAMPLING_S*SAMPLING_RATE_HZ
#define TIME_SAMPLING_US    1e6/SAMPLING_RATE_HZ // 1ms
#define NUM_SAMPLES_CONTROL        5*SAMPLING_RATE_HZ /* 5s sampling data */

///< Motor control timing definitions
#define TIME_SAMP_MOTOR_S   5  // 10s sampling data
#define SAMP_RATE_MOTOR_HZ  1000 // 1kHz sampling rate
#define SAMP_PERIOD_MOTOR_S 1.0/SAMP_RATE_MOTOR_HZ // 1ms
#define TIME_SAMP_MOTOR_US  1e6/SAMP_RATE_MOTOR_HZ // 1ms
#define NUM_SAMP_MOTOR      TIME_SAMP_MOTOR_S*SAMP_RATE_MOTOR_HZ // 10s sampling data

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define CM_TO_M(cm)     ((cm) / 100.0f)
#define RADIUS_M        0.03
#define UART_NUM     0 // UART number for the console

///< JSON definitions for the xIMU3 GUI
#define MAX_JSON_LEN 64
#define MAX_PARSED_LEN 32
// #define I2C_MASTER_FREQ_HZ  400*1000    /*!< I2C master clock frequency */

///< Sensor fusion definitions
#define KALMAN_1D_ENC_Q 0.001f // Process noise covariance for enconders
#define KALMAN_1D_ENC_R 1.12 // Measurement noise covariance for encoders

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

        CTRL_BLDC_C1, ///< Control the BLDC motor with the first command: straight linear movement
        CTRL_BLDC_C2, ///< Control the BLDC motor with the second command: rotation
        CTRL_BLDC_C3, ///< Control the BLDC motor with the third command: circular movement
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
    SemaphoreHandle_t mtx_printf; ///< Mutex to protect the access to the printf function
    SemaphoreHandle_t mtx_cntrl; ///< Mutex to protect the access to the control variables
    SemaphoreHandle_t mtx_traj; ///< Mutex to protect the access to the trajectory variables
    SemaphoreHandle_t smph_bldc; ///< Semaphore to synchronize the motor identification task

    ///< Variables for the control of each BLDC motor: setpoint, direction, distance, velocity and time stamp


    ///< Task handles for the tasks
    TaskHandle_t task_handle_bno055;    ///< Task handle for the BNO055 sensor
    TaskHandle_t task_handle_vl53l1x;   ///< Task handle for the VL53L1X sensor
    TaskHandle_t task_handle_bldc_ctrl;      ///< Task handle for the control task
    TaskHandle_t task_handle_robot_ctrl;      ///< Task handle for the robot control task
    TaskHandle_t task_handle_trigger;    ///< Task handle for the trigger task
    TaskHandle_t task_handle_save;      ///< Task handle for the save task

    esp_timer_handle_t oneshot_timer;    ///< Timer to control the system
    esp_timer_handle_t timer_bldc;       ///< Timer to control the BLDC motor

}system_t;


typedef struct 
{
    enum 
    {
        C1_STRAIGHT, ///< First command -> straight linear: angle, speed, distance
        C2_ROTATION, ///< Second command -> rotation: angle, speed
        C3_CIRCULAR, ///< Third command -> circular movement: angle, speed, radius, direction
        C_NONE, ///< No command
    }cmd;

    uint32_t ktime; ///< Actual instant
    uint32_t k_duration; ///< Duration of the command in milliseconds
    float omega_sp[3]; ///< Setpoint for the angular velocity of each motor
    float vbx_sp, vby_sp, wb_sp; ///< Setpoint for the robot body

    ///< Variables needed for each command
    float angle; ///< Angle of the robot in radians
    float distance; ///< Distance to move in meters
    float radius; ///< Radius of the circular movement in meters
    float speed; ///< Speed of the robot in m/s
    float omega; ///< Angular velocity of the robot in rad/s
    bool dir; ///< false: Counter-clockwise, true: Clockwise

}trajectory_t;


// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

static const char* TAG_UART_TASK = "uart_task";
static const char* TAG_CMD = "cmd";
static const char* TAG_BNO055_TASK = "bno055_task";
static const char* TAG_VL53L1X_TASK = "vl53l1x_task";
static const char* TAG_AS5600_TASK = "as5600_task";
static const char* TAG_CTRL_TASK = "ctrl_task";
static const char* TAG_UDP_SERVER = "udp_server";


extern bldc_pwm_motor_t gMotor[3]; ///< Array of BLDC motors
extern control_t gCtrl[3]; ///< Array of control structures
extern system_t gSys;
extern senfusion_t gSenFusion; ///< Sensor fusion structure
extern uart_console_t gUc;
extern trajectory_t gTraj; ///< Trajectory structure

extern AS5600_t gAS5600[3]; ///< Array of AS5600 sensors
extern vl53l1x_t gVL53L1X[3]; ///< Array of VL53L1X sensors
extern BNO055_t gBNO055;

extern esp_ip4_addr_t gIpAddr;
#endif // __TYPES_H__