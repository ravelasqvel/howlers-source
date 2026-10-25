#include "functions.h"

void init_drivers(void)
{
    ///< ---------------- CNTROL + SENFUSION ----------------
    pid_block_t config_pid = {
        .Kp = 0.8, ///< Proportional gain
        .Kd = 0.0012, ///< Derivative gain
        .Ki = 137, ///< Integral gain
        .max_output   = 60,
        .min_output   = -60,
        .max_integral = 200,
        .min_integral = -200,
    };

    ctrl_senfusion_init(&gCtrl, config_pid, SAMPLING_PERIOD_S); // 10ms sampling time

    ///< ---------------------- UART ---------------------
    uconsole_init(&gUc, UART_NUM);

    ///< ---------------------- BLDC ---------------------
    bldc_init(&gMotor, MOTOR_MCPWM_GPIO, MOTOR_REVERSE_GPIO, MOTOR_MCPWM_FREQ_HZ, 0, 
                MOTOR_MCPWM_TIMER_RESOLUTION_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY);
    bldc_enable(&gMotor);
    // bldc_calibrate(&gMotor, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY); ///< Calibrate the BLDC motor
    bldc_set_duty(&gMotor, MOTOR_PWM_BOTTOM_DUTY); 

}

bool setup_as5600(uint32_t num_checks)
{
    ///< Initialize the AS5600 sensor
    ESP_LOGI(TAG_AS5600_TASK, "Initializing AS5600 sensor\n");
    AS5600_Init(&gAS5600, AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    ///< Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis off
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 16x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Slow filter only
        .WD = AS5600_WATCHDOG_ON, ///< Watchdog on
    };
    gAS5600.conf = conf;

    while (!AS5600_InitADC(&gAS5600)) {
        ESP_LOGE("setup_as5600", "AS5600 ADC initialization failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    }
    gSys.angle_origin_offset = AS5600_ADC_GetAngle(&gAS5600); ///< Get the angle in degrees
    gSys.is_as5600_calibrated = true;

    return true;
}

bool setup_bno055(uint32_t num_checks)
{
    ///< Initialize BNO055 sensor
    ESP_LOGI(TAG_BNO055_TASK, "Initializing BNO055 sensor\n");
    int8_t success = BNO055_Init(&gBNO055, BNO055_I2C_MASTER_SDA_GPIO, BNO055_I2C_MASTER_SCL_GPIO, BNO055_I2C_MASTER_NUM, BNO055_RST_GPIO);
    while (success != BNO055_SUCCESS) {
        printf("Error: Failed to initialize BNO055 sensor\n");
        BNO055_Reset(&gBNO055);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_Init(&gBNO055, BNO055_I2C_MASTER_SDA_GPIO, BNO055_I2C_MASTER_SCL_GPIO, BNO055_I2C_MASTER_NUM, BNO055_RST_GPIO);
    }
    
    ///< Load Calibration Data
    BNO055_SetOperationMode(&gBNO055, CONFIGMODE);
    uint8_t calib_offsets[22] = {
        0xF7, 0xFF, 0xCC, 0xFF, 0xC5, 0xFF, 
        0x8A, 0x01, 0x4E, 0x01, 0x5D, 0x00,
        0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
        0xE8, 0x03, 0xAD, 0x01   
    };
    BN055_Write(&gBNO055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, calib_offsets, 22);
    BNO055_SetOperationMode(&gBNO055, IMU);
    gSys.is_bno055_calibrated = true;

    ///< Individual sensor checks
    float acce = 0, acce_prev = 0;
    uint16_t cnt = 0;
    for (uint32_t i = 0; i < num_checks; i++) {
        BNO055_ReadAll_Lineal(&gBNO055);
        acce = sqrt(gBNO055.ax*gBNO055.ax + gBNO055.ay*gBNO055.ay);
        if (fabs(acce - acce_prev) < 0.0001) {
            cnt++;
        }
        acce_prev = acce;
        // printf("Acce: %.2f m^2\n", acce);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI("setup_bno055", "Verifing BNO055 sensor: err %d - num %d", (int)cnt, (int)num_checks);
    if (cnt > num_checks/2) {
        return false;
    }

    return true;
}

bool setup_vl53l1x(uint32_t num_checks)
{
    ///< Initialize the VL53L1X sensor and set the parameters
    ESP_LOGI(TAG_VL53L1X_TASK, "Initializing VL53L1X sensor\n");
    while (!VL53L1X_init(&gVL53L1X, VL53L1X_I2C_MASTER_NUM, VL53L1X_I2C_MASTER_SCL_GPIO, VL53L1X_I2C_MASTER_SDA_GPIO, false)) {
        ESP_LOGE(TAG_VL53L1X, "VL53L1X initialization failed\n");
        i2c_deinit(&gVL53L1X.i2c_handle); ///< Deinitialize the I2C handle
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    }

    ///< Get the distance respect to the origin
    VL53L1X_startContinuous(&gVL53L1X,10);
    while (true) {
        if (VL53L1X_dataReady(&gVL53L1X)) {
            gSys.dist_origin_offset = VL53L1X_readDistance(&gVL53L1X, false)/1000.0f; ///< Get the distance in meters
            break;
        }
    }
    gSys.is_vl53l1x_calibrated = true;

    return true;
}

bool verify_sensors(uint32_t num_checks)
{
    ///< Count the number of times the sensors are sensing the same value
    int cnt_as5600 = 0;
    int cnt_vl53l1x = 0;
    int cnt_bno055 = 0;

    float angle_prev = 0;
    float dist_prev = 0;
    float acce_prev = 0;

    float angle = 0;
    float dist = 0;
    float acce = 0;

    for (uint8_t i = 0; i < num_checks; i++) {
        ///< Get sensor values
        angle = AS5600_ADC_GetAngle(&gAS5600); ///< Get the angle in degrees

        if (VL53L1X_dataReady(&gVL53L1X)){            
            dist = VL53L1X_readDistance(&gVL53L1X, false); // false for non-blocking read
        }

        BNO055_ReadAll_Lineal(&gBNO055);
        acce = sqrt(gBNO055.ax*gBNO055.ax + gBNO055.ay*gBNO055.ay);

        ///< Print the values
        printf("Angle: %.2f deg, Distance: %.3f m, Acce: %.2f m^2\n", angle, dist/1000.0, acce);

        ///< Check if the values are the same
        if (fabs(angle - angle_prev) < 0.001) {
            cnt_as5600++;
        }
        if (fabs(dist - dist_prev) < 0.001) {
            cnt_vl53l1x++;
        }
        if (fabs(acce - acce_prev) < 0.001) {
            cnt_bno055++;
        }

        angle_prev = angle;
        dist_prev = dist;
        acce_prev = acce;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI("verify_sensors", "AS5600: %d, VL53L1X: %d, BNO055: %d", cnt_as5600, cnt_vl53l1x, cnt_bno055);

    if (cnt_as5600 > num_checks/2 && cnt_vl53l1x > num_checks/2 && cnt_bno055 > num_checks/2) {
        return false;
    }

    return true;
}

void init_system(void)
{
    ///< Initialize the system variables
    gSys.cnt_sample = 0; ///< Initialize the number of samples readed from all the sensors
    gSys.STATE = INIT_BLDC_STEP_1; ///< Initialize the state machine: initialize the BLDC motor

    // Create a one-shot timer to control the sequence
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &sys_timer_cb,
        .arg = NULL, ////< argument specified here will be passed to timer callback function
        .name = "sys-one-shot" ///< name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t oneshot_timer_s;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer_s));
    gSys.oneshot_timer = oneshot_timer_s;
    ESP_ERROR_CHECK(esp_timer_start_once(gSys.oneshot_timer, NONE_TO_STEPS_US));

}

void sys_timer_cb(void *arg)
{
    switch(gSys.STATE)
    {
        case INIT_BLDC_STEP_1:
            ESP_LOGI("sys_timer_cb", "INIT_PWM_BLDC_STEP_1");
            bldc_set_duty(&gMotor, 0); ///< Set the duty to the top position for the ESC

            ///< Use the time for the sensor sampling and control the BLDC motor
            gSys.STATE = CHECK_SENSORS;
            gSys.is_bldc_calibrated = true; ///< Set the flag to true to indicate that the BLDC motor is calibrated

            ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.oneshot_timer, TIME_SAMPLING_US));
            break;

        case CHECK_SENSORS:
            // Check if the sensors are calibrated
            if (gSys.is_as5600_calibrated && gSys.is_bno055_calibrated && gSys.is_vl53l1x_calibrated && gSys.is_bldc_calibrated) {
                // gSys.STATE = SYS_SAMPLING_EXP; ///< Set the state to sampling
                gSys.STATE = SYS_SAMPLING_EXP; ///< Set the state to sampling
                // ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                ESP_LOGI("sys_timer_cb", "Sensors calibrated. Starting the system.");
            }
            break;

        case SYS_SAMPLING_EXP:
            BaseType_t mustYield = pdFALSE;
            vTaskNotifyGiveFromISR(gSys.task_handle_trigger, &mustYield);

            // portYIELD_FROM_ISR(mustYield); ///< Yield the task to allow the other tasks to run
            gSys.cnt_sample++; ///< Increment the number of samples readed from all the sensors
            if (gSys.cnt_sample >= NUM_SAMPLES) {
                ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                // ESP_ERROR_CHECK(esp_timer_delete(gSys.oneshot_timer)); ///< Delete the timer to stop the sampling
                bldc_set_duty(&gMotor, 0); ///< Stop the motor
                ESP_LOGI("sys_timer_cb", "Samples readed from all the sensors: %d", gSys.cnt_sample);
                gSys.STATE = NONE;
            }

            break;

        case SYS_SAMPLING_CONTROL:
            mustYield = pdFALSE;
            vTaskNotifyGiveFromISR(gSys.task_handle_trigger, &mustYield);
            // portYIELD_FROM_ISR(mustYield); ///< Yield the task to allow the other tasks to run

            gSys.cnt_smp_control++; ///< Increment the number of samples readed from all the sensors for control
            gSys.cnt_sample++; ///< Increment the number of samples readed from all the sensors
            if (gSys.cnt_smp_control >= NUM_SAMPLES_CONTROL) {
                ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                // ESP_ERROR_CHECK(esp_timer_delete(gSys.oneshot_timer)); ///< Delete the timer to stop the sampling
                bldc_set_duty(&gMotor, 0); ///< Stop the motor
                ESP_LOGI("sys_timer_cb", "Samples readed from all the sensors: %d", gSys.cnt_smp_control);
                gSys.STATE = NONE; ///< Set the state to NONE to stop the system
            }
            break;

        default:
            ESP_LOGI("sys_timer_cb", "default");
            break;
    }
}

void process_cmd(const char *cmd)
{
    ///< Check if the data is valid
    uint8_t len_uc_data = strlen((const char *)gUc.data);
    if (len_uc_data < 5) { ///< If the data is less than 5 characters, that means the data is not valid
        ESP_LOGI(TAG_CMD, "Invalid CMD");
        return;
    }
    ///< Command to set the speed of the motor
    if (strcmp(cmd, "pwm") == 0) {
        char str_value[len_uc_data - 4]; ///< 4 is the length of the command "pwm "
        strncpy(str_value, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the value after the command

        int value = atoi(str_value)/10;
        ESP_LOGI(TAG_CMD, "value-> %d", (int)value);
        if (value != 0) { ///< If value=0, that means data is not a number
            bldc_set_duty(&gMotor, (int)value);
        }
    }
    ///< Command to read the angle from the AS5600 sensor
    else if(strcmp(cmd, "as5") == 0){
        ///< Check the minimum length of the command
        if (len_uc_data <= 8) { ///< 4 for the command "as5 " and 4 for the register
            ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
            return;
        }
        ///< From cmd, get if it is read or write: "as5 r" or "as5 w"
        char rw = *(gUc.data + 4);

        ///< Then get the register to read or write: "as5 r zmco", "as5 w zpos", ....
        uint8_t len_reg = 4;
        char reg[len_reg]; ///< 4 is the length of the register, +1 for the null terminator
        strncpy(reg, (const char *)gUc.data + strlen("as5 r "), len_reg); ///< Get the register after the command
        reg[len_reg] = '\0'; ///< Add the null terminator
        AS5600_reg_t addr = AS5600_RegStrToAddr(&gAS5600, reg); ///< Map the str to int address
        if (addr == -1) {
            ESP_LOGI(TAG_CMD, "Invalid register");
            return;
        }

        ///< Read or write the register
        uint16_t data;
        if (rw == 'r') {
            ESP_LOGI(TAG_CMD, "addr-> %02x", (uint8_t)addr);
            AS5600_ReadReg(&gAS5600, addr, &data);
            ESP_LOGI(TAG_CMD, "readed-> %04x", data);
        }
        ///< For write commands, it is necessary to get the value to write
        else if (rw == 'w') {
            ///< Check the minimum length of the command
            if (len_uc_data != 15) { ///< 15 for a command like "as5 w [regi] [0000]"
                ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
                return;
            }
            ///< Get the exadecimal value from 3 characters
            uint8_t len_value = 4;
            char str_value[len_value]; ///< value to write in chars
            strncpy(str_value, (const char *)gUc.data + strlen("as5 w regi "), len_value); ///< Get the value after the command
            str_value[len_value] = '\0'; ///< Add the null terminator
            uint16_t value = 0; ///< value to write in int
            for (int i = 0; i < len_value; i++) {
                if (str_value[i] >= '0' && str_value[i] <= '9') {
                    value += (str_value[i] - '0') << (4 * (len_value - i - 1));
                }
                else if (str_value[i] >= 'a' && str_value[i] <= 'f') {
                    value += (str_value[i] - 'a' + 10) << (4 * (len_value - i - 1));
                }
                else {
                    ESP_LOGI(TAG_CMD, "Invalid value");
                    return;
                }
            }
            AS5600_WriteReg(&gAS5600, addr, value);
            ESP_LOGI(TAG_CMD, "addr-> %02x, value-> %04x", (uint8_t)addr, value);
        }
        else if (rw == 'o') {
            printf("as5 o");
        }
        else {
            ESP_LOGI(TAG_CMD, "rw not recognized");
        }

    }
    ///< Command to turn on the LED
    else if(strcmp(cmd, "led") == 0) {
        if (len_uc_data < 7) { ///< 4 for the command "led " and 5 for the color
            ESP_LOGI(TAG_CMD, "Invalid LED cmd");
            return;
        }
        char str_color[len_uc_data - 4]; ///< 4 is the length of the command "led "
        strncpy(str_color, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the color after the command
        str_color[len_uc_data - 4] = '\0'; ///< Add the null terminator
        if (strcmp(str_color, "red") == 0) {
            led_setup_red(&gLed, LED_TIME_US);
        }
        else if (strcmp(str_color, "green") == 0) {
            led_setup_green(&gLed, LED_TIME_US);
        }
        else if (strcmp(str_color, "blue") == 0) {
            led_setup_blue(&gLed, LED_TIME_US);
        }
        else {
            ESP_LOGI(TAG_CMD, "color not recognized");
        }
    }
    ///< Control ommand to set the setpoint of the motor
    else if (strcmp(cmd, "set") == 0) {
        if (gSys.STATE != NONE) { ///< If the system is not in the NONE state, that means the system is busy
            ESP_LOGI(TAG_CMD, "System is busy");
            return;
        }
        if (len_uc_data < 8) { ///< 4 for the command "set " and 4 for the setpoint
            ESP_LOGI(TAG_CMD, "Invalid SET cmd");
            return;
        }
        char str_value[len_uc_data - 4]; ///< 4 is the length of the command "set "
        strncpy(str_value, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the value after the command
        str_value[len_uc_data - 4] = '\0'; ///< Add the null terminator

        bool dir = false; ///< Direction of the motor
        int dist = 0; ///< Distance to move (cm)
        int vel = 0; ///< Velocity to move (cm/s)

        parse_command_setpoint((const uint8_t *)str_value, &dir, &dist, &vel); ///< Parse the command to get the direction, distance and velocity
        ESP_LOGI(TAG_CMD, "dir-> %d, dist-> %d, vel-> %d", dir, dist, vel);

        gSys.setpoint_dir = dir; ///< Set the direction of the motor
        if (gSys.setpoint_dir) {
            gSys.setpoint_dist = (float)dist/100; ///< Set the distance to move (m)
            gSys.setpoint_vel = (float)vel/100; ///< Set the velocity to move (m/s)
        }
        else {
            gSys.setpoint_dist = -(float)dist/100; ///< Set the distance to move (m)
            gSys.setpoint_vel = -(float)vel/100; ///< Set the velocity to move (m/s)
        }
        gSys.setpoint_dist = gSys.setpoint_dist + gSys.dist_enc; ///< Add the distance from the encoder to the setpoint distance
        printf("Setpoint: dir %d, dist %.3f m, vel %.3f m/s\n", gSys.setpoint_dir, gSys.setpoint_dist, gSys.setpoint_vel);

        ///< Init the control experiment
        gSys.STATE = SYS_SAMPLING_CONTROL; ///< Set the state to control the BLDC motor
        gSys.duty = 0; ///< Set the duty to 0
        gSys.cnt_smp_control = 0; ///< Set the number of samples to 0
        ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.oneshot_timer, TIME_SAMPLING_US));
    }
    else {
        ESP_LOGI(TAG_CMD, "cmd not recognized");
    }
}


void parse_command_setpoint(const uint8_t *command, bool *dir, int *dist, int *vel) {
    ///< The first character indicates the direction
    if (command[0] == 'D') {
        *dir = true;  // right
    } else if (command[0] == 'I') {
        *dir = false; // left
    } else {
        printf("Invalid command: unknown direction\n");
        return;
    }

    ///< Find the position of the '_'
    const char *underscore = strchr((const char *)command, '_');
    if (!underscore) {
        printf("Invalid command: '_' not found\n");
        return;
    }

    ///< Extract distance: from command[1] up to the character before '_'
    char distance_str[16] = {0};
    strncpy(distance_str, (const char *)&command[1], underscore - (const char *)&command[1]);
    *dist = atoi(distance_str);

    ///< Extract speed: from the character after '_'
    *vel = atoi(underscore + 1);
}

