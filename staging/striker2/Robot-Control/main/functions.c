#include "functions.h"

void init_drivers(void)
{
    ///< ---------------- CNTROL + SENFUSION ----------------
    pid_block_t config_pid = {
        .Kp = 2.1167, ///< Proportional gain
        .Ki = 25.0639, ///< Integral gain
        .Kd = 0, ///< Derivative gain
        .max_output   = 60,
        .min_output   = -60,
        .max_integral = 200,
        .min_integral = -200,
        .a1 = 1.53, .a0 = -1.5, .b1 = 1, .b0 = -1,
        .order = 1,
    };

    senfusion_init(&gSenFusion, SAMPLING_PERIOD_S); // 10ms sampling time
    for (int i = 0; i < 3; i++) {
        control_init(&gCtrl[i], SAMPLING_PERIOD_S/10, config_pid); ///< Initialize the control structure
    }

    ///< ---------------------- UART ---------------------
    uconsole_init(&gUc, UART_NUM);

    ///< ---------------------- BLDC ---------------------
    ///< Initialize the BLDC motors
    for (int i = 0; i < 3; i++) {
        bldc_init(&gMotor[i], MOTOR_N_MCPWM_GPIO(i), MOTOR_N_MCPWM_REVERSE_GPIO(i), MOTOR_MCPWM_FREQ_HZ, 1, 
            MOTOR_MCPWM_TIMER_RESOLUTION_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY);

        bldc_enable(&gMotor[i]);
        // bldc_calibrate(&gMotor[i], MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY); ///< Calibrate the BLDC motor
        bldc_set_duty_motor(&gMotor[i], 0); ///< Set the initial duty cycle to 0%
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI("drivers", "Drivers initialized successfully");

}

bool setup_as5600(uint32_t num_checks)
{
    ///< Initialize the AS5600 sensor
    ESP_LOGI(TAG_AS5600_TASK, "Initializing AS5600 sensors \n");

    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };

    // ///< Calibrate the AS5600 sensors
    // for (int i = 0; i < 1; i++) {
    //     AS5600_Init(&gAS5600[i], AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_N_OUT_GPIO(i));
    //     AS5600_Calibrate(&gAS5600[i], conf, 0x000, 0xFFF); ///< Calibrate the AS5600 sensor with the configuration
    //     if (!gAS5600[i].is_calibrated) {
    //         wrap_printf("AS5600 sensor %d calibration failed\n", i);
    //         return false; ///< Return false if calibration fails
    //     }
    // }

    ///< Initialize the ADC for the AS5600 sensors
    adc_oneshot_unit_handle_t handle;
    if (!adc_create_unit(&handle, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE(TAG_AS5600_TASK, "AS5600 ADC initialization failed\n");
        return false;
    }
    for (int i = 0; i < 3; i++) {
        gAS5600[i].conf = conf; ///< Set the configuration for the AS5600 sensor
        gAS5600[i].adc_handle.adc_handle = handle; ///< Assign the ADC handle to the AS5600 sensor
        if (!adc_config_channel(&gAS5600[i].adc_handle, AS5600_N_OUT_GPIO(i), AS5600_ADC_UNIT_ID)) {
            ESP_LOGE(TAG_AS5600_TASK, "AS5600 sensor %d ADC initialization failed\n", i);
            continue;
        }
        gAS5600[i].is_calibrated = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
        gAS5600[i].angle_offset = AS5600_ADC_GetAngle(&gAS5600[i]);
        wrap_printf("AS5600 sensor %d initialized successfully with angle offset: %.2f\n", i, gAS5600[i].angle_offset);
    }

    return true;
}

bool setup_bno055(uint32_t num_checks)
{
    ///< Initialize BNO055 sensor
    ESP_LOGI(TAG_BNO055_TASK, "Initializing BNO055 sensors \n");
    int8_t success = BNO055_Init(&gBNO055, BNO055_I2C_MASTER_SDA_GPIO, BNO055_I2C_MASTER_SCL_GPIO, BNO055_I2C_MASTER_NUM, BNO055_RST_GPIO);
    while (success != BNO055_SUCCESS) {
        wrap_printf("Error: Failed to initialize BNO055 sensor\n");
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
    gBNO055.is_calibrated = true;

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
        // wrap_printf("Acce: %.2f m^2\n", acce);
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
    ///< Initialize the VL53L1X sensors and set the parameters
    ESP_LOGI(TAG_VL53L1X_TASK, "Initializing VL53L1X sensor\n");

    // // ///< Create I2C bus and add the devices
    // // i2c_init_new_bus(&gVL53L1X[0].i2c_handle, VL53L1X_I2C_MASTER_NUM, VL53L1X_I2C_MASTER_SCL_GPIO, VL53L1X_I2C_MASTER_SDA_GPIO);
    // // gVL53L1X[1].i2c_handle.bus_handle = gVL53L1X[0].i2c_handle.bus_handle;
    // // gVL53L1X[2].i2c_handle.bus_handle = gVL53L1X[0].i2c_handle.bus_handle;
    // // for (int i = 0; i < 3; i++) {
    // //     i2c_init_new_device(&gVL53L1X[i].i2c_handle, VL53L1X_I2C_MASTER_NUM, VL53L1X_SENSOR_ADDR, I2C_MASTER_FREQ_HZ);
    // // }

    for (int i = 0; i < 3; i++) {
        if (!VL53L1X_init(&gVL53L1X[i], VL53L1X_I2C_MASTER_NUM, VL53L1X_I2C_MASTER_SCL_GPIO, VL53L1X_I2C_MASTER_SDA_GPIO, false)) {
            ESP_LOGE(TAG_VL53L1X_TASK, "VL53L1X sensor %d initialization failed\n", i);
            continue;
        }
        // VL53L1X_setAddress(&gVL53L1X[i], VL53L1X_SENSOR_ADDR + i + 1); ///< Set the address of the sensor
        VL53L1X_startContinuous(&gVL53L1X[i], 10); ///< Start continuous mode with a period of 10ms
        gVL53L1X[i].is_calibrated = true; ///< Set the flag to true to indicate that the sensor is calibrated
    }

    return true;
}

bool verify_sensors(uint32_t num_checks)
{
    ///< Count the number of times the sensors are sensing the same value
    int cnt_as5600[3] = {0};
    int cnt_vl53l1x[3] = {0};
    int cnt_bno055 = 0;

    float angle_prev[3] = {0};
    float dist_prev[3] = {0};
    float acce_prev = 0;

    float angle[3] = {0};
    float dist[3] = {0};
    float acce = 0;

    for (uint8_t i = 0; i < num_checks; i++) {
        ///< Get sensor values
        for (int j = 0; j < 3; j++) {
            angle[j] = AS5600_ADC_GetAngle(&gAS5600[j]); ///< Get the angle from the AS5600 sensor
        }

        for (int j = 0; j < 3; j++) {
            if (VL53L1X_dataReady(&gVL53L1X[j])) {            
                dist[j] = VL53L1X_readDistance(&gVL53L1X[j], false); // false for non-blocking read
            } else {
                dist[j] = dist_prev[j]; ///< If the data is not ready, use the previous value
            }
        }

        BNO055_ReadAll_Lineal(&gBNO055);
        acce = sqrt(gBNO055.ax*gBNO055.ax + gBNO055.ay*gBNO055.ay);

        ///< Print the values
        // wrap_printf("Angle: %.2f deg, Distance: %.3f m, Acce: %.2f m^2\n", angle, dist/1000.0, acce);

        ///< Check if the values are the same
        for (int j = 0; j < 3; j++) {
            if (fabs(angle[j] - angle_prev[j]) < 0.001) {
                cnt_as5600[j]++;
            }
            if (fabs(dist[j] - dist_prev[j]) < 0.001) {
                cnt_vl53l1x[j]++;
            }
        }
        if (fabs(acce - acce_prev) < 0.001) {
            cnt_bno055++;
        }

        ///< Update the previous values
        for (int j = 0; j < 3; j++) {
            angle_prev[j] = angle[j];
            dist_prev[j] = dist[j];
        }
        acce_prev = acce;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // ESP_LOGI("verify_sensors", "AS5600: %d, VL53L1X: %d, BNO055: %d", cnt_as5600, cnt_vl53l1x, cnt_bno055);

    // if (cnt_as5600 > num_checks/2 && cnt_vl53l1x > num_checks/2 && cnt_bno055 > num_checks/2) {
    //     return false;
    // }

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
            stop_robot();

            ///< Use the time for the sensor sampling and control the BLDC motor
            gSys.STATE = CHECK_SENSORS;
            for (int i = 0; i < 3; i++) {
                gMotor[i].is_calibrated = true;
            }

            ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.oneshot_timer, TIME_SAMPLING_US));
            break;

        case CHECK_SENSORS:
            // Check if the sensors are calibrated
            if (is_drivers_ready()) {
                gSys.STATE = SYS_SAMPLING_EXP; ///< Set the state to sampling
                // gSys.STATE = NONE; ///< Set the state to NONE
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
                stop_robot();
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
                stop_robot();
                ESP_LOGI("sys_timer_cb", "Samples readed from all the sensors: %d", gSys.cnt_smp_control);
                gSys.STATE = NONE; ///< Set the state to NONE to stop the system
            }
            break;

        default:
            ESP_LOGI("sys_timer_cb", "default");
            break;
    }
}

void parse_and_update_setpoints(const char* uart_buffer)
{
    char parsed[MAX_PARSED_LEN];
    float setpoints[3] = {0};

    // Find the "default" key in the string
    char *start = strstr(uart_buffer, "\"default\":\"");
    if (start) {
        start += strlen("\"default\":\""); // Move past the key

        // Find the closing quote of the value
        char *end = strchr(start, '"');
        if (end && (end - start) < sizeof(parsed)) {
            // Copy the value substring into a temporary buffer
            strncpy(parsed, start, end - start);
            parsed[end - start] = '\0';

            float sp0 = 0, sp1 = 0, sp2 = 0;

            // Parse three float values from the string
            if (sscanf(parsed, "%f %f %f", &sp0, &sp1, &sp2) == 3) {
                // Update the setpoints array
                xSemaphoreTake(gSys.mtx_cntrl, portMAX_DELAY); // Take the mutex to protect the control variables
                control_set_setpoint(&gCtrl[0], sp0);
                control_set_setpoint(&gCtrl[1], sp1);
                control_set_setpoint(&gCtrl[2], sp2);
                xSemaphoreGive(gSys.mtx_cntrl); // Release the mutex

                wrap_printf("Setpoints updated: %.2f, %.2f, %.2f\n", sp0, sp1, sp2);
            } else {
                wrap_printf("Failed to parse three float values.\n");
            }
        } else {
            wrap_printf("Invalid format: closing quote not found.\n");
        }
    } else {
        wrap_printf(" 'default' key not found.\n");
    }
}

void motor_identification_all(void)
{
    wrap_printf("Starting motor identification of all motors one by one...\n");

    const int num_motors = 3;
    const int num_samples = TIME_SAMP_MOTOR_S * SAMP_RATE_MOTOR_HZ;

    ///< Kalman filter variables for encoders
    kalman1D_t kalman_enc[3];
    for (int i = 0; i < 3; i++) {
        kalman1D_init(&kalman_enc[i], KALMAN_1D_ENC_Q, KALMAN_1D_ENC_R);
    }
    
    ///< Structure: row = muestra, columns = duty, velocity (rad/s) of each motor 1-2-3
    typedef struct {
        int duty;
        float speed[3];  // M1, M2, M3
    } motor_data_t;

    motor_data_t* data = calloc(num_samples, sizeof(motor_data_t));
    if (data == NULL) {
        wrap_printf("Error: no se pudo asignar memoria.\n");
        return;
    }

    ///< Timer wich will be used for all motors
    const esp_timer_create_args_t timer_args = {
        .callback = &motor_bldc_cb,
        .arg = NULL,
        .name = "bldc-ident"
    };
    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    gSys.timer_bldc = timer_handle;

    for (int motor_index = 0; motor_index < num_motors; motor_index++) {
        wrap_printf("\n⚙️  Identificando motor %d...\n", motor_index + 1);

        bldc_pwm_motor_t *motor = &gMotor[motor_index];
        AS5600_t *as5600 = &gAS5600[motor_index];
        bldc_set_duty_motor(motor, 0);

        ///< Iniciar timer periódico
        ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.timer_bldc, TIME_SAMP_MOTOR_US));

        for (int cnt = 0; cnt < num_samples; cnt++) {
            xSemaphoreTake(gSys.smph_bldc, portMAX_DELAY);

            int duty;
            if (cnt < 0.5 * SAMP_RATE_MOTOR_HZ) {
                duty = 7;
            } else if (cnt < 1.5 * SAMP_RATE_MOTOR_HZ) {
                duty = 20;
            } else if (cnt < 2.5 * SAMP_RATE_MOTOR_HZ) {
                duty = 30;
            } else if (cnt < 3 * SAMP_RATE_MOTOR_HZ) {
                duty = 0;
            } else if (cnt < 3.5 * SAMP_RATE_MOTOR_HZ) {
                duty = -7;
            } else if (cnt < 4.5 * SAMP_RATE_MOTOR_HZ) {
                duty = -20;
            } else if (cnt < 5 * SAMP_RATE_MOTOR_HZ) {
                duty = -30;
            } else {
                break;
            }

            bldc_set_duty_motor(motor, (float)duty);

            if (motor_index == 0) {
                data[cnt].duty = duty;  // Solo se asigna una vez en la primera ronda
            }

            float speed = calculate_motor_speed(&kalman_enc[motor_index], motor_index);
            data[cnt].speed[motor_index] = speed;
        }

        ESP_ERROR_CHECK(esp_timer_stop(gSys.timer_bldc));
        bldc_set_duty_motor(motor, 0);  // Seguridad: detener el motor al terminar
    }

    ///< Imprimir tabla
    wrap_printf("\n Resultados:\n");
    wrap_printf("Duty(%%)\tASpeed_M1\tSpeed_M2\tSpeed_M3\n");
    for (int i = 0; i < num_samples; i++) {
        wrap_printf("%d\t%.2f\t%.2f\t%.2f\n",
                    data[i].duty,
                    data[i].speed[0],
                    data[i].speed[1],
                    data[i].speed[2]);

        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    free(data);
}

void motor_bldc_cb(void *arg)
{
    BaseType_t mustYield = pdFALSE;
    xSemaphoreGiveFromISR(gSys.smph_bldc, &mustYield); 
}

bool is_drivers_ready(void)
{
    ///< Check if the drivers and sensors are ready to be used
    if (gMotor[0].is_calibrated && gMotor[1].is_calibrated && gMotor[2].is_calibrated &&
        gAS5600[0].is_calibrated && gAS5600[1].is_calibrated && gAS5600[2].is_calibrated &&
        gBNO055.is_calibrated && gVL53L1X[0].is_calibrated && gVL53L1X[1].is_calibrated && gVL53L1X[2].is_calibrated) {
        return true;
    }
    return false;
}

void stop_robot(void)
{
    for (int i = 0; i < 3; i++) {
        bldc_set_duty_motor(&gMotor[i], 0); ///< Stop the motor
        control_set_setpoint(&gCtrl[i], 0); ///< Set the setpoint to 0
    }
}

float calculate_motor_speed(kalman1D_t *kf, int midx)
{
    float raw = gAS5600[midx].angle_offset - AS5600_ADC_GetAngle(&gAS5600[midx]);
    float delta = raw - gAS5600[midx].angle_prev;

    if      (delta >  M_PI) delta -= 2 * M_PI;
    else if (delta < -M_PI) delta += 2 * M_PI;

    gAS5600[midx].angle_prev = raw; ///< Update the previous angle    

    return kalman1D_update(kf, delta * SAMP_RATE_MOTOR_HZ);
}

void calculate_motor_setpoints(float *w1, float *w2, float *w3)
{
    if (gTraj.ktime >= gTraj.k_duration) {
        *w1 = 0, *w2 = 0, *w3 = 0;
        xSemaphoreTake(gSys.mtx_cntrl, portMAX_DELAY); ///< Take the mutex to protect the control variables
        control_reset_pid(&gCtrl[0]); ///< Reset the controller for motor 1
        control_reset_pid(&gCtrl[1]); ///< Reset the controller for motor 2
        control_reset_pid(&gCtrl[2]); ///< Reset the controller for motor 3
        xSemaphoreGive(gSys.mtx_cntrl); ///< Release the mutex
        return;
    }
    
    float vbx = 0, vby = 0, wb = 0;

    switch (gTraj.cmd)
    {
    case C1_STRAIGHT:
        vbx = gTraj.speed * cos(gTraj.angle);
        vby = gTraj.speed * sin(gTraj.angle);
        wb = 0;
        calc_invkinematics(vbx, vby, wb, w1, w2, w3);
        break;

    case C2_ROTATION:
        vbx = 0;
        vby = 0;
        wb = gTraj.omega; ///< Angular velocity for rotation
        calc_invkinematics(vbx, vby, wb, w1, w2, w3);
        break;

    case C3_CIRCULAR:
        vbx = - gTraj.radius * gTraj.omega * sin(gTraj.omega * (float)gTraj.ktime/SAMP_RATE_MOTOR_HZ);
        vby =   gTraj.radius * gTraj.omega * cos(gTraj.omega * (float)gTraj.ktime/SAMP_RATE_MOTOR_HZ);
        wb = 0;
        calc_invkinematics(vbx, vby, wb, w1, w2, w3);
        break;
    
    default:
        *w1 = 0, *w2 = 0, *w3 = 0; ///< If no command is set, stop the motors
        break;
    }
    gTraj.ktime++;
}

void calculate_trajectory_params(uint8_t cmd, float angle, float speed, float dist_r, bool dir)
{
    gTraj.cmd = cmd; ///< Set the command
    gTraj.angle = angle; ///< Set the angle
    gTraj.dir = dir; ///< Set the direction

    switch (cmd)
    {
    case 1: ///< C1_STRAIGHT
        gTraj.cmd = C1_STRAIGHT; ///< Set the command to straight
        gTraj.speed = speed / 100; ///< Set the speed in m/s
        gTraj.distance = dist_r / 100; ///< Set the distance in meters
        gTraj.k_duration = (int)( (dist_r / speed) * SAMP_RATE_MOTOR_HZ); ///< Calculate the duration in samples
        break;

    case 2: ///< C2_ROTATION
        gTraj.cmd = C2_ROTATION; ///< Set the command to rotation
        gTraj.omega = speed;
        gTraj.k_duration = (int)( (angle / speed) * SAMP_RATE_MOTOR_HZ); ///< Calculate the duration in samples
        break;

    case 3: ///< C3_CIRCULAR
        gTraj.cmd = C3_CIRCULAR; ///< Set the command to circular
        gTraj.speed = speed / 100; 
        gTraj.radius = dist_r / 100; 
        gTraj.dir = dir;
        gTraj.omega = gTraj.speed / gTraj.radius; ///< Calculate the angular velocity
        gTraj.k_duration = (uint32_t)( (gTraj.angle * gTraj.radius / gTraj.speed) * SAMP_RATE_MOTOR_HZ); ///< Calculate the duration in samples
        break;

    default:
        gTraj.cmd = C_NONE;
        break;
    }
    ///< Print the trajectory parameters
    wrap_printf("Trajectory parameters: cmd: %d, angle: %.2f, speed: %.2f, omega: %.3f, distance: %.2f, radius: %.2f, dir: %d, k_duration: %d\n",
                gTraj.cmd, gTraj.angle, gTraj.speed, gTraj.omega, gTraj.distance, gTraj.radius, gTraj.dir, gTraj.k_duration);
    
    gTraj.ktime = 0; ///< Reset the time counter

    ///< Reset the controllers
    xSemaphoreTake(gSys.mtx_cntrl, portMAX_DELAY); ///< Take the mutex to protect the control variables
    for (int i = 0; i < 3; i++) {
        control_reset_pid(&gCtrl[i]); ///< Reset the controller
    }
    xSemaphoreGive(gSys.mtx_cntrl); ///< Release the mutex
}

bool parse_command(const char* input, size_t len)
{
    char buffer[128];
    if (len >= sizeof(buffer)) return false;

    strncpy(buffer, input, len);
    buffer[len] = '\0';

    char* token = strtok(buffer, ",");
    if (!token) return false;

    // Local variables to store parsed values
    float angle = 0.0f;
    float speed = 0.0f;
    float dist_or_radius = 0.0f;
    bool dir = false; // false = CCW, true = CW
    uint8_t cmd = 0;

    if (strcmp(token, "C1") == 0) 
    {
        cmd = 1; // C1_STRAIGHT

        // Angle (degrees)
        token = strtok(NULL, ",");
        if (!token) return false;
        angle = atof(token);

        // Speed (cm/s)
        token = strtok(NULL, ",");
        if (!token) return false;
        speed = atof(token);

        // Distance (cm)
        token = strtok(NULL, ",");
        if (!token) return false;
        dist_or_radius = atof(token);

        printf("C1 -> angle: %.2f deg, speed: %.2f cm/s, distance: %.2f cm\n",
               angle, speed, dist_or_radius);
    }
    else if (strcmp(token, "C2") == 0)
    {
        cmd = 2; // C2_ROTATION

        // Angle (degrees)
        token = strtok(NULL, ",");
        if (!token) return false;
        angle = atof(token);

        // Speed (rad/s)
        token = strtok(NULL, ",");
        if (!token) return false;
        speed = atof(token);

        printf("C2 -> angle: %.2f deg, speed: %.2f rad/s\n",
               angle, speed);
    }
    else if (strcmp(token, "C3") == 0)
    {
        cmd = 3; // C3_CIRCULAR

        // Direction
        token = strtok(NULL, ",");
        if (!token) return false;
        if (strcmp(token, "CW") == 0)
            dir = true;
        else if (strcmp(token, "CCW") == 0)
            dir = false;
        else
            return false;

        // Radius (cm)
        token = strtok(NULL, ",");
        if (!token) return false;
        dist_or_radius = atof(token);

        // Speed (cm/s)
        token = strtok(NULL, ",");
        if (!token) return false;
        speed = atof(token);

        // Angle (degrees)
        token = strtok(NULL, ",");
        if (!token) return false;
        angle = atof(token);

        printf("C3 -> direction: %s, radius: %.2f cm, speed: %.2f cm/s, angle: %.2f deg\n",
               dir ? "CW" : "CCW", dist_or_radius, speed, angle);
    }
    else
    {
        return false; // Unknown command
    }

    // Update trajectory parameters
    calculate_trajectory_params(cmd, angle * M_PI/180, speed, dist_or_radius, dir);

    return true;
}

void wrap_printf(const char *format, ...)
{
    va_list args;

    xSemaphoreTake(gSys.mtx_printf, portMAX_DELAY);

    va_start(args, format);
    vprintf(format, args); ///< Use vprintf to handle formatted output
    va_end(args);
    fflush(stdout); ///< Ensure the output is flushed to the console

    xSemaphoreGive(gSys.mtx_printf); ///< Release the mutex after printing
}

